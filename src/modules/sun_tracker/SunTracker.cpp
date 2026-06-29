/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "SunTracker.hpp"
#include "solar_position.hpp"

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <string.h>

// Wing tilt axis selection (SUN_AXIS).
enum class TiltAxis : int32_t {
	BodyY = 0, // pitch: panel normal sweeps the body X-Z plane (default)
	BodyX = 1, // roll:  panel normal sweeps the body Y-Z plane
};

SunTracker::SunTracker() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

SunTracker::~SunTracker()
{
	perf_free(_loop_perf);
}

bool SunTracker::init()
{
	// SUN_GEAR_RATIO is fixed mechanical configuration (reboot_required), not a runtime knob.
	// Resolve the encoder->wing conversion factor once here so the control loop never re-reads it.
	const float gear_ratio = _param_sun_gear.get();
	_encoder_to_wing = (gear_ratio > 0.01f) ? (1.f / gear_ratio) : 1.f;

	// Slow auxiliary loop: 20 Hz is far faster than wing-tilt dynamics need and keeps
	// the rate of DO_SET_ACTUATOR commands (which commander ACKs) modest.
	ScheduleOnInterval(50_ms);
	return true;
}

void SunTracker::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}
}

float SunTracker::desiredTiltFromSun(float sun_body_x, float sun_body_y, float sun_body_z) const
{
	// At tilt = 0 the panel is level (normal = body -Z, i.e. up). The tilt angle that points
	// the panel normal at the sun is the bearing of the sun's projection onto the rotation plane.
	// SUN_TILT_REV negates the result to match the physical wing/encoder mounting (see module.yaml).
	const float dir = _param_sun_tilt_rev.get() ? -1.f : 1.f;

	switch ((TiltAxis)_param_sun_axis.get()) {
	case TiltAxis::BodyX:
		return dir * atan2f(sun_body_y, -sun_body_z);

	case TiltAxis::BodyY:
	default:
		return dir * atan2f(sun_body_x, -sun_body_z);
	}
}

uint64_t SunTracker::resolveUtcUsec(const sensor_gps_s &gps, bool have_gps) const
{
	if (have_gps && gps.time_utc_usec > 0) {
		return gps.time_utc_usec;
	}

	// Fall back to the system realtime clock (set from GPS on hardware, host wall-clock in
	// SITL). Require a plausibly-current time (after ~2020-09) so a default 1970 clock can't
	// drive a bogus sun position.
	timespec ts{};

	if (px4_clock_gettime(CLOCK_REALTIME, &ts) == 0 && ts.tv_sec > 1600000000) {
		return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
	}

	return 0;
}

void SunTracker::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	parameters_update();

	// >>> TEMP DEBUG SWEEP: lets the bench sweep run even when tracking is disabled. <<<
	const bool debug_sweep = _param_sun_dbg_swp.get();

	// Park-to-zero (console `park` command). Runs even when tracking is disabled so the wing can be
	// stowed level before power-off without needing a fix / SUN_TRK_EN.
	const bool park = _park_request.load();

	if (!_param_sun_trk_en.get() && !debug_sweep && !park) {
		// Disabled: leave the ESC at its configured disarmed/failsafe value.
		_status = "disabled";
		publishStatus(false, false, false, false, false);
		perf_end(_loop_perf);
		return;
	}

	vehicle_attitude_s att{};
	vehicle_global_position_s gpos{};
	vehicle_local_position_s lpos{};
	sensor_gps_s gps{};
	sensor_encoder_s enc{};

	const bool have_att = _vehicle_attitude_sub.copy(&att);
	const bool have_gpos = _vehicle_global_position_sub.copy(&gpos);
	const bool have_lpos = _vehicle_local_position_sub.copy(&lpos);
	const bool have_gps = _sensor_gps_sub.copy(&gps);
	const bool have_enc = _sensor_encoder_sub.copy(&enc);

	const uint64_t utc_usec = resolveUtcUsec(gps, have_gps);

	// Heading must be referenced to true north for the azimuth projection below to be valid; an
	// unreferenced yaw (no magnetometer / no GPS heading) would point the wing at the wrong bearing.
	// Gate on the estimator heading variance rather than vehicle_local_position.heading_good_for_control:
	// that flag only asserts after an in-flight magnetometer alignment and so never becomes true on the
	// ground, whereas this tracker runs stationary. A bounded heading_var only occurs once yaw is aided
	// by an absolute reference (mag / GPS yaw), so the threshold doubles as a "yaw is aligned" check.
	const bool time_valid = (utc_usec > 0);
	const bool pos_valid = have_gpos && gpos.lat_lon_valid;
	const bool heading_valid = have_lpos && PX4_ISFINITE(lpos.heading) && (lpos.heading_var > 0.f)
				   && (lpos.heading_var <= _param_sun_hdg_var_max.get());

	if (have_lpos) {
		_heading = lpos.heading;
		_heading_var = lpos.heading_var;
	}

	bool pose_ok = have_att && pos_valid && heading_valid && time_valid;

	// Compute the sun direction and tilt setpoint whenever we have pose + time, even if the
	// encoder is missing, so status/diagnostics stay informative while gated.
	float tilt_setpoint = _param_sun_park.get();

	if (pose_ok) {
		float sun_az = 0.f;
		float sun_el = 0.f;
		solar::solar_position(utc_usec, gpos.lat, gpos.lon, sun_az, sun_el);
		_sun_az = sun_az;
		_sun_el = sun_el;

		if (sun_el > 0.f) {
			const matrix::Vector3f sun_ned{cosf(sun_el) *cosf(sun_az), cosf(sun_el) *sinf(sun_az), -sinf(sun_el)};

			// Rotate the sun vector from NED into the body (FRD) frame. Dcm(q) maps body->NED.
			const matrix::Dcmf R_body_to_ned{matrix::Quatf(att.q)};
			const matrix::Vector3f sun_body = R_body_to_ned.transpose() * sun_ned;

			tilt_setpoint = desiredTiltFromSun(sun_body(0), sun_body(1), sun_body(2)) + _param_sun_tilt_off.get();
		}

		tilt_setpoint = math::constrain(tilt_setpoint, _param_sun_tilt_min.get(), _param_sun_tilt_max.get());
		_tilt_setpoint = tilt_setpoint;
	}

	// >>> TEMP DEBUG SWEEP: override the setpoint with a +/-SUN_DBG_AMP sine sweep and bypass the
	//     pose/time gate, so the wing oscillates on the bench without a GPS fix. The encoder loop
	//     below is unchanged, so feedback + ESC are still exercised. Remove this whole block. <<<
	if (debug_sweep) {
		static constexpr float kSweepPeriodUs = 4e6f; // 4 s per full cycle
		const float amp = math::radians(_param_sun_dbg_amp.get());
		const float phase = (float)(hrt_absolute_time() % (uint64_t)kSweepPeriodUs) / kSweepPeriodUs;
		tilt_setpoint = amp * sinf(2.f * (float)M_PI * phase);
		tilt_setpoint = math::constrain(tilt_setpoint, _param_sun_tilt_min.get(), _param_sun_tilt_max.get());
		_tilt_setpoint = tilt_setpoint;
		pose_ok = true;
	}

	// <<< TEMP DEBUG SWEEP

	// Park overrides tracking and the debug sweep: drive the wing to the boot-time zero and hold it
	// there. Closing the loop to zero only needs the encoder, so bypass the pose/time/heading gate
	// (the encoder gate below still applies). Hold until released with `park off` so the wing stays
	// level through power-off, regardless of whether the drive holds position passively.
	if (park) {
		tilt_setpoint = math::constrain(0.f, _param_sun_tilt_min.get(), _param_sun_tilt_max.get());
		_tilt_setpoint = tilt_setpoint;
		pose_ok = true;
	}

	const bool encoder_valid = have_enc && enc.valid;

	if (!pose_ok || !encoder_valid) {
		// Hold a safe neutral command and reset the integrator while we lack a prerequisite. Report
		// the specific missing input so a stuck tracker is easy to diagnose (no mag -> wait: heading).
		if (!pose_ok) {
			if (!have_att) {
				_status = "wait: attitude";

			} else if (!time_valid) {
				_status = "wait: time";

			} else if (!pos_valid) {
				_status = "wait: position";

			} else { // !heading_valid
				_status = "wait: heading";
			}

		} else {
			_status = "wait: encoder";
		}

		_integral = 0.f;
		_last_error_valid = false;
		_output = 0.f;
		publishActuator(0.f);
		publishStatus(_param_sun_trk_en.get(), debug_sweep, pose_ok, heading_valid, encoder_valid);
		perf_end(_loop_perf);
		return;
	}

	_status = park ? "parking" : (debug_sweep ? "DEBUG sweep" : "tracking"); // TEMP DEBUG SWEEP

	// The AS5600 is geared to the wing shaft, so it rotates faster than the wing. Scale the measured
	// encoder angle back into the wing frame using the factor resolved at init (1 / SUN_GEAR_RATIO),
	// so the setpoint, tilt limits and deadband all stay in real wing angle. The wing range keeps the
	// geared encoder angle within +/-pi, so the driver's wrap before scaling is safe here.
	const float measured_wing = enc.angle * _encoder_to_wing;
	_measured_angle = measured_wing;

	// Position error (wrapped) against the de-geared encoder feedback.
	const float error = matrix::wrap_pi(tilt_setpoint - measured_wing);

	const hrt_abstime now = hrt_absolute_time();
	float dt = (_last_run > 0) ? (now - _last_run) * 1e-6f : 0.05f;
	dt = math::constrain(dt, 0.001f, 0.2f);
	_last_run = now;

	// Deadband to avoid ESC dither when we are essentially on target.
	const float error_eff = (fabsf(error) < _param_sun_deadband.get()) ? 0.f : error;

	// PID with integral anti-windup (clamped so the integral term alone cannot saturate output).
	_integral = math::constrain(_integral + _param_sun_ki.get() * error_eff * dt, -1.f, 1.f);

	float derivative = 0.f;

	if (_last_error_valid) {
		derivative = (error_eff - _last_error) / dt;
	}

	_last_error = error_eff;
	_last_error_valid = true;

	const float u = math::constrain(_param_sun_kp.get() * error_eff + _integral + _param_sun_kd.get() * derivative,
					-1.f, 1.f);

	_error = error;
	_output = u;

	publishActuator(u);
	publishStatus(_param_sun_trk_en.get(), debug_sweep, true, heading_valid, true);

	perf_end(_loop_perf);
}

void SunTracker::publishActuator(float value)
{
	const hrt_abstime now = hrt_absolute_time();

	// Only emit a command when the value changed meaningfully, plus a slow heartbeat. This
	// keeps the steady-state rate of commander-ACK'd DO_SET_ACTUATOR commands low.
	const bool changed = !PX4_ISFINITE(_last_output) || (fabsf(value - _last_output) > 0.005f);
	const bool heartbeat = (now - _last_publish) > 200_ms;

	if (!changed && !heartbeat) {
		return;
	}

	vehicle_command_s cmd{};
	cmd.timestamp = now;
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;
	cmd.param1 = value; // -> Peripheral_via_Actuator_Set1 (PWM_AUX_FUNCx = 301)
	cmd.param2 = NAN;
	cmd.param3 = NAN;
	cmd.param4 = NAN;
	cmd.param5 = NAN;
	cmd.param6 = NAN;
	cmd.param7 = 0.f;   // actuator set index
	cmd.from_external = false;
	_vehicle_command_pub.publish(cmd);

	_last_output = value;
	_last_publish = now;
}

void SunTracker::publishStatus(bool enabled, bool debug_sweep, bool pose_valid, bool heading_valid,
			       bool encoder_valid)
{
	sun_tracker_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.sun_azimuth = _sun_az;
	status.sun_elevation = _sun_el;
	status.heading = _heading;
	status.heading_var = _heading_var;
	status.tilt_setpoint = _tilt_setpoint;
	status.measured_angle = _measured_angle;
	status.error = _error;
	status.output = _output;
	status.enabled = enabled;
	status.debug_sweep = debug_sweep;
	status.pose_valid = pose_valid;
	status.heading_valid = heading_valid;
	status.encoder_valid = encoder_valid;
	_sun_tracker_status_pub.publish(status);
}

int SunTracker::print_status()
{
	const float r2d = 180.f / (float)M_PI;
	PX4_INFO("state: %s (axis %d)", _status, (int)_param_sun_axis.get());
	PX4_INFO("sun:   az %6.1f deg  el %6.1f deg", (double)(_sun_az * r2d), (double)(_sun_el * r2d));
	PX4_INFO("head:  yaw %6.1f deg  var %.4f rad^2 (max %.4f)", (double)(_heading * r2d),
		 (double)_heading_var, (double)_param_sun_hdg_var_max.get());
	PX4_INFO("tilt:  setpoint %6.1f deg  measured %6.1f deg  error %6.1f deg",
		 (double)(_tilt_setpoint * r2d), (double)(_measured_angle * r2d), (double)(_error * r2d));
	PX4_INFO("out:   u %.3f  (last published %.3f)", (double)_output, (double)_last_output);
	perf_print_counter(_loop_perf);
	return 0;
}

int SunTracker::task_spawn(int argc, char *argv[])
{
	SunTracker *instance = new SunTracker();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int SunTracker::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (argc >= 1 && strcmp(argv[0], "park") == 0) {
		const bool engage = !(argc >= 2 && strcmp(argv[1], "off") == 0);
		SunTracker *inst = get_instance();

		if (inst) {
			inst->_park_request.store(engage);
			PX4_INFO(engage ? "park: driving wing to zero and holding (use `park off` to release)"
				 : "park: released");
			return 0;
		}

		return 1;
	}

	return print_usage("unknown command");
}

int SunTracker::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Sun-tracking tilt-wing controller. Computes the sun direction from GPS position and UTC time,
expresses it in the body frame via the vehicle attitude, derives the wing tilt angle that points
the solar panel normal at the sun, and closes a position loop on the AS5600 encoder feedback
(`sensor_encoder`). The control effort drives a dedicated reversible ESC through
`vehicle_command`/`DO_SET_ACTUATOR` (Peripheral_via_Actuator_Set1, mapped with `PWM_AUX_FUNCx = 301`).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sun_tracker", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("park", "Drive the wing to the boot-time zero and hold it level (for stowing before power-off)");
	PRINT_MODULE_USAGE_ARG("off", "Release a park request and resume normal tracking", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sun_tracker_main(int argc, char *argv[])
{
	return SunTracker::main(argc, argv);
}
