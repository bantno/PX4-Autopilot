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
	// Slow auxiliary loop: 20 Hz matches the wing_tilt controller consuming our setpoints.
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
		// Disabled: publish no setpoint — the wing_tilt controller stops the wing once our
		// last setpoint goes stale.
		_status = "disabled";
		publishStatus(false, false, false, false);
		perf_end(_loop_perf);
		return;
	}

	vehicle_attitude_s att{};
	vehicle_global_position_s gpos{};
	vehicle_local_position_s lpos{};
	sensor_gps_s gps{};

	const bool have_att = _vehicle_attitude_sub.copy(&att);
	const bool have_gpos = _vehicle_global_position_sub.copy(&gpos);
	const bool have_lpos = _vehicle_local_position_sub.copy(&lpos);
	const bool have_gps = _sensor_gps_sub.copy(&gps);

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

	if (!pose_ok) {
		// Publish no setpoint while a prerequisite is missing (the wing_tilt controller stops
		// the wing once ours goes stale). Report the specific missing input so a stuck tracker
		// is easy to diagnose (no mag -> wait: heading).
		if (!have_att) {
			_status = "wait: attitude";

		} else if (!time_valid) {
			_status = "wait: time";

		} else if (!pos_valid) {
			_status = "wait: position";

		} else { // !heading_valid
			_status = "wait: heading";
		}

		publishStatus(_param_sun_trk_en.get(), debug_sweep, pose_ok, heading_valid);
		perf_end(_loop_perf);
		return;
	}

	_status = park ? "parking" : (debug_sweep ? "DEBUG sweep" : "tracking"); // TEMP DEBUG SWEEP

	// Hand the wing-angle setpoint to the wing_tilt controller — the single owner of the tilt
	// motor. It closes the encoder loop and arbitrates against higher-priority sources
	// (self_right, console holds); we keep ownership by republishing every cycle.
	wing_tilt_setpoint_s sp{};
	sp.timestamp = hrt_absolute_time();
	sp.source = wing_tilt_setpoint_s::SOURCE_SUN_TRACKER;
	sp.angle = tilt_setpoint;
	_wing_tilt_setpoint_pub.publish(sp);

	publishStatus(_param_sun_trk_en.get(), debug_sweep, true, heading_valid);

	perf_end(_loop_perf);
}

void SunTracker::publishStatus(bool enabled, bool debug_sweep, bool pose_valid, bool heading_valid)
{
	sun_tracker_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.sun_azimuth = _sun_az;
	status.sun_elevation = _sun_el;
	status.heading = _heading;
	status.heading_var = _heading_var;
	status.tilt_setpoint = _tilt_setpoint;
	status.enabled = enabled;
	status.debug_sweep = debug_sweep;
	status.pose_valid = pose_valid;
	status.heading_valid = heading_valid;
	_sun_tracker_status_pub.publish(status);
}

int SunTracker::print_status()
{
	const float r2d = 180.f / (float)M_PI;
	PX4_INFO("state: %s (axis %d)", _status, (int)_param_sun_axis.get());
	PX4_INFO("sun:   az %6.1f deg  el %6.1f deg", (double)(_sun_az * r2d), (double)(_sun_el * r2d));
	PX4_INFO("head:  yaw %6.1f deg  var %.4f rad^2 (max %.4f)", (double)(_heading * r2d),
		 (double)_heading_var, (double)_param_sun_hdg_var_max.get());
	PX4_INFO("tilt:  setpoint %6.1f deg (loop closed by wing_tilt)", (double)(_tilt_setpoint * r2d));
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
