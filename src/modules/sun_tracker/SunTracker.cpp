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
	switch ((TiltAxis)_param_sun_axis.get()) {
	case TiltAxis::BodyX:
		return atan2f(sun_body_y, -sun_body_z);

	case TiltAxis::BodyY:
	default:
		return atan2f(sun_body_x, -sun_body_z);
	}
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

	if (!_param_sun_trk_en.get()) {
		// Disabled: leave the ESC at its configured disarmed/failsafe value.
		perf_end(_loop_perf);
		return;
	}

	vehicle_attitude_s att{};
	vehicle_global_position_s gpos{};
	sensor_gps_s gps{};
	sensor_encoder_s enc{};

	const bool have_att = _vehicle_attitude_sub.copy(&att);
	const bool have_gpos = _vehicle_global_position_sub.copy(&gpos);
	const bool have_gps = _sensor_gps_sub.copy(&gps);
	const bool have_enc = _sensor_encoder_sub.copy(&enc);

	const bool inputs_ok = have_att && have_gpos && gpos.lat_lon_valid
			       && have_gps && (gps.time_utc_usec > 0)
			       && have_enc && enc.valid;

	if (!inputs_ok) {
		// Hold a safe neutral command and reset the integrator while we have no valid feedback.
		_integral = 0.f;
		_last_error_valid = false;
		publishActuator(0.f);
		perf_end(_loop_perf);
		return;
	}

	// Sun direction in the local NED frame.
	float sun_az = 0.f;
	float sun_el = 0.f;
	solar::solar_position(gps.time_utc_usec, gpos.lat, gpos.lon, sun_az, sun_el);

	float tilt_setpoint = _param_sun_park.get();

	if (sun_el > 0.f) {
		const matrix::Vector3f sun_ned{cosf(sun_el) *cosf(sun_az), cosf(sun_el) *sinf(sun_az), -sinf(sun_el)};

		// Rotate the sun vector from NED into the body (FRD) frame. Dcm(q) maps body->NED.
		const matrix::Dcmf R_body_to_ned{matrix::Quatf(att.q)};
		const matrix::Vector3f sun_body = R_body_to_ned.transpose() * sun_ned;

		tilt_setpoint = desiredTiltFromSun(sun_body(0), sun_body(1), sun_body(2)) + _param_sun_tilt_off.get();
	}

	tilt_setpoint = math::constrain(tilt_setpoint, _param_sun_tilt_min.get(), _param_sun_tilt_max.get());

	// Position error (wrapped) against the encoder feedback.
	const float error = matrix::wrap_pi(tilt_setpoint - enc.angle);

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

	publishActuator(u);

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
	cmd.param1 = value; // -> Peripheral_via_Actuator_Set1 (PWM_MAIN_FUNCx = 301)
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

int SunTracker::print_status()
{
	perf_print_counter(_loop_perf);
	PX4_INFO("enabled: %s, last output: %.3f", _param_sun_trk_en.get() ? "yes" : "no", (double)_last_output);
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
`vehicle_command`/`DO_SET_ACTUATOR` (Peripheral_via_Actuator_Set1, mapped with `PWM_MAIN_FUNCx = 301`).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sun_tracker", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int sun_tracker_main(int argc, char *argv[])
{
	return SunTracker::main(argc, argv);
}
