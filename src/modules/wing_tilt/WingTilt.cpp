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

#include "WingTilt.hpp"

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <float.h>
#include <string.h>

WingTilt::WingTilt() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

WingTilt::~WingTilt()
{
	perf_free(_loop_perf);
}

bool WingTilt::init()
{
	// TILT_GEAR is fixed mechanical configuration (reboot_required), not a runtime knob.
	// Resolve the encoder->wing conversion factor once so the control loop never re-reads it.
	const float gear_ratio = _param_tilt_gear.get();
	_encoder_to_wing = (gear_ratio > 0.01f) ? (1.f / gear_ratio) : 1.f;

	// 20 Hz is far faster than wing-tilt dynamics need and keeps the rate of
	// DO_SET_ACTUATOR commands (which commander ACKs) modest.
	ScheduleOnInterval(50_ms);
	return true;
}

void WingTilt::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}
}

void WingTilt::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	parameters_update();

	// Drain the setpoint queue, keeping the latest per source.
	wing_tilt_setpoint_s sp;

	while (_wing_tilt_setpoint_sub.update(&sp)) {
		if (sp.source < NUM_SOURCES) {
			// The wing wiring only permits half a revolution either side of center: clamp
			// every setpoint to [-pi, pi] wing angle.
			sp.angle = math::constrain(sp.angle, -(float)M_PI, (float)M_PI);
			_setpoints[sp.source] = sp;
		}
	}

	const hrt_abstime now = hrt_absolute_time();

	// ESC arming sequence: run below-neutral -> above-neutral -> neutral so the reversible ESC
	// arms. The outputs go live (COM_PREARM_MODE 2) as soon as commander first publishes —
	// seconds after battery plug-in, while the ESC is still in its own power-on init and would
	// miss the sweep — so it is scheduled TILT_ARM_DLY after the live transition; the pin holds
	// neutral until then. Setpoints are not serviced until the sequence completes, and it can be
	// re-run on demand with `wing_tilt esc_arm`.
	actuator_armed_s armed;

	if (_actuator_armed_sub.copy(&armed)) {
		const bool live = armed.armed || armed.prearmed;

		if (live && !_outputs_live_prev && fabsf(_param_tilt_arm_v.get()) > FLT_EPSILON) {
			_esc_arm_scheduled = now + (hrt_abstime)(math::max(_param_tilt_arm_dly.get(), 0.f) * 1e6f);
		}

		_outputs_live_prev = live;
	}

	if (_esc_arm_request.load()) {
		_esc_arm_request.store(false);
		_esc_arm_scheduled = now;
	}

	if (_esc_arm_scheduled != 0 && now >= _esc_arm_scheduled) {
		_esc_arm_scheduled = 0;
		_esc_arm_start = now;
		_integral = 0.f;
		_last_error_valid = false;
	}

	if (_esc_arm_start != 0) {
		// Sinusoidal wiggle about neutral (steps don't register with this ESC's arming logic):
		// TILT_ARM_N full cycles of period TILT_ARM_T at amplitude |TILT_ARM_V|; positive V
		// swings below neutral first, negative above. Sampled at the 20 Hz loop rate.
		const float t = (now - _esc_arm_start) * 1e-6f;
		const float period = math::max(_param_tilt_arm_t.get(), 0.1f);
		const float v = _param_tilt_arm_v.get();
		const float duration = period * math::max((float)_param_tilt_arm_n.get(), 1.f);

		if (t < duration) {
			publishActuator(-v * sinf(2.f * M_PI_F * t / period));

		} else {
			publishActuator(0.f);
			_esc_arm_start = 0;

			// Once the ESC arms mid-wiggle the remaining cycles physically move the wing:
			// actively drive it back to the boot-zero position.
			_recenter = true;
			_recenter_start = now;
		}
	}

	if (_esc_arm_start != 0 || _esc_arm_scheduled != 0) {
		// Sequence pending or running: the ESC is not armed yet, hold off setpoint servicing.
		_active_source = wing_tilt_status_s::SOURCE_NONE;
		_setpoint = NAN;
		_error = NAN;
		_output = _last_output;
		publishStatus();
		perf_end(_loop_perf);
		return;
	}

	// Arbitrate: highest-priority source with a fresh setpoint owns the wing.
	int active = -1;

	for (int s = NUM_SOURCES - 1; s >= 0; s--) {
		if (_setpoints[s].timestamp != 0 && (now - _setpoints[s].timestamp) < SETPOINT_TIMEOUT) {
			active = s;
			break;
		}
	}

	sensor_encoder_s enc;
	const bool encoder_ok = _sensor_encoder_sub.copy(&enc)
				&& enc.valid && (now - enc.timestamp) < ENCODER_TIMEOUT;

	_encoder_valid = encoder_ok;
	_measured_angle = encoder_ok ? (enc.angle * _encoder_to_wing) : NAN;

	// Post-arm recenter ends when a real source takes over, the wing is back near zero, or the
	// timeout expires (never chase an unreachable target forever).
	if (_recenter
	    && (active >= 0
		|| (encoder_ok && fabsf(_measured_angle) < RECENTER_TOL)
		|| (now - _recenter_start) > RECENTER_TIMEOUT)) {
		_recenter = false;
	}

	const bool recentering = _recenter && encoder_ok;

	if ((active < 0 && !recentering) || !encoder_ok) {
		// No owner, or no trustworthy feedback: stop the wing (single zero command — the
		// actuator is a rate plant) and reset the loop.
		if (_was_driving) {
			publishActuator(0.f);
			_integral = 0.f;
			_last_error_valid = false;
			_was_driving = false;
		}

		_active_source = wing_tilt_status_s::SOURCE_NONE;
		_setpoint = NAN;
		_error = NAN;
		_output = 0.f;
		publishStatus();
		perf_end(_loop_perf);
		return;
	}

	_was_driving = true;
	_active_source = (active >= 0) ? (uint8_t)active : wing_tilt_status_s::SOURCE_RECENTER;
	_setpoint = (active >= 0) ? _setpoints[active].angle : 0.f;

	if (active < 0) {
		// Recenter: constant-magnitude drive back to boot-zero, not the position PID — at the
		// few degrees of error the wiggle leaves, the PID commands far less than the ESC's
		// motion threshold and the wing never moves. This is a coarse park, not tracking; the
		// stop condition above (|angle| < RECENTER_TOL) ends it.
		_error = -_measured_angle;
		_output = (_error > 0.f) ? RECENTER_DRIVE : -RECENTER_DRIVE;
		publishActuator(_output);
		publishStatus();
		perf_end(_loop_perf);
		return;
	}

	float dt = (_last_run > 0) ? (now - _last_run) * 1e-6f : 0.05f;
	dt = math::constrain(dt, 0.001f, 0.2f);
	_last_run = now;

	// Position error against the de-geared encoder feedback. Deliberately NOT wrapped: the
	// wing wiring only permits +/- half a revolution from center, so the controller must
	// always drive through zero — never the short path across the +/-180 deg seam.
	const float error = _setpoint - _measured_angle;

	// Deadband to avoid ESC dither when we are essentially on target.
	const float error_eff = (fabsf(error) < _param_tilt_db.get()) ? 0.f : error;

	float derivative = 0.f;

	if (_last_error_valid) {
		derivative = (error_eff - _last_error) / dt;
	}

	_last_error = error_eff;
	_last_error_valid = true;

	const float u_unsat = _param_tilt_kp.get() * error_eff + _integral
			      + _param_tilt_kd.get() * derivative;
	const float u = math::constrain(u_unsat, -1.f, 1.f);

	// Anti-windup (conditional integration): while the output is saturated — e.g. the whole
	// 0 -> 90 deg transit, where P alone pins the command at full — integrating the large error
	// would only store surplus command that overshoots the target and takes tens of seconds to
	// bleed off. Freeze the integrator when saturated, unless the error is actively unwinding
	// it. The integral's job is the steady-state hold (gravity droop) once the loop is linear.
	const bool saturated = fabsf(u_unsat) >= 1.f;

	if (!saturated || (error_eff * u_unsat < 0.f)) {
		_integral = math::constrain(_integral + _param_tilt_ki.get() * error_eff * dt, -1.f, 1.f);
	}

	_error = error;
	_output = u;

	publishActuator(u);
	publishStatus();
	perf_end(_loop_perf);
}

void WingTilt::publishActuator(float value)
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

void WingTilt::publishStatus()
{
	wing_tilt_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.source = _active_source;
	status.setpoint = _setpoint;
	status.measured_angle = _measured_angle;
	status.error = _error;
	status.output = _output;
	status.encoder_valid = _encoder_valid;
	_wing_tilt_status_pub.publish(status);
}

int WingTilt::print_status()
{
	const float r2d = 180.f / (float)M_PI;
	const char *source_names[NUM_SOURCES] = {"sun_tracker", "console", "self_right"};
	PX4_INFO("owner: %s  encoder: %s%s",
		 (_active_source < NUM_SOURCES) ? source_names[_active_source]
		 : (_active_source == wing_tilt_status_s::SOURCE_RECENTER) ? "recenter" : "none",
		 _encoder_valid ? "ok" : "invalid/stale",
		 (_esc_arm_start != 0) ? "  [ESC arming sequence running]"
		 : (_esc_arm_scheduled != 0) ? "  [ESC arming sequence scheduled]" : "");
	PX4_INFO("tilt: setpoint %6.1f deg  measured %6.1f deg  error %6.1f deg  u %.3f",
		 (double)(_setpoint * r2d), (double)(_measured_angle * r2d),
		 (double)(_error * r2d), (double)_output);
	perf_print_counter(_loop_perf);
	return 0;
}

int WingTilt::task_spawn(int argc, char *argv[])
{
	WingTilt *instance = new WingTilt();

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

int WingTilt::custom_command(int argc, char *argv[])
{
	if (argc >= 1 && strcmp(argv[0], "esc_arm") == 0) {
		WingTilt *inst = get_instance();

		if (!is_running() || inst == nullptr) {
			PX4_ERR("not running");
			return PX4_ERROR;
		}

		if (fabsf(inst->_param_tilt_arm_v.get()) < FLT_EPSILON) {
			PX4_ERR("esc_arm: sequence disabled (TILT_ARM_V is 0)");
			return PX4_ERROR;
		}

		inst->_esc_arm_request.store(true);
		PX4_INFO("esc_arm: running the ESC arming sequence");
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int WingTilt::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Wing tilt controller — the single owner of the tilt motor. Closes the position loop between the
wing encoder (sensor_encoder, de-geared by 1/TILT_GEAR) and the reversible tilt ESC
(DO_SET_ACTUATOR). Client modules (sun_tracker, self_right, console holds) publish wing-angle
setpoints on wing_tilt_setpoint; the freshest setpoint from the highest-priority source wins
(self_right > console > sun_tracker), and a source releases the wing by not republishing for
0.5 s. With no owner or no valid encoder the wing is stopped with a neutral command.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wing_tilt", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("esc_arm", "Re-run the reversible-ESC arming wiggle "
					 "(sine about neutral; e.g. after power-cycling the ESC)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int wing_tilt_main(int argc, char *argv[])
{
	return WingTilt::main(argc, argv);
}
