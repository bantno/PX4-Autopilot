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

#include "SelfRight.hpp"

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <parameters/param.h>

// Righting strategy (SR_STRATEGY).
enum class Strategy : int32_t {
	OpenLoop = 0,     // replay/ramp a thrust profile, no in-loop attitude feedback
	AttitudePID = 1,  // close on pitch-from-upright
};

// MAVLink custom main mode value for MANUAL (PX4_CUSTOM_MAIN_MODE_MANUAL); used for the handoff
// DO_SET_MODE. Hardcoded to avoid a build dependency on the commander header.
static constexpr float CUSTOM_MAIN_MODE_MANUAL = 1.f;

SelfRight::SelfRight() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

SelfRight::~SelfRight()
{
	perf_free(_loop_perf);
}

bool SelfRight::init()
{
	// 100 Hz: fast enough for the righting control loop and a steady actuator_motors stream.
	ScheduleOnInterval(10_ms);
	return true;
}

void SelfRight::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}
}

bool SelfRight::modeActive(const vehicle_status_s &status) const
{
	return status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
	       && status.nav_state == vehicle_status_s::NAVIGATION_STATE_SELF_RIGHT;
}

float SelfRight::pitchFromUpright(const vehicle_attitude_s &att) const
{
	// Body +Z expressed in NED. Its down component is +1 upright, -1 inverted.
	const matrix::Vector3f body_z_in_ned = matrix::Quatf(att.q).dcm_z();
	return acosf(math::constrain(body_z_in_ned(2), -1.f, 1.f));
}

void SelfRight::resetManeuver()
{
	_state = State::Idle;
	_abort_reason = self_right_status_s::ABORT_NONE;
	_over_center = false;
	_tilt_integral = 0.f;
	_tilt_last_error_valid = false;
	_pid_integral = 0.f;
	_theta_prev = NAN;
	_theta_min = NAN;
}

void SelfRight::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	parameters_update();

	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);

	const hrt_abstime now = hrt_absolute_time();
	float dt = (_last_run > 0) ? (now - _last_run) * 1e-6f : 0.01f;
	dt = math::constrain(dt, 0.001f, 0.1f);
	_last_run = now;

	if (!_param_sr_en.get() || !modeActive(status)) {
		// Not our mode: stay inert and let the normal allocation path own the outputs.
		if (_state != State::Idle) {
			resetManeuver();
		}

		publishStatus();
		perf_end(_loop_perf);
		return;
	}

	// --- Active: gather inputs --------------------------------------------------------------
	vehicle_attitude_s att{};
	vehicle_angular_velocity_s ang_vel{};
	sensor_encoder_s enc{};
	const bool have_att = _vehicle_attitude_sub.copy(&att);
	_vehicle_angular_velocity_sub.copy(&ang_vel);
	const bool have_enc = _sensor_encoder_sub.copy(&enc);

	_theta = have_att ? pitchFromUpright(att) : _theta;
	_pitch_rate = ang_vel.xyz[1];
	_tilt_angle = have_enc ? enc.angle : _tilt_angle;

	// Over-center: latched once pitch-from-upright drops below the tipping angle.
	if (PX4_ISFINITE(_theta)) {
		_theta_min = PX4_ISFINITE(_theta_min) ? fminf(_theta_min, _theta) : _theta;

		if (_theta < _param_sr_overctr.get()) {
			_over_center = true;
		}
	}

	const float measured_tilt = have_enc ? enc.angle : 0.f;
	const float t_in_state = (now - _state_start) * 1e-6f;

	switch (_state) {
	case State::Idle:
		// Just entered the active mode: arm the maneuver; RotateWing drives it next cycle.
		_attempt_thrust = _param_sr_adapt_en.get()
				  ? math::constrain(_param_sr_thr_lrn.get(), 0.f, _param_sr_thr_max.get())
				  : _param_sr_thr_max.get();
		_state = State::RotateWing;
		_state_start = now;
		_abort_reason = self_right_status_s::ABORT_NONE;
		_over_center = false;
		_theta_min = _theta;
		publishMotors(0.f);
		break;

	case State::RotateWing:
		commandTilt(_param_sr_tilt_sp.get(), measured_tilt, dt);
		publishMotors(0.f);

		if (checkAbort(t_in_state)) {
			_state = State::Cut;
			_state_start = now;

		} else if (fabsf(measured_tilt - _param_sr_tilt_sp.get()) < _param_sr_tilt_tol.get()
			   || t_in_state > _param_sr_tilt_tmo.get()) {
			_state = State::Righting;
			_state_start = now;
			_maneuver_start = now;
			_pid_integral = 0.f;
			_theta_prev = _theta;
		}

		break;

	case State::Righting: {
			// Keep the wing held props-up while thrusting.
			commandTilt(_param_sr_tilt_sp.get(), measured_tilt, dt);

			const float throttle = computeRightingThrottle(t_in_state, _theta, dt);
			publishMotors(throttle);
			_theta_prev = _theta;

			const float t_in_maneuver = (now - _maneuver_start) * 1e-6f;

			if (checkAbort(t_in_maneuver)) {
				_state = State::Cut;
				_state_start = now;

			} else if (_over_center) {
				// Success: the flip is committed. Cut and let buoyancy settle it upright.
				if (_param_sr_adapt_en.get()) {
					// Latch the successful (minimum-so-far) thrust.
					float lrn = _attempt_thrust;
					param_set(param_find("SR_THR_LRN"), &lrn);
				}

				_abort_reason = self_right_status_s::ABORT_NONE;
				_state = State::Cut;
				_state_start = now;

			} else if (t_in_maneuver > _param_sr_timeout.get()) {
				// Failed to reach over-center within budget: cut and (if adaptive) step thrust up.
				_abort_reason = self_right_status_s::ABORT_TIMEOUT;

				if (_param_sr_adapt_en.get()) {
					float lrn = math::min(_param_sr_thr_lrn.get() + _param_sr_thr_step.get(),
							      _param_sr_thr_max.get());
					param_set(param_find("SR_THR_LRN"), &lrn);
					_attempt++;
				}

				_state = State::Cut;
				_state_start = now;
			}

			break;
		}

	case State::Cut:
		// Throttle off, retract the wing toward park (clears the props from the swept arc).
		publishMotors(0.f);
		commandTilt(_param_sr_tilt_park.get(), measured_tilt, dt);

		if (t_in_state > 0.5f) {
			_state = State::Done;
			_state_start = now;
		}

		break;

	case State::Done:
		publishMotors(0.f);
		requestManual(); // hand back to MANUAL; commander switching nav_state ends the maneuver
		break;
	}

	publishStatus();
	perf_end(_loop_perf);
}

float SelfRight::computeRightingThrottle(float t_in_state, float theta, float dt)
{
	const float ramp = (_param_sr_ramp_t.get() > 0.01f)
			   ? math::min(t_in_state / _param_sr_ramp_t.get(), 1.f) : 1.f;

	if ((Strategy)_param_sr_strategy.get() == Strategy::AttitudePID) {
		float derivative = 0.f;

		if (PX4_ISFINITE(_theta_prev)) {
			derivative = (theta - _theta_prev) / dt;
		}

		// Anti-windup: bound the integral so it alone cannot saturate the command.
		_pid_integral = math::constrain(_pid_integral + theta * dt, 0.f, 5.f);

		const float u = _param_sr_pid_p.get() * theta
				+ _param_sr_pid_i.get() * _pid_integral
				+ _param_sr_pid_d.get() * derivative;

		return ramp * math::constrain(u, 0.f, _param_sr_thr_max.get());
	}

	// OpenLoop: ramp to the (possibly adaptively-learned) peak thrust and hold.
	return ramp * _attempt_thrust;
}

bool SelfRight::checkAbort(float t_in_maneuver)
{
	// Pilot stick override (always active).
	manual_control_setpoint_s manual;

	if (_manual_control_setpoint_sub.copy(&manual) && manual.valid) {
		const float dz = _param_sr_stick_dz.get();

		if (fabsf(manual.roll) > dz || fabsf(manual.pitch) > dz || fabsf(manual.yaw) > dz) {
			_abort_reason = self_right_status_s::ABORT_STICK;
			return true;
		}
	}

	// Downward rangefinder backstop (only when SR_RNG_MIN > 0).
	if (_param_sr_rng_min.get() > 0.f) {
		distance_sensor_s dist;

		if (_distance_sensor_sub.copy(&dist)
		    && dist.orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING
		    && dist.signal_quality != 0
		    && dist.current_distance < _param_sr_rng_min.get()) {
			_abort_reason = self_right_status_s::ABORT_RANGEFINDER;
			return true;
		}
	}

	// Estimator clipping: degrades the attitude the PID strategy depends on. OpenLoop replay
	// uses no in-loop attitude, so it is allowed to continue.
	if ((Strategy)_param_sr_strategy.get() == Strategy::AttitudePID) {
		vehicle_imu_s imu;
		estimator_status_flags_s est_flags;
		bool bad_accel = false;

		if (_vehicle_imu_sub.copy(&imu) && imu.delta_velocity_clipping != 0) {
			bad_accel = true;
		}

		if (_estimator_status_flags_sub.copy(&est_flags)
		    && (est_flags.fs_bad_acc_clipping || est_flags.fs_bad_acc_vertical)) {
			bad_accel = true;
		}

		if (bad_accel) {
			_abort_reason = self_right_status_s::ABORT_ESTIMATOR;
			return true;
		}
	}

	(void)t_in_maneuver; // timeout is handled in the Righting state to distinguish adaptive fail
	return false;
}

void SelfRight::commandTilt(float setpoint_rad, float measured_rad, float dt)
{
	// Position loop on the encoder feedback -> normalized rate command for the tilt ESC,
	// same structure as SunTracker (the tilt actuator is a rate-source plant).
	const float error = matrix::wrap_pi(setpoint_rad - measured_rad);

	_tilt_integral = math::constrain(_tilt_integral + _param_sr_ki.get() * error * dt, -1.f, 1.f);

	float derivative = 0.f;

	if (_tilt_last_error_valid) {
		derivative = (error - _tilt_last_error) / dt;
	}

	_tilt_last_error = error;
	_tilt_last_error_valid = true;

	const float u = math::constrain(_param_sr_kp.get() * error + _tilt_integral
					+ _param_sr_kd.get() * derivative, -1.f, 1.f);

	// Rate-limit the DO_SET_ACTUATOR traffic (commander ACKs each one), like SunTracker.
	const hrt_abstime now = hrt_absolute_time();
	const bool changed = !PX4_ISFINITE(_last_tilt_cmd) || (fabsf(u - _last_tilt_cmd) > 0.005f);
	const bool heartbeat = (now - _last_tilt_publish) > 200_ms;

	if (!changed && !heartbeat) {
		return;
	}

	vehicle_command_s cmd{};
	cmd.timestamp = now;
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;
	cmd.param1 = u; // -> Peripheral_via_Actuator_Set1 (PWM_AUX_FUNCx = 301)
	cmd.param2 = NAN;
	cmd.param3 = NAN;
	cmd.param4 = NAN;
	cmd.param5 = NAN;
	cmd.param6 = NAN;
	cmd.param7 = 0.f; // actuator set index
	cmd.from_external = false;
	_vehicle_command_pub.publish(cmd);

	_last_tilt_cmd = u;
	_last_tilt_publish = now;
}

void SelfRight::publishMotors(float throttle)
{
	throttle = math::constrain(throttle, 0.f, 1.f);

	actuator_motors_s motors{};
	motors.timestamp = hrt_absolute_time();
	motors.timestamp_sample = motors.timestamp;

	// Symmetric thrust on the two tractor motors -> pure pitching moment through the tilted
	// thrust line. Remaining channels NaN (disarmed/unused).
	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; i++) {
		motors.control[i] = NAN;
	}

	motors.control[0] = throttle;
	motors.control[1] = throttle;

	_actuator_motors_pub.publish(motors);
	_cmd_throttle = throttle;
}

void SelfRight::requestManual()
{
	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);

	vehicle_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	cmd.param1 = 1.f; // base mode: custom mode enabled
	cmd.param2 = CUSTOM_MAIN_MODE_MANUAL;
	cmd.param3 = NAN;
	cmd.target_system = status.system_id;
	cmd.target_component = status.component_id;
	cmd.source_system = status.system_id;
	cmd.source_component = status.component_id;
	cmd.from_external = false;
	_vehicle_command_pub.publish(cmd);
}

void SelfRight::publishStatus()
{
	self_right_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.state = (uint8_t)_state;
	status.strategy = (uint8_t)_param_sr_strategy.get();
	status.abort_reason = _abort_reason;
	status.pitch_from_upright = _theta;
	status.pitch_rate = _pitch_rate;
	status.tilt_angle = _tilt_angle;
	status.throttle = _cmd_throttle;
	status.over_center = _over_center;
	status.active = _state != State::Idle;
	status.learned_thrust = _param_sr_thr_lrn.get();
	status.attempt = _attempt;
	_self_right_status_pub.publish(status);
}

int SelfRight::print_status()
{
	const float r2d = 180.f / (float)M_PI;
	PX4_INFO("state: %d  strategy: %d  over_center: %d  abort: %d",
		 (int)_state, (int)_param_sr_strategy.get(), _over_center, _abort_reason);
	PX4_INFO("theta: %.1f deg  pitch_rate: %.2f rad/s  tilt: %.1f deg",
		 (double)(_theta * r2d), (double)_pitch_rate, (double)(_tilt_angle * r2d));
	PX4_INFO("adaptive: en %d  learned_thrust %.2f  attempt %d",
		 (int)_param_sr_adapt_en.get(), (double)_param_sr_thr_lrn.get(), _attempt);
	perf_print_counter(_loop_perf);
	return 0;
}

int SelfRight::task_spawn(int argc, char *argv[])
{
	SelfRight *instance = new SelfRight();

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

int SelfRight::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SelfRight::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Autonomous self-righting controller for the tilt-wing aircraft. Active in the SELF_RIGHT flight
mode (enterable only when commander reports the aircraft inverted and at rest): rotates the wing so
the props point up, uses symmetric propeller thrust to drive the airframe past its over-center
tipping point, then cuts throttle and hands back to MANUAL. Selectable righting strategies
(SR_STRATEGY): open-loop thrust replay, or PID on pitch-from-upright, with an optional adaptive
minimum-thrust learning layer. See .CLAUDE/self_right_architecture.md.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("self_right", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int self_right_main(int argc, char *argv[])
{
	return SelfRight::main(argc, argv);
}
