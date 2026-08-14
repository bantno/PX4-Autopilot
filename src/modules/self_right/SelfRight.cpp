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

#include <stdlib.h>
#include <string.h>

// Max time in Verify for the preconditions to pass before giving up and disarming.
static constexpr float VERIFY_TIMEOUT_S = 1.f;

// Magic param2 for COMPONENT_ARM_DISARM: force, i.e. skip the "not landed" disarm refusal
// (see Commander::disarm()). The land detector cannot be trusted floating on water.
static constexpr float FORCE_ARM_DISARM_MAGIC = 21196.f;

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
	// TILT_GEAR is fixed mechanical configuration (reboot_required); resolve the encoder->wing
	// conversion once, matching the wing_tilt controller.
	const float gear_ratio = _param_tilt_gear.get();
	_encoder_to_wing = (gear_ratio > 0.01f) ? (1.f / gear_ratio) : 1.f;

	// 100 Hz: fast enough for the righting control loop and a steady actuator_motors stream.
	// Wing tilt setpoints are rate-limited to the wing_tilt controller's 20 Hz.
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
	_last_tilt_sp_publish = 0;
	_last_disarm_request = 0;
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
		// Not our mode: stay inert and let the normal allocation path own the outputs. The
		// wing_tilt controller stops the wing once our setpoints go stale.
		if (_state != State::Idle) {
			resetManeuver();
		}

		// Keep the diagnostics (theta, tilt, pitch rate) live while idle so bench checks can
		// watch them on self_right_status before entering the mode.
		vehicle_attitude_s att;

		if (_vehicle_attitude_sub.copy(&att)) {
			_theta = pitchFromUpright(att);
		}

		vehicle_angular_velocity_s ang_vel;

		if (_vehicle_angular_velocity_sub.copy(&ang_vel)) {
			_pitch_rate = ang_vel.xyz[1];
		}

		sensor_encoder_s enc;
		const bool have_enc = _sensor_encoder_sub.copy(&enc);

		if (have_enc) {
			_tilt_angle = enc.angle * _encoder_to_wing;
		}

		// Manual tilt hold (console `tilt` command): keep requesting the hold angle from the
		// wing_tilt controller in any mode and arming state — it must keep holding while armed
		// in MANUAL so the wing resists prop thrust during a hand-flown righting. Motors are
		// never touched; `tilt off` releases by letting the setpoint go stale.
		if (_manual_tilt_active.load()) {
			publishTiltSetpoint(wing_tilt_setpoint_s::SOURCE_CONSOLE,
					    _manual_tilt_sp_mrad.load() * 1e-3f);
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
	_tilt_angle = have_enc ? (enc.angle * _encoder_to_wing) : _tilt_angle;

	// Over-center: latched once pitch-from-upright drops below the tipping angle while thrusting.
	if (_state == State::Righting && PX4_ISFINITE(_theta) && _theta < _param_sr_overctr.get()) {
		_over_center = true;
	}

	const float measured_tilt = have_enc ? (enc.angle * _encoder_to_wing) : 0.f;
	const float t_in_state = (now - _state_start) * 1e-6f;

	switch (_state) {
	case State::Idle:
		// Just entered the active mode. The state machine owns the tilt from here on — drop any
		// console-commanded hold (our maneuver setpoints outrank console ones anyway).
		_manual_tilt_active.store(false);
		_state = State::Verify;
		_state_start = now;
		_abort_reason = self_right_status_s::ABORT_NONE;
		_over_center = false;
		publishMotors(0.f);
		break;

	case State::Verify:
		// Nothing moves until the preconditions hold.
		publishMotors(0.f);

		if (stickOverride()) {
			_abort_reason = self_right_status_s::ABORT_STICK;
			_state = State::Disarm; // nothing has moved: no Cut needed
			_state_start = now;

		} else if (verifyPreconditions()) {
			_state = State::RotateWing;
			_state_start = now;

		} else if (t_in_state > VERIFY_TIMEOUT_S) {
			_abort_reason = self_right_status_s::ABORT_VERIFY_FAIL;
			_state = State::Disarm;
			_state_start = now;
		}

		break;

	case State::RotateWing:
		publishTiltSetpoint(wing_tilt_setpoint_s::SOURCE_SELF_RIGHT, _param_sr_tilt_sp.get());
		publishMotors(0.f);

		if (stickOverride()) {
			_abort_reason = self_right_status_s::ABORT_STICK;
			_state = State::Cut;
			_state_start = now;

		} else if (fabsf(measured_tilt - _param_sr_tilt_sp.get()) < _param_sr_tilt_tol.get()) {
			_state = State::Righting;
			_state_start = now;

		} else if (t_in_state > _param_sr_tilt_tmo.get()) {
			// The wing never reached props-up: abort — never thrust with the props in an
			// unknown position.
			_abort_reason = self_right_status_s::ABORT_TILT_TIMEOUT;
			_state = State::Cut;
			_state_start = now;
		}

		break;

	case State::Righting: {
			// Keep the wing held props-up while ramping symmetric thrust.
			publishTiltSetpoint(wing_tilt_setpoint_s::SOURCE_SELF_RIGHT, _param_sr_tilt_sp.get());

			const float ramp = (_param_sr_ramp_t.get() > 0.01f)
					   ? math::min(t_in_state / _param_sr_ramp_t.get(), 1.f) : 1.f;
			publishMotors(ramp * _param_sr_thr_max.get());

			if (stickOverride()) {
				_abort_reason = self_right_status_s::ABORT_STICK;
				_state = State::Cut;
				_state_start = now;

			} else if (_over_center) {
				// Success: the flip is committed. Cut and let buoyancy settle it upright.
				_state = State::Cut;
				_state_start = now;

			} else if (t_in_state > _param_sr_timeout.get()) {
				_abort_reason = self_right_status_s::ABORT_RIGHTING_TIMEOUT;
				_state = State::Cut;
				_state_start = now;
			}

			break;
		}

	case State::Cut:
		// Throttle off, retract the wing toward park (clears the props from the swept arc).
		publishMotors(0.f);
		publishTiltSetpoint(wing_tilt_setpoint_s::SOURCE_SELF_RIGHT, _param_sr_tilt_park.get());

		if (stickOverride()
		    || (t_in_state > 0.5f
			&& fabsf(measured_tilt - _param_sr_tilt_park.get()) < _param_sr_tilt_tol.get())
		    || t_in_state > _param_sr_tilt_tmo.get()) {
			_state = State::Disarm;
			_state_start = now;
		}

		break;

	case State::Disarm:
		publishMotors(0.f);
		// Keep the wing held at park while the disarm goes through.
		publishTiltSetpoint(wing_tilt_setpoint_s::SOURCE_SELF_RIGHT, _param_sr_tilt_park.get());

		if (_last_disarm_request == 0 || (now - _last_disarm_request) > 500_ms) {
			sendDisarm();
			_last_disarm_request = now;
		}

		// Once commander processes the disarm, modeActive() goes false and the top of Run()
		// resets the state machine to Idle.
		break;
	}

	publishStatus();
	perf_end(_loop_perf);
}

bool SelfRight::verifyPreconditions()
{
	const hrt_abstime now = hrt_absolute_time();

	// Inverted: same test as commander's entry gate (selfRightingCheck) — the body +Z axis points
	// down in the world by at least SR_INV_THR. cos(theta) is exactly dcm_z(q)(2).
	const bool inverted = PX4_ISFINITE(_theta) && cosf(_theta) < -_param_sr_inv_thr.get();

	// At rest, with a converged EKF tilt estimate.
	// vehicle_land_detected_s land;
	// const bool at_rest = _vehicle_land_detected_sub.copy(&land) && land.at_rest;

	estimator_status_flags_s est_flags;
	const bool tilt_aligned = _estimator_status_flags_sub.copy(&est_flags) && est_flags.cs_tilt_align;

	// Wing encoder fresh and trustworthy — never drive the tilt open-loop.
	sensor_encoder_s enc;
	const bool encoder_ok = _sensor_encoder_sub.copy(&enc)
				&& (now - enc.timestamp) < 1_s
				&& enc.valid && enc.zeroed;

	// Battery healthy enough to spend a high-throttle attempt.
	battery_status_s bat;
	const bool battery_ok = _battery_status_sub.copy(&bat)
				&& (now - bat.timestamp) < 5_s
				&& bat.connected
				&& bat.warning < battery_status_s::WARNING_LOW;

	// return inverted && at_rest && tilt_aligned && encoder_ok && battery_ok;
	return inverted && tilt_aligned && encoder_ok && battery_ok;
}

bool SelfRight::stickOverride()
{
	manual_control_setpoint_s manual;

	if (_manual_control_setpoint_sub.copy(&manual) && manual.valid) {
		const float dz = _param_sr_stick_dz.get();

		if (fabsf(manual.roll) > dz || fabsf(manual.pitch) > dz || fabsf(manual.yaw) > dz) {
			return true;
		}
	}

	return false;
}

void SelfRight::publishTiltSetpoint(uint8_t source, float angle)
{
	// The wing_tilt controller runs at 20 Hz — publishing faster only fills its queue.
	const hrt_abstime now = hrt_absolute_time();

	if (_last_tilt_sp_publish != 0 && (now - _last_tilt_sp_publish) < 50_ms) {
		return;
	}

	wing_tilt_setpoint_s sp{};
	sp.timestamp = now;
	sp.source = source;
	sp.angle = angle;
	_wing_tilt_setpoint_pub.publish(sp);
	_last_tilt_sp_publish = now;
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

void SelfRight::sendDisarm()
{
	vehicle_status_s status{};
	_vehicle_status_sub.copy(&status);

	vehicle_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
	cmd.param1 = (float)vehicle_command_s::ARMING_ACTION_DISARM;
	cmd.param2 = FORCE_ARM_DISARM_MAGIC;
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
	status.abort_reason = _abort_reason;
	status.pitch_from_upright = _theta;
	status.pitch_rate = _pitch_rate;
	status.tilt_angle = _tilt_angle;
	status.throttle = _cmd_throttle;
	status.over_center = _over_center;
	status.active = _state != State::Idle;
	_self_right_status_pub.publish(status);
}

int SelfRight::print_status()
{
	const float r2d = 180.f / (float)M_PI;
	PX4_INFO("state: %d  over_center: %d  abort: %d",
		 (int)_state, _over_center, _abort_reason);
	PX4_INFO("theta: %.1f deg  pitch_rate: %.2f rad/s  tilt: %.1f deg",
		 (double)(_theta * r2d), (double)_pitch_rate, (double)(_tilt_angle * r2d));

	if (_manual_tilt_active.load()) {
		PX4_INFO("manual tilt hold: %.1f deg", (double)(_manual_tilt_sp_mrad.load() * 1e-3f * r2d));
	}

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
	if (argc >= 1 && strcmp(argv[0], "tilt") == 0) {
		if (argc < 2) {
			return print_usage("tilt: missing angle");
		}

		SelfRight *inst = get_instance();

		if (!is_running() || inst == nullptr) {
			PX4_ERR("not running");
			return PX4_ERROR;
		}

		if (strcmp(argv[1], "off") == 0) {
			inst->_manual_tilt_active.store(false);
			PX4_INFO("tilt: released");
			return PX4_OK;
		}

		char *end = nullptr;
		const float deg = strtof(argv[1], &end);

		if (end == argv[1] || *end != '\0') {
			return print_usage("tilt: invalid angle");
		}

		inst->_manual_tilt_sp_mrad.store((int32_t)roundf(math::radians(deg) * 1000.f));
		inst->_manual_tilt_active.store(true);
		PX4_INFO("tilt: driving wing to %.1f deg and holding (use `tilt off` to release)", (double)deg);
		return PX4_OK;
	}

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
mode (enterable only when commander reports the aircraft inverted and at rest): verifies the
preconditions (inverted, at rest, EKF tilt-aligned, fresh wing encoder, battery OK), rotates the
wing so the props point up, ramps symmetric propeller thrust to drive the airframe past its
over-center tipping point, then cuts throttle, parks the wing and force-disarms. Every exit path
(success, timeout, verify failure, pilot stick override) ends in a disarm.
See documentation/self_right_architecture.md.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("self_right", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("tilt", "Manually drive and hold the wing at an angle (encoder position loop; "
					 "works in any mode/arming state, released automatically when the maneuver starts)");
	PRINT_MODULE_USAGE_ARG("<deg>|off", "Wing angle in degrees, or 'off' to release the hold", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int self_right_main(int argc, char *argv[])
{
	return SelfRight::main(argc, argv);
}
