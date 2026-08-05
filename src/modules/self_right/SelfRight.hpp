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

/**
 * @file SelfRight.hpp
 *
 * Autonomous self-righting controller for the tilt-wing aircraft.
 *
 * Recovers the aircraft from an inverted, at-rest float to upright using propeller thrust only:
 * verifies the preconditions, rotates the wing so the props point up, ramps symmetric thrust to
 * drive the airframe past its over-center tipping point, cuts throttle, parks the wing and force-
 * disarms. Every exit path (success, timeout, verify failure, pilot stick override) ends in a
 * disarm. The mode is entered only when commander's latched SELF_RIGHT gate (inverted + at rest)
 * allows it. See documentation/self_right_architecture.md.
 */

#pragma once

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/self_right_status.h>
#include <uORB/topics/sensor_encoder.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class SelfRight : public ModuleBase<SelfRight>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	SelfRight();
	~SelfRight() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	// State machine; values mirror SelfRightStatus.msg STATE_*.
	enum class State : uint8_t {
		Idle = 0,        // mode not active
		Verify = 1,      // checking preconditions, nothing moving
		RotateWing = 2,  // driving the wing to the props-up setpoint
		Righting = 3,    // ramped symmetric thrust toward over-center
		Cut = 4,         // throttle cut, parking the wing
		Disarm = 5,      // requesting forced disarm
	};

	void Run() override;
	void parameters_update();

	// Returns true while the SELF_RIGHT mode owns the outputs (armed + nav_state matches).
	bool modeActive(const vehicle_status_s &status) const;

	// Angle of the body +Z axis from world-up [rad]: 0 = upright, pi = fully inverted.
	float pitchFromUpright(const vehicle_attitude_s &att) const;

	// True when all safety preconditions hold this cycle (inverted, at rest, EKF tilt-aligned,
	// fresh+valid encoder, battery OK).
	bool verifyPreconditions();

	// True if the pilot moved any stick beyond SR_STICK_DZ (manual override).
	bool stickOverride();

	// Drive the wing tilt position loop toward `setpoint_rad` (publishes DO_SET_ACTUATOR).
	// Self-paced at the sun tracker's 20 Hz cadence; angles are in the wing frame.
	void commandTilt(float setpoint_rad, float measured_rad);

	// Publish a zero (neutral) tilt command so the rate-source wing actuator stops moving.
	void publishTiltNeutral();

	// Publish symmetric motor throttle on actuator_motors (NaN past the two tractors = disarmed).
	void publishMotors(float throttle);

	// Request a forced disarm (cannot be refused for "not landed").
	void sendDisarm();

	void publishStatus();
	void resetManeuver();

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _sensor_encoder_sub{ORB_ID(sensor_encoder)};
	uORB::Subscription _estimator_status_flags_sub{ORB_ID(estimator_status_flags)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};

	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<self_right_status_s> _self_right_status_pub{ORB_ID(self_right_status)};

	State _state{State::Idle};
	uint8_t _abort_reason{0};       // SelfRightStatus.msg ABORT_*
	bool _over_center{false};

	hrt_abstime _last_run{0};
	hrt_abstime _state_start{0};         // time the current State was entered
	hrt_abstime _last_disarm_request{0}; // Disarm state: last COMPONENT_ARM_DISARM sent (500 ms retry)

	// Tilt position loop state (mirrors SunTracker's PID; shares its SUN_* tune).
	float _encoder_to_wing{1.f};   // 1 / SUN_GEAR_RATIO, resolved once in init()
	float _tilt_integral{0.f};
	float _tilt_last_error{0.f};
	bool _tilt_last_error_valid{false};
	float _last_tilt_cmd{NAN};
	hrt_abstime _last_tilt_publish{0};
	hrt_abstime _last_tilt_loop{0};

	// Manual tilt hold (console `tilt` command; written from the console thread, read in Run()).
	// Lets the pilot position and actively hold the wing (e.g. props-up for a hand-flown
	// righting) using the same encoder position loop the maneuver uses. Never touches motors.
	px4::atomic_bool _manual_tilt_active{false};
	px4::atomic<int32_t> _manual_tilt_sp_mrad{0}; // setpoint in millirad (atomic<float> unsupported)
	bool _manual_tilt_was_active{false};          // for the neutral command on hold release

	// Diagnostics.
	float _theta{NAN};
	float _pitch_rate{0.f};
	float _tilt_angle{NAN};
	float _cmd_throttle{0.f};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::SR_EN>) _param_sr_en,
		(ParamFloat<px4::params::SR_INV_THR>) _param_sr_inv_thr,
		(ParamFloat<px4::params::SR_TILT_SP>) _param_sr_tilt_sp,
		(ParamFloat<px4::params::SR_TILT_PARK>) _param_sr_tilt_park,
		(ParamFloat<px4::params::SR_TILT_TOL>) _param_sr_tilt_tol,
		(ParamFloat<px4::params::SR_TILT_TMO>) _param_sr_tilt_tmo,
		// Tilt loop tune shared with the sun tracker — same physical actuator/encoder,
		// so one set of gains serves both (self_right Kconfig depends on sun_tracker).
		(ParamFloat<px4::params::SUN_KP>) _param_sun_kp,
		(ParamFloat<px4::params::SUN_KI>) _param_sun_ki,
		(ParamFloat<px4::params::SUN_KD>) _param_sun_kd,
		(ParamFloat<px4::params::SUN_DEADBAND>) _param_sun_deadband,
		(ParamFloat<px4::params::SUN_GEAR_RATIO>) _param_sun_gear,
		(ParamFloat<px4::params::SR_THR_MAX>) _param_sr_thr_max,
		(ParamFloat<px4::params::SR_RAMP_T>) _param_sr_ramp_t,
		(ParamFloat<px4::params::SR_OVERCTR>) _param_sr_overctr,
		(ParamFloat<px4::params::SR_TIMEOUT>) _param_sr_timeout,
		(ParamFloat<px4::params::SR_STICK_DZ>) _param_sr_stick_dz
	)
};
