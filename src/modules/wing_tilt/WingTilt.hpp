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
 * @file WingTilt.hpp
 *
 * Wing tilt controller — the single owner of the tilt motor.
 *
 * Closes the position loop between the AS5600 wing encoder (sensor_encoder, de-geared by
 * 1/TILT_GEAR) and the reversible tilt ESC (DO_SET_ACTUATOR -> Peripheral_via_Actuator_Set1).
 * Client modules (sun_tracker, self_right, console holds) never command the ESC directly —
 * they publish wing-angle setpoints on wing_tilt_setpoint and this controller arbitrates:
 * highest-priority fresh source wins, and a source releases the wing by simply not
 * republishing for 0.5 s. With no owner (or no trustworthy encoder) the wing is stopped
 * with a single neutral command — the actuator is a rate plant, so zero means "hold still".
 */

#pragma once

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
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_encoder.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/wing_tilt_setpoint.h>
#include <uORB/topics/wing_tilt_status.h>

using namespace time_literals;

class WingTilt : public ModuleBase<WingTilt>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	WingTilt();
	~WingTilt() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;
	void parameters_update();

	// Command the ESC to a normalized value in [-1, 1] (rate-limited to reduce command traffic).
	void publishActuator(float value);

	void publishStatus();

	static constexpr int NUM_SOURCES = 3;                     // WingTiltSetpoint SOURCE_* count
	static constexpr hrt_abstime SETPOINT_TIMEOUT = 500_ms;   // source released when older
	static constexpr hrt_abstime ENCODER_TIMEOUT = 200_ms;    // feedback considered stale when older

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _wing_tilt_setpoint_sub{ORB_ID(wing_tilt_setpoint)};
	uORB::Subscription _sensor_encoder_sub{ORB_ID(sensor_encoder)};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};

	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<wing_tilt_status_s> _wing_tilt_status_pub{ORB_ID(wing_tilt_status)};

	// Latest setpoint per source; timestamp 0 = never received. The source index doubles as
	// its priority (higher wins), matching the SOURCE_* values in WingTiltSetpoint.msg.
	wing_tilt_setpoint_s _setpoints[NUM_SOURCES] {};

	// Encoder->wing conversion factor (1 / TILT_GEAR), resolved once in init(). The gear ratio
	// is fixed mechanical configuration, so it is never re-read in the control loop.
	float _encoder_to_wing{1.f};

	// PID state
	float _integral{0.f};
	float _last_error{0.f};
	bool _last_error_valid{false};
	hrt_abstime _last_run{0};
	bool _was_driving{false};   // to send exactly one stop command on ownership/feedback loss

	// Output rate limiting / change detection
	float _last_output{NAN};
	hrt_abstime _last_publish{0};

	// ESC arming sequence (below neutral, then above, then neutral): the reversible tilt ESC
	// only arms after seeing this, and the AUX pin is pinned to the disarmed neutral until the
	// vehicle outputs go live — so the sequence runs on every disarmed->live transition.
	bool _outputs_live_prev{false};
	hrt_abstime _esc_arm_start{0};   // 0 = sequence not running

	// Diagnostics; angles in radians.
	uint8_t _active_source{wing_tilt_status_s::SOURCE_NONE};
	float _setpoint{NAN};
	float _measured_angle{NAN};
	float _error{NAN};
	float _output{0.f};
	bool _encoder_valid{false};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::TILT_KP>) _param_tilt_kp,
		(ParamFloat<px4::params::TILT_KI>) _param_tilt_ki,
		(ParamFloat<px4::params::TILT_KD>) _param_tilt_kd,
		(ParamFloat<px4::params::TILT_DB>) _param_tilt_db,
		(ParamFloat<px4::params::TILT_GEAR>) _param_tilt_gear,
		(ParamFloat<px4::params::TILT_ARM_V>) _param_tilt_arm_v,
		(ParamFloat<px4::params::TILT_ARM_T>) _param_tilt_arm_t
	)
};
