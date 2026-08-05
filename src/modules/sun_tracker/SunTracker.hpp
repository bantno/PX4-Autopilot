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
 * @file SunTracker.hpp
 *
 * Sun-tracking tilt-wing controller.
 *
 * Computes the sun direction from GPS position + UTC time, expresses it in the body frame
 * using the vehicle attitude (including a north-referenced heading, gated on the estimator
 * heading variance so it works stationary on the ground), derives the wing tilt angle that points
 * the panel normal at the sun, and closes a position loop on the AS5600 encoder feedback
 * (sensor_encoder). The control
 * effort is sent to a dedicated reversible ESC via vehicle_command / DO_SET_ACTUATOR
 * (Peripheral_via_Actuator_Set1, mapped with PWM_AUX_FUNCx = 301).
 */

#pragma once

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_encoder.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sun_tracker_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/wing_tilt_setpoint.h>

using namespace time_literals;

class SunTracker : public ModuleBase<SunTracker>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	SunTracker();
	~SunTracker() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	void parameters_update();

	// Publish the per-cycle diagnostics (sun_tracker_status) for logging/telemetry.
	void publishStatus(bool enabled, bool debug_sweep, bool pose_valid, bool heading_valid);

	// Compute the desired wing tilt angle [rad] from the body-frame sun unit vector.
	float desiredTiltFromSun(float sun_body_x, float sun_body_y, float sun_body_z) const;

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};

	uORB::Publication<wing_tilt_setpoint_s> _wing_tilt_setpoint_pub{ORB_ID(wing_tilt_setpoint)};
	uORB::Publication<sun_tracker_status_s> _sun_tracker_status_pub{ORB_ID(sun_tracker_status)};

	// Resolve a usable UTC (microseconds since the Unix epoch): prefer GPS, fall back to the
	// system realtime clock when GPS provides none (e.g. SITL). Returns 0 if neither is valid.
	uint64_t resolveUtcUsec(const sensor_gps_s &gps, bool have_gps) const;

	// Park-to-zero request (set by the `park` console command). When active, the loop ignores the
	// sun/pose gate and holds the wing at the boot-time zero so it can be stowed level before
	// power-off. Atomic: written from the shell thread, read from the work-queue thread.
	px4::atomic_bool _park_request{false};

	// Diagnostics (for print_status); angles in radians.
	const char *_status{"init"};
	float _sun_az{NAN};
	float _sun_el{NAN};
	float _heading{NAN};
	float _heading_var{NAN};
	float _tilt_setpoint{NAN};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::SUN_TRK_EN>) _param_sun_trk_en,
		(ParamFloat<px4::params::SUN_TILT_OFF>) _param_sun_tilt_off,
		(ParamFloat<px4::params::SUN_TILT_MIN>) _param_sun_tilt_min,
		(ParamFloat<px4::params::SUN_TILT_MAX>) _param_sun_tilt_max,
		(ParamFloat<px4::params::SUN_PARK>) _param_sun_park,
		(ParamInt<px4::params::SUN_AXIS>) _param_sun_axis,
		(ParamBool<px4::params::SUN_TILT_REV>) _param_sun_tilt_rev,
		(ParamFloat<px4::params::SUN_HDG_VAR_MAX>) _param_sun_hdg_var_max,

		// >>> TEMP DEBUG SWEEP — remove with the matching code in SunTracker.cpp. <<<
		(ParamBool<px4::params::SUN_DBG_SWP>) _param_sun_dbg_swp,
		(ParamFloat<px4::params::SUN_DBG_AMP>) _param_sun_dbg_amp
	)
};
