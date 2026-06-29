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

#include "WingTiltSim.hpp"

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>

// Plant model constants. The tilt motor is treated as a rate source: full command (|u|=1)
// slews the wing at MAX_RATE_RAD_S; u=0 holds (a non-backdrivable / closed-current actuator).
// TRAVEL_LIMIT_RAD spans past +/-pi/2 so the self_right mode can rotate the wing to the
// props-up (~90 deg) self-righting setpoint, not just the sun-tracking range (~+/-69 deg).
static constexpr float MAX_RATE_RAD_S = 1.5f;
static constexpr float TRAVEL_LIMIT_RAD = 1.8f;

WingTiltSim::WingTiltSim() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool WingTiltSim::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz, matches the real AS5600 driver rate
	return true;
}

void WingTiltSim::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Pick up the latest tilt-motor command (same path the mixer's FunctionActuatorSet reads).
	vehicle_command_s cmd;

	while (_vehicle_command_sub.update(&cmd)) {
		if (cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR
		    && (int)(cmd.param7 + 0.5f) == 0 && PX4_ISFINITE(cmd.param1)) {
			_u = math::constrain((float)cmd.param1, -1.f, 1.f);
		}
	}

	const hrt_abstime now = hrt_absolute_time();
	float dt = (_last_run > 0) ? (now - _last_run) * 1e-6f : 0.01f;
	dt = math::constrain(dt, 0.f, 0.1f);
	_last_run = now;

	// Integrate the rate-source plant and clamp at the mechanical stops.
	_angle = math::constrain(_angle + MAX_RATE_RAD_S * _u * dt, -TRAVEL_LIMIT_RAD, TRAVEL_LIMIT_RAD);

	const float wrapped = matrix::wrap_pi(_angle);

	sensor_encoder_s report{};
	report.timestamp = now;
	report.device_id = 0x5123; // synthetic device id
	report.angle = wrapped;
	report.raw_count = (uint16_t)((wrapped + (float)M_PI) / (2.f * (float)M_PI) * 4096.f) & 0x0FFF;
	report.valid = true;
	report.zeroed = true;
	_sensor_encoder_pub.publish(report);
}

int WingTiltSim::print_status()
{
	const float r2d = 180.f / (float)M_PI;
	PX4_INFO("modeled wing angle: %.1f deg, command u: %.3f", (double)(_angle * r2d), (double)_u);
	return 0;
}

int WingTiltSim::task_spawn(int argc, char *argv[])
{
	WingTiltSim *instance = new WingTiltSim();

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

int WingTiltSim::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WingTiltSim::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
SITL-only plant model for the sun_tracker tilt wing. Integrates the tilt-motor command
(vehicle_command/DO_SET_ACTUATOR) into a wing angle and publishes it as `sensor_encoder`, so the
sun_tracker position loop closes in simulation without AS5600 hardware.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wing_tilt_sim", "simulation");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int wing_tilt_sim_main(int argc, char *argv[])
{
	return WingTiltSim::main(argc, argv);
}
