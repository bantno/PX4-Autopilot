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

#include "acconeer_xm125.hpp"

#include <lib/mathlib/mathlib.h>

AcconeerXM125::AcconeerXM125(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	ModuleParams(nullptr),
	_px4_rangefinder(get_device_id(), config.rotation)
{
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_XM125);

	// 60 GHz integrated-antenna beamwidth is a few tens of degrees; report a
	// conservative value (informational - EKF2 handles validity gating).
	_px4_rangefinder.set_fov(math::radians(30.f));

	// Allow a few retries: the module NACKs while its MCU is busy.
	_retries = 3;
}

AcconeerXM125::~AcconeerXM125()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int AcconeerXM125::init()
{
	updateParams();

	const float min_dist = _param_xm125_min_dist.get();
	const float max_dist = _param_xm125_max_dist.get();

	_start_mm = (uint32_t)roundf(math::max(min_dist, 0.f) * 1000.f);
	_end_mm = (uint32_t)roundf(math::max(max_dist, min_dist) * 1000.f);
	_reflector_shape = (uint32_t)_param_xm125_refl_shp.get();
	_strength_min = (uint32_t)math::max(_param_xm125_str_min.get(), INT32_C(0));

	_px4_rangefinder.set_min_distance(min_dist);
	_px4_rangefinder.set_max_distance(max_dist);

	// do I2C init (and probe) first
	int ret = I2C::init();

	if (ret != PX4_OK) {
		return ret;
	}

	_state = State::Configure;
	ScheduleNow();
	return PX4_OK;
}

int AcconeerXM125::probe()
{
	// A successful read of a sane version register confirms the Distance
	// Detector application is present and answering on this address.
	uint32_t version = 0;

	if (readReg(Register::VERSION, version) != PX4_OK) {
		return -EIO;
	}

	if (version == 0 || version == 0xffffffff) {
		return -EIO;
	}

	return PX4_OK;
}

int AcconeerXM125::readReg(uint16_t reg, uint32_t &value)
{
	// Write the 16-bit register address (big-endian), repeated start, read 4 data bytes.
	const uint8_t addr[2] {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xff)};
	uint8_t rx[4] {};

	if (transfer(addr, sizeof(addr), rx, sizeof(rx)) != PX4_OK) {
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	value = ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | (uint32_t)rx[3];
	return PX4_OK;
}

int AcconeerXM125::writeReg(uint16_t reg, uint32_t value)
{
	// 16-bit address followed by 32-bit data, both big-endian.
	const uint8_t buf[6] {
		(uint8_t)(reg >> 8), (uint8_t)(reg & 0xff),
		(uint8_t)(value >> 24), (uint8_t)(value >> 16), (uint8_t)(value >> 8), (uint8_t)(value & 0xff)
	};

	if (transfer(buf, sizeof(buf), nullptr, 0) != PX4_OK) {
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	return PX4_OK;
}

int AcconeerXM125::writeConfiguration()
{
	int ret = PX4_OK;

	// Near-range blanking: START is set beyond the in-fuselage standoff so the
	// fuselage wall / radome and the ~6 cm direct-leakage zone are never reported.
	ret |= writeReg(Register::START, _start_mm);
	ret |= writeReg(Register::END, _end_mm);

	// Strongest peak near nadir is the surface (specular over water); CFAR adapts
	// the detection threshold to the changing noise floor; PLANAR weights large
	// flat reflectors (water) correctly with range.
	ret |= writeReg(Register::PEAK_SORTING, PEAK_SORT_STRONGEST);
	ret |= writeReg(Register::THRESHOLD_METHOD, THRESHOLD_CFAR);
	ret |= writeReg(Register::REFLECTOR_SHAPE, _reflector_shape);

	// Apply the configuration and run sensor + detector calibration.
	ret |= writeReg(Register::COMMAND, Command::APPLY_CONFIG_AND_CALIBRATE);

	return ret;
}

void AcconeerXM125::collect()
{
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	uint32_t result = 0;

	if (readReg(Register::RESULT, result) != PX4_OK) {
		perf_end(_sample_perf);
		return;
	}

	_num_distances = result & RESULT_NUM_DISTANCES_MASK;
	_near_start_edge = result & RESULT_NEAR_START_EDGE;
	const bool measure_error = result & RESULT_MEASURE_DIST_ERROR;

	if (_num_distances == 0 || measure_error) {
		// No valid surface return (e.g. specular dropout off-nadir). Publish
		// quality 0 so EKF2's quality hysteresis blocks fusion and holds terrain.
		_px4_rangefinder.update(timestamp_sample, _last_distance_m, 0);
		perf_end(_sample_perf);
		return;
	}

	// Peak 0 is the selected candidate (strongest, inside the range window).
	uint32_t distance_mm = 0;
	uint32_t strength = 0;

	if (readReg(Register::PEAK0_DISTANCE, distance_mm) != PX4_OK
	    || readReg(Register::PEAK0_STRENGTH, strength) != PX4_OK) {
		perf_end(_sample_perf);
		return;
	}

	const float distance_m = distance_mm * 1e-3f;

	// Binary strength gate (EKF2 only distinguishes quality 0 vs >0). Default
	// threshold 0 accepts any CFAR-detected peak.
	const int8_t quality = (strength >= _strength_min) ? 100 : 0;

	_last_distance_m = distance_m;
	_px4_rangefinder.update(timestamp_sample, distance_m, quality);

	perf_end(_sample_perf);
}

void AcconeerXM125::RunImpl()
{
	switch (_state) {
	case State::Configure:
		if (writeConfiguration() == PX4_OK) {
			_state = State::WaitConfig;
			ScheduleDelayed(POLL_INTERVAL);

		} else {
			ScheduleDelayed(CONFIG_RETRY_INTERVAL);
		}

		break;

	case State::WaitConfig: {
			uint32_t status = 0;

			if (readReg(Register::DETECTOR_STATUS, status) != PX4_OK) {
				_state = State::Configure;
				ScheduleDelayed(CONFIG_RETRY_INTERVAL);
				break;
			}

			if (status & STATUS_ERROR_MASK) {
				PX4_DEBUG("config/calibration error: 0x%08lx", (unsigned long)status);
				_state = State::Configure;
				ScheduleDelayed(CONFIG_RETRY_INTERVAL);

			} else if (status & STATUS_BUSY) {
				// calibration still running
				ScheduleDelayed(POLL_INTERVAL);

			} else if ((status & STATUS_CONFIG_APPLY_OK) && (status & STATUS_DETECTOR_CALIBRATE_OK)) {
				_state = State::Measure;
				ScheduleDelayed(MEASUREMENT_INTERVAL);

			} else {
				// not busy but config/calibration did not complete - retry
				_state = State::Configure;
				ScheduleDelayed(CONFIG_RETRY_INTERVAL);
			}

			break;
		}

	case State::Measure:
		if (writeReg(Register::COMMAND, Command::MEASURE_DISTANCE) == PX4_OK) {
			_state = State::Collect;
			ScheduleDelayed(POLL_INTERVAL);

		} else {
			ScheduleDelayed(MEASUREMENT_INTERVAL);
		}

		break;

	case State::Collect: {
			uint32_t status = 0;

			if (readReg(Register::DETECTOR_STATUS, status) != PX4_OK) {
				_state = State::Measure;
				ScheduleDelayed(MEASUREMENT_INTERVAL);
				break;
			}

			if (status & STATUS_ERROR_MASK) {
				PX4_DEBUG("measure error: 0x%08lx", (unsigned long)status);
				_state = State::Configure;
				ScheduleDelayed(CONFIG_RETRY_INTERVAL);

			} else if (status & STATUS_BUSY) {
				// measurement still running
				ScheduleDelayed(POLL_INTERVAL);

			} else {
				collect();
				_state = State::Measure;
				ScheduleDelayed(MEASUREMENT_INTERVAL);
			}

			break;
		}
	}
}

void AcconeerXM125::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("range window: %lu - %lu mm, reflector shape: %s",
		 (unsigned long)_start_mm, (unsigned long)_end_mm,
		 _reflector_shape == 2 ? "planar" : "generic");
	PX4_INFO("last distance: %.2f m, peaks: %u%s",
		 (double)_last_distance_m, _num_distances,
		 _near_start_edge ? " (near start edge)" : "");
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void AcconeerXM125::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for the Acconeer XM125 (A121 60 GHz pulsed coherent radar) running the
Acconeer I2C Distance Detector application. Intended as a downward-facing radar
altimeter to aid landing, including over water. Configure the trusted range window and
near-range blanking with the XM125_* parameters.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("acconeer_xm125", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(XM125_BASEADDR);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int acconeer_xm125_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = AcconeerXM125;
	BusCLIArguments cli{true, false};
	cli.rotation = (Rotation)distance_sensor_s::ROTATION_DOWNWARD_FACING;
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = XM125_BASEADDR;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_XM125);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
