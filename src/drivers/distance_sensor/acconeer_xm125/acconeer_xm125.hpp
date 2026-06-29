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
 * @file acconeer_xm125.hpp
 *
 * Driver for the Acconeer XM125 (A121 60 GHz pulsed coherent radar) running the
 * Acconeer I2C Distance Detector application, connected via I2C.
 *
 * Used as a downward-facing radar altimeter to aid landing (incl. over water).
 * The module returns up to 10 detected peaks; the driver selects the strongest
 * one inside the configured range window and publishes it on the distance_sensor
 * topic. Outlier/consistency rejection is delegated to EKF2 (EKF2_RNG_GATE /
 * EKF2_RNG_K_GATE / EKF2_RNG_QLTY_T) which is what survives specular dropout.
 */

#pragma once

#include <px4_log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>

using namespace time_literals;

/* Default I2C address of the XM125 Distance Detector application */
#define XM125_BASEADDR 0x52

class AcconeerXM125 : public device::I2C, public I2CSPIDriver<AcconeerXM125>, public ModuleParams
{
public:
	AcconeerXM125(const I2CSPIDriverConfig &config);
	~AcconeerXM125() override;

	static void print_usage();

	int init() override;

	void print_status() override;

	/**
	 * Drives the configure -> measure -> collect state machine. Each entry either
	 * issues a register command or polls the BUSY bit, then reschedules itself.
	 */
	void RunImpl();

private:
	/* Distance Detector register map (16-bit addresses, 32-bit big-endian data) */
	enum Register : uint16_t {
		VERSION          = 0x0000,
		DETECTOR_STATUS  = 0x0003,
		RESULT           = 0x0010,
		PEAK0_DISTANCE   = 0x0011,
		PEAK0_STRENGTH   = 0x001b,
		START            = 0x0040,
		END              = 0x0041,
		THRESHOLD_METHOD = 0x0046,
		PEAK_SORTING     = 0x0047,
		REFLECTOR_SHAPE  = 0x004b,
		COMMAND          = 0x0100,
	};

	/* COMMAND register values */
	enum Command : uint32_t {
		APPLY_CONFIG_AND_CALIBRATE = 1,
		MEASURE_DISTANCE           = 2,
	};

	/* THRESHOLD_METHOD / PEAK_SORTING / REFLECTOR_SHAPE enum values */
	static constexpr uint32_t THRESHOLD_CFAR     = 3;
	static constexpr uint32_t PEAK_SORT_STRONGEST = 2;

	/* DETECTOR_STATUS register bit masks */
	static constexpr uint32_t STATUS_BUSY                  = 0x80000000;
	static constexpr uint32_t STATUS_CONFIG_APPLY_OK       = 0x00000080;
	static constexpr uint32_t STATUS_DETECTOR_CALIBRATE_OK = 0x00000200;
	static constexpr uint32_t STATUS_ERROR_MASK            = 0x13ff0000; // any RSS/config/sensor/detector error bit

	/* RESULT register bit masks */
	static constexpr uint32_t RESULT_NUM_DISTANCES_MASK   = 0x0000000f;
	static constexpr uint32_t RESULT_NEAR_START_EDGE      = 0x00000100;
	static constexpr uint32_t RESULT_MEASURE_DIST_ERROR   = 0x00000400;

	/* Scheduling intervals */
	static constexpr uint32_t MEASUREMENT_INTERVAL = 100_ms; // ~10 Hz publish rate
	static constexpr uint32_t POLL_INTERVAL        = 10_ms;  // BUSY-bit poll while measuring
	static constexpr uint32_t CONFIG_RETRY_INTERVAL = 500_ms;

	enum class State {
		Configure,    // write config registers + apply/calibrate command
		WaitConfig,   // poll until calibration finished
		Measure,      // issue a MEASURE_DISTANCE command
		Collect,      // poll until measurement done, then read + publish
	};

	int probe() override;

	/** Read a 32-bit register (big-endian address + big-endian data). */
	int readReg(uint16_t reg, uint32_t &value);

	/** Write a 32-bit register (big-endian address + big-endian data). */
	int writeReg(uint16_t reg, uint32_t value);

	/** Push the detector configuration and kick off apply+calibrate. */
	int writeConfiguration();

	/** Read the latest peak and publish it (or publish quality 0 if no valid peak). */
	void collect();

	PX4Rangefinder _px4_rangefinder;

	State _state{State::Configure};

	// Configuration derived from parameters (resolved in init()).
	uint32_t _start_mm{300};
	uint32_t _end_mm{7000};
	uint32_t _reflector_shape{2}; // PLANAR
	uint32_t _strength_min{0};

	float _last_distance_m{NAN};
	uint8_t _num_distances{0};
	bool _near_start_edge{false};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::XM125_MIN_DIST>) _param_xm125_min_dist,
		(ParamFloat<px4::params::XM125_MAX_DIST>) _param_xm125_max_dist,
		(ParamInt<px4::params::XM125_REFL_SHP>)   _param_xm125_refl_shp,
		(ParamInt<px4::params::XM125_STR_MIN>)    _param_xm125_str_min
	)
};
