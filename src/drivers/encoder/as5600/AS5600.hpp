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
 * @file AS5600.hpp
 *
 * Driver for the AMS AS5600 12-bit magnetic rotary position sensor connected via I2C.
 *
 * Reports the wing tilt angle relative to a zero (wing level with the fuselage body-X axis) reference.
 * If AS5600_ZERO holds a saved raw count, that is the reference and the angle is absolute from
 * power-on (the first reading is wrapped to [-PI, PI) about it). Otherwise (AS5600_ZERO = -1) the
 * wing is assumed level at startup and the first valid reading (magnet detected) latches the
 * reference. `as5600 reset` re-latches at the current position and saves it to AS5600_ZERO.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_encoder.h>

/* Configuration constants */
#define AS5600_BASEADDR        0x36     // fixed I2C address of the AS5600
#define AS5600_RESOLUTION      4096.0f  // 12-bit counts per revolution

/* Register map (8-bit register addressing) */
#define AS5600_REG_ZMCO        0x00     // ZMCO..CONF (0x00-0x08) read as one burst for regdump
#define AS5600_REG_STATUS      0x0B     // MH (bit3), ML (bit4), MD (bit5)
#define AS5600_REG_RAW_ANGLE   0x0C     // 0x0C high (bits 11:8), 0x0D low (bits 7:0)
#define AS5600_REG_ANGLE       0x0E
#define AS5600_REG_AGC         0x1A
#define AS5600_REG_MAGNITUDE   0x1B

#define AS5600_STATUS_MH       (1 << 3) // magnet too strong
#define AS5600_STATUS_ML       (1 << 4) // magnet too weak
#define AS5600_STATUS_MD       (1 << 5) // magnet detected

class AS5600 : public device::I2C, public I2CSPIDriver<AS5600>
{
public:
	AS5600(const I2CSPIDriverConfig &config);
	~AS5600() override;

	static void print_usage();

	void print_status() override;

	int init() override;

	void RunImpl();

	// Custom runtime verb dispatch (e.g. "reset" to re-latch the level reference).
	void custom_method(const BusCLIArguments &cli) override;

private:
	int probe() override;

	int collect();

	void regdump();

	// Read len bytes starting at reg using the AS5600's auto-increment.
	int read_reg(uint8_t reg, uint8_t *buf, uint8_t len);

	uORB::PublicationMulti<sensor_encoder_s> _sensor_encoder_pub{ORB_ID(sensor_encoder)};

	uint16_t _zero_count{0};       // raw count latched as "wing level" (tilt = 0)
	uint16_t _last_count{0};       // previous raw count, for the multi-turn step accumulation
	int32_t _accum_counts{0};      // accumulated counts relative to the zero reference (multi-turn)
	bool _zeroed{false};           // true once the zero reference has been applied
	int32_t _saved_zero{-1};       // AS5600_ZERO at startup (-1 = latch at boot)
	param_t _param_zero{PARAM_INVALID};
	volatile bool _zero_request{false}; // set by the "reset" verb to re-latch the reference

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
};
