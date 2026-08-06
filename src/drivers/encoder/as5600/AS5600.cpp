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

#include "AS5600.hpp"

#include <matrix/math.hpp>

using namespace time_literals;

// Sample at 100 Hz: fast enough to close the wing position loop, light on the I2C bus.
static constexpr hrt_abstime AS5600_SAMPLE_INTERVAL{10_ms};

AS5600::AS5600(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

AS5600::~AS5600()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int AS5600::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

	ScheduleOnInterval(AS5600_SAMPLE_INTERVAL);
	return PX4_OK;
}

int AS5600::probe()
{
	// The AS5600 has no chip-id register; confirm presence by a successful STATUS read.
	uint8_t status = 0;
	return read_reg(AS5600_REG_STATUS, &status, 1);
}

int AS5600::read_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
	return transfer(&reg, 1, buf, len);
}

void AS5600::RunImpl()
{
	collect();
}

int AS5600::collect()
{
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	uint8_t status = 0;
	uint8_t raw[2] = {};

	if (read_reg(AS5600_REG_STATUS, &status, 1) != PX4_OK ||
	    read_reg(AS5600_REG_RAW_ANGLE, raw, 2) != PX4_OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	const bool magnet_detected = (status & AS5600_STATUS_MD);
	const uint16_t count = (uint16_t)(((raw[0] & 0x0F) << 8) | raw[1]);

	// Latch the boot-time level reference on the first valid reading, or on a reset request.
	if (magnet_detected && (!_zeroed || _zero_request)) {
		_zero_count = count;
		_last_count = count;
		_accum_counts = 0;
		_zeroed = true;
		_zero_request = false;
	}

	// Multi-turn accumulation: with the encoder geared up off the wing spar, the wing's travel
	// can exceed one encoder revolution, so a wrapped absolute angle is ambiguous. Integrate the
	// per-sample step instead (sign-extended 12-bit shortest-path difference — valid while the
	// shaft moves less than half a revolution between samples, far above any physical rate).
	const int16_t step = (int16_t)((uint16_t)((count - _last_count) << 4)) >> 4;
	_last_count = count;

	if (_zeroed) {
		_accum_counts += step;
	}

	const float angle = _accum_counts * (2.0f * M_PI_F / AS5600_RESOLUTION);

	sensor_encoder_s report{};
	report.timestamp = hrt_absolute_time();
	report.device_id = get_device_id();
	report.angle = angle;
	report.raw_count = count;
	report.valid = magnet_detected && _zeroed;
	report.zeroed = _zeroed;
	_sensor_encoder_pub.publish(report);

	perf_end(_sample_perf);

	// Suppress unused-variable warning if asserts are disabled.
	(void)timestamp_sample;

	return PX4_OK;
}

void AS5600::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom1) {
	case 1:
		// Re-latch the level reference on the next sample (hold the wing level first).
		_zero_request = true;
		PX4_INFO("AS5600: zero reference will be re-latched on next sample");
		break;

	default:
		break;
	}
}

void AS5600::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("zeroed: %s, zero_count: %u", _zeroed ? "yes" : "no", _zero_count);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void AS5600::print_usage()
{
	PRINT_MODULE_USAGE_NAME("as5600", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("encoder");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(AS5600_BASEADDR);
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Re-latch the level (zero) reference (hold wing level)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" __EXPORT int as5600_main(int argc, char *argv[])
{
	using ThisDriver = AS5600;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.i2c_address = AS5600_BASEADDR;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ENC_DEVTYPE_AS5600);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "reset")) {
		cli.custom1 = 1;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
