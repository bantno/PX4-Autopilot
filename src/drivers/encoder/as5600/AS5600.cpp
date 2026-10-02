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

	case 2:
		regdump();
		break;

	default:
		break;
	}
}

void AS5600::regdump()
{
	uint8_t cfg[9] = {};      // ZMCO, ZPOS(2), MPOS(2), MANG(2), CONF(2)
	uint8_t status = 0;
	uint8_t raw[2] = {};
	uint8_t ang[2] = {};
	uint8_t agc = 0;
	uint8_t mag[2] = {};

	if (read_reg(AS5600_REG_ZMCO, cfg, sizeof(cfg)) != PX4_OK
	    || read_reg(AS5600_REG_STATUS, &status, 1) != PX4_OK
	    || read_reg(AS5600_REG_RAW_ANGLE, raw, 2) != PX4_OK
	    || read_reg(AS5600_REG_ANGLE, ang, 2) != PX4_OK
	    || read_reg(AS5600_REG_AGC, &agc, 1) != PX4_OK
	    || read_reg(AS5600_REG_MAGNITUDE, mag, 2) != PX4_OK) {
		PX4_ERR("regdump: I2C read failed");
		return;
	}

	const unsigned zmco = cfg[0] & 0x03;
	const unsigned zpos = ((cfg[1] & 0x0F) << 8) | cfg[2];
	const unsigned mpos = ((cfg[3] & 0x0F) << 8) | cfg[4];
	const unsigned mang = ((cfg[5] & 0x0F) << 8) | cfg[6];
	const unsigned conf = ((cfg[7] & 0x3F) << 8) | cfg[8];
	const unsigned raw_angle = ((raw[0] & 0x0F) << 8) | raw[1];
	const unsigned angle = ((ang[0] & 0x0F) << 8) | ang[1];
	const unsigned magnitude = ((mag[0] & 0x0F) << 8) | mag[1];

	const unsigned pm = conf & 0x3;
	const unsigned hyst = (conf >> 2) & 0x3;
	const unsigned outs = (conf >> 4) & 0x3;
	const unsigned pwmf = (conf >> 6) & 0x3;
	const unsigned sf = (conf >> 8) & 0x3;
	const unsigned fth = (conf >> 10) & 0x7;
	const unsigned wd = (conf >> 13) & 0x1;

	static const char *pm_str[] = {"NOM (always on)", "LPM1 (5ms poll)", "LPM2 (20ms poll)", "LPM3 (100ms poll)"};
	static const char *hyst_str[] = {"OFF", "1 LSB", "2 LSB", "3 LSB"};
	static const char *sf_str[] = {"16x (2.2ms)", "8x (1.1ms)", "4x (0.55ms)", "2x (0.286ms)"};

	PX4_INFO("ZMCO %u (OTP burns used)  ZPOS %u  MPOS %u  MANG %u", zmco, zpos, mpos, mang);
	PX4_INFO("CONF 0x%04x: PM %s  HYST %s  OUTS %u  PWMF %u  SF %s  FTH %u  WD %u",
		 conf, pm_str[pm], hyst_str[hyst], outs, pwmf, sf_str[sf], fth, wd);
	PX4_INFO("STATUS 0x%02x: MD %u  ML(weak) %u  MH(strong) %u",
		 status, !!(status & AS5600_STATUS_MD), !!(status & AS5600_STATUS_ML), !!(status & AS5600_STATUS_MH));
	PX4_INFO("RAW_ANGLE %u  ANGLE %u  AGC %u  MAGNITUDE %u", raw_angle, angle, agc, magnitude);

	// The driver uses RAW_ANGLE as an unscaled 12-bit (4096/rev) angle; flag anything that breaks that.
	if (!(status & AS5600_STATUS_MD)) {
		PX4_WARN("magnet not detected: driver publishes valid=false");
	}

	if (status & (AS5600_STATUS_ML | AS5600_STATUS_MH)) {
		PX4_WARN("magnet field out of range (AGC saturated): angle may be nonlinear");
	}

	if (pm != 0) {
		PX4_WARN("low-power mode: RAW_ANGLE updates slower than the 10 ms sample rate");
	}

	if (wd) {
		PX4_WARN("watchdog on: chip drops to LPM3 after 1 min without motion");
	}

	if (zmco != 0 || zpos != 0 || mpos != 0 || mang != 0) {
		PX4_INFO("range programmed (ZPOS/MPOS/MANG/ZMCO): affects ANGLE only, RAW_ANGLE unaffected");
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
	PRINT_MODULE_USAGE_COMMAND_DESCR("regdump", "Print chip config/magnet registers and check them against driver assumptions");
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

	if (!strcmp(verb, "regdump")) {
		// Run in the shell's thread so the output reaches the (MAVLink) shell; each register read is a
		// single bus-locked I2C transaction, so it cannot interleave with the sampling work item.
		cli.custom1 = 2;
		return ThisDriver::module_custom_method(cli, iterator, false);
	}

	ThisDriver::print_usage();
	return -1;
}
