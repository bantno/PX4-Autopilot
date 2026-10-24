/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * @file ina228_charger.cpp
 *
 * Driver for an I2C attached INA228 on the battery charge path. Derived from the stock ina228
 * driver, but publishes battery_charging and also reports the INA228's power, die temperature,
 * diagnostic flags and hardware ENERGY/CHARGE accumulators.
 */

#include "ina228_charger.h"

#include <mathlib/mathlib.h>


INA228Charger::INA228Charger(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config),
	_sample_perf(perf_alloc(PC_ELAPSED, "ina228chg_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ina228chg_com_err")),
	_collection_errors(perf_alloc(PC_COUNT, "ina228chg_collection_err")),
	_measure_errors(perf_alloc(PC_COUNT, "ina228chg_measurement_err"))
{
	float fvalue = MAX_CURRENT;
	_max_current = fvalue;
	param_t ph = param_find("INA228C_CURRENT");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_max_current = fvalue;
	}

	fvalue = INA228_SHUNT;
	_rshunt = fvalue;
	ph = param_find("INA228C_SHUNT");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_rshunt = fvalue;
	}

	// According to page 8.2.2.1, page 36/48 of the INA228 interface datasheet (Rev. A),
	// the requirement is: R_SHUNT < V_SENSE_MAX / I_MAX
	// therefore: R_SHUNT * I_MAX < V_SENSE_MAX
	// and so if V_SENSE_MAX is bigger, we need to use the bigger ADC range to avoid
	// the device from capping the measured current. Otherwise the ±40.96 mV range gives 4x the resolution.

	const float v_sense_max = _rshunt * _max_current;

	if (v_sense_max > INA228_ADCRANGE_LOW_V_SENSE) {
		_range = INA228_ADCRANGE_HIGH;

	} else {
		_range = INA228_ADCRANGE_LOW;
	}

	ph = param_find("INA228C_CONFIG");
	int32_t value = INA228_ADCCONFIG;
	_config = (uint16_t)value;

	if (ph != PARAM_INVALID && param_get(ph, &value) == PX4_OK) {
		_config = (uint16_t)value;
	}

	_mode_triggered = ((_config & INA228_MODE_MASK) >> INA228_MODE_SHIFTS) <=
			  ((INA228_MODE_TEMP_SHUNT_BUS_TRIG & INA228_MODE_MASK) >>
			   INA228_MODE_SHIFTS);

	_current_lsb = _max_current / DN_MAX;
	_power_lsb = 3.2f * _current_lsb;
}

INA228Charger::~INA228Charger()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_collection_errors);
	perf_free(_measure_errors);
}

int INA228Charger::read(uint8_t address, int16_t &data)
{
	// read desired little-endian value via I2C
	int16_t received_bytes;
	const int ret = transfer(&address, 1, (uint8_t *)&received_bytes, sizeof(received_bytes));

	if (ret == PX4_OK) {
		data = swap16(received_bytes);

	} else {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
	}

	return ret;
}

int INA228Charger::read(uint8_t address, uint16_t &data)
{
	// read desired little-endian value via I2C
	uint16_t received_bytes;
	const int ret = transfer(&address, 1, (uint8_t *)&received_bytes, sizeof(received_bytes));

	if (ret == PX4_OK) {
		data = swap16(received_bytes);

	} else {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
	}

	return ret;
}

int INA228Charger::read20(uint8_t address, int32_t &data)
{
	uint32_t raw{0};
	const int ret = read24(address, raw);

	if (ret == PX4_OK) {
		// The 20-bit result sits in bits 23:4, the low nibble is reserved
		raw >>= 4;

		// Sign extend the 20-bit two's complement value
		data = (raw & 0x80000) ? (int32_t)(raw | 0xFFF00000) : (int32_t)raw;
	}

	return ret;
}

int INA228Charger::read24(uint8_t address, uint32_t &data)
{
	// 24-bit registers are sent MSB first
	uint8_t buf[3] {};
	const int ret = transfer(&address, 1, buf, sizeof(buf));

	if (ret == PX4_OK) {
		data = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

	} else {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
	}

	return ret;
}

int INA228Charger::read40(uint8_t address, uint64_t &data)
{
	// 40-bit accumulator registers are sent MSB first
	uint8_t buf[5] {};
	const int ret = transfer(&address, 1, buf, sizeof(buf));

	if (ret == PX4_OK) {
		data = 0;

		for (unsigned i = 0; i < sizeof(buf); i++) {
			data = (data << 8) | buf[i];
		}

	} else {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
	}

	return ret;
}

int INA228Charger::write(uint8_t address, uint16_t value)
{
	uint8_t data[3] = {address, ((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};
	return transfer(data, sizeof(data), nullptr, 0);
}

int INA228Charger::write(uint8_t address, int16_t value)
{
	uint8_t data[3] = {address, ((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};
	return transfer(data, sizeof(data), nullptr, 0);
}

int INA228Charger::reset_accumulators()
{
	// RSTACC self-clears; the ADC range bit must be preserved in the same write
	const int ret = write(INA228_REG_CONFIG, (uint16_t)(INA228_RSTACC_CLEAR | _range));

	if (ret == PX4_OK) {
		_accum_overflow = false;
	}

	return ret;
}

int
INA228Charger::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != PX4_OK) {
		return ret;
	}

	// Full reset, which also zeroes the ENERGY and CHARGE accumulators
	write(INA228_REG_CONFIG, (uint16_t)(INA228_RST_RESET | _range));

	_cal = INA228_CONST * _current_lsb * _rshunt;

	if (_range == INA228_ADCRANGE_LOW) {
		_cal *= 4;
	}

	if (write(INA228_REG_SHUNTCAL, _cal) < 0) {
		return -3;
	}

	// Set the CONFIG for max I

	write(INA228_REG_CONFIG, (uint16_t) _range);

	// If we run in continuous mode then start it here (the accumulators only integrate in continuous mode)

	if (!_mode_triggered) {

		ret = write(INA228_REG_ADCCONFIG, _config);

	} else {
		ret = PX4_OK;
	}

	_accum_overflow = false;

	start();
	_sensor_ok = true;

	_initialized = ret == PX4_OK;
	return ret;
}

int
INA228Charger::force_init()
{
	int ret = init();

	start();

	return ret;
}

int
INA228Charger::probe()
{
	uint16_t value{0};

	if (read(INA228_MANUFACTURER_ID, value) != PX4_OK || value != INA228_MFG_ID_TI) {
		PX4_DEBUG("probe mfgid %d", value);
		return -1;
	}

	if (read(INA228_DEVICE_ID, value) != PX4_OK || INA228_DEVICEID(value) != INA228_MFG_DIE) {
		PX4_DEBUG("probe die id %d", value);
		return -1;
	}

	return PX4_OK;
}

int
INA228Charger::measure()
{
	int ret = PX4_OK;

	if (_mode_triggered) {
		ret = write(INA228_REG_ADCCONFIG, _config);

		if (ret < 0) {
			perf_count(_comms_errors);
			PX4_DEBUG("i2c::transfer returned %d", ret);
		}
	}

	return ret;
}

int
INA228Charger::collect()
{
	perf_begin(_sample_perf);

	if (_parameter_update_sub.updated()) {
		// Read from topic to clear updated flag
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);

		updateParams();
	}

	// read from the sensor
	// Note: If the shunt is connected backwards, then current and charge will be negative but otherwise valid.
	uint64_t charge_raw{0};
	bool success{true};
	success = success && (read20(INA228_REG_VSBUS, _bus_voltage) == PX4_OK);
	success = success && (read20(INA228_REG_CURRENT, _current) == PX4_OK);
	success = success && (read24(INA228_REG_POWER, _power) == PX4_OK);
	success = success && (read(INA228_REG_DIETEMP, _temperature) == PX4_OK);
	success = success && (read40(INA228_REG_ENERGY, _energy) == PX4_OK);
	success = success && (read40(INA228_REG_CHARGE, charge_raw) == PX4_OK);
	success = success && (read(INA228_REG_DIAG_ALRT, _diag) == PX4_OK);

	if (success) {
		// CHARGE is 40-bit two's complement
		_charge = (charge_raw & (UINT64_C(1) << 39)) ? (int64_t)(charge_raw | ~INA228_CHARGE_MASK) : (int64_t)charge_raw;

		// The overflow flags clear once the accumulator is read, so latch them until the next reset
		_accum_overflow = _accum_overflow || (_diag & (INA228_ENERGYOF | INA228_CHARGEOF));

	} else {
		PX4_DEBUG("error reading from sensor");
		_bus_voltage = _power = _current = 0;
		_temperature = 0;
	}

	battery_charging_s &report = _report;
	report.connected = success;
	report.voltage_v = static_cast<float>(_bus_voltage) * INA228_VSCALE;
	report.current_a = static_cast<float>(_current) * _current_lsb;
	// POWER is unsigned on the chip; carry the current's sign so positive still means into the battery
	report.power_w = copysignf(static_cast<float>(_power) * _power_lsb, report.current_a);

	if (success) {
		report.energy_wh = static_cast<float>(static_cast<double>(_energy) * (double)(INA228_ENERGY_SCALE * _current_lsb) / 3600.0);
		report.charge_ah = static_cast<float>(static_cast<double>(_charge) * (double)_current_lsb / 3600.0);
		report.temperature_c = static_cast<float>(_temperature) * INA228_TSCALE;
		report.diag_flags = _diag;
	}

	report.accum_overflow = _accum_overflow;
	report.timestamp = hrt_absolute_time();
	_battery_charging_pub.publish(report);

	perf_end(_sample_perf);

	if (success) {
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}
}

void
INA228Charger::start()
{
	ScheduleClear();

	/* reset the report ring and state machine */
	_collect_phase = false;

	_measure_interval = INA228_CONVERSION_INTERVAL;

	/* schedule a cycle to start things */
	ScheduleDelayed(5);
}

void
INA228Charger::RunImpl()
{
	if (_initialized) {
		if (_collect_phase) {
			/* perform collection */
			if (collect() != PX4_OK) {
				perf_count(_collection_errors);
				/* if error restart the measurement state machine */
				start();
				return;
			}

			/* next phase is measurement */
			_collect_phase = !_mode_triggered;

			if (_measure_interval > INA228_CONVERSION_INTERVAL) {
				/* schedule a fresh cycle call when we are ready to measure again */
				ScheduleDelayed(_measure_interval - INA228_CONVERSION_INTERVAL);
				return;
			}
		}

		/* Measurement  phase */

		/* Perform measurement */
		if (measure() != PX4_OK) {
			perf_count(_measure_errors);
		}

		/* next phase is collection */
		_collect_phase = true;

		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(INA228_CONVERSION_INTERVAL);

	} else {
		battery_charging_s report{};
		report.connected = false;
		report.timestamp = hrt_absolute_time();
		_battery_charging_pub.publish(report);

		if (init() != PX4_OK) {
			ScheduleDelayed(INA228_INIT_RETRY_INTERVAL_US);
		}
	}
}

void
INA228Charger::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom1) {
	case 1:
		if (reset_accumulators() == PX4_OK) {
			PX4_INFO("energy and charge accumulators reset");

		} else {
			PX4_ERR("accumulator reset failed");
		}

		break;
	}
}

void
INA228Charger::print_status()
{
	I2CSPIDriverBase::print_status();

	if (_initialized) {
		perf_print_counter(_sample_perf);
		perf_print_counter(_comms_errors);

		printf("poll interval:  %u \n", _measure_interval);
		printf("ADC range:      %s, shunt %.4f Ohm, max current %.2f A\n",
		       _range == INA228_ADCRANGE_LOW ? "+-40.96 mV" : "+-163.84 mV", (double)_rshunt, (double)_max_current);
		printf("voltage:        %.3f V\n", (double)_report.voltage_v);
		printf("current:        %.4f A\n", (double)_report.current_a);
		printf("power:          %.3f W\n", (double)_report.power_w);
		printf("energy:         %.4f Wh (since driver start / reset_acc)\n", (double)_report.energy_wh);
		printf("charge:         %.5f Ah\n", (double)_report.charge_ah);
		printf("die temp:       %.2f C\n", (double)_report.temperature_c);
		printf("diag flags:     0x%04x%s%s\n", _report.diag_flags,
		       (_report.diag_flags & INA228_MEMSTAT) ? "" : " MEMSTAT-ERROR",
		       _report.accum_overflow ? " ACCUM-OVERFLOW" : "");

	} else {
		PX4_INFO("Device not initialized. Retrying every %d ms until the charger monitor is powered.",
			 INA228_INIT_RETRY_INTERVAL_US / 1000);
	}
}
