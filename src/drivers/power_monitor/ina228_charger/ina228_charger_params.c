/****************************************************************************
 *
 *   Copyright (c) 2019-2026 PX4 Development Team. All rights reserved.
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
 * Enable INA228 Battery Charging Monitor
 *
 * For systems with an INA228 on the battery charge path, this should be set to true
 *
 * @group Sensors
 * @boolean
 * @reboot_required true
*/
PARAM_DEFINE_INT32(SENS_EN_INA228C, 0);

/**
 * INA228 Charging Monitor ADC Config
 *
 * Raw ADC_CONFIG register value. The default (63779 = 0xF923) is continuous
 * temperature + shunt + bus conversions, 540 us each, 64 averages. The ENERGY
 * and CHARGE accumulators only integrate in a continuous mode.
 *
 * @group Sensors
 * @min 0
 * @max 65535
 * @decimal 1
 * @increment 1
 * @reboot_required true
*/
PARAM_DEFINE_INT32(INA228C_CONFIG, 63779);

/**
 * INA228 Charging Monitor Max Current
 *
 * Sets the current LSB (max / 2^19). If shunt * max current is below 40.96 mV the
 * driver uses the INA228's low ADC range for 4x finer resolution.
 *
 * @group Sensors
 * @unit A
 * @min 0.1
 * @max 327.68
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(INA228C_CURRENT, 8.0f);

/**
 * INA228 Charging Monitor Shunt
 *
 * @group Sensors
 * @unit Ohm
 * @min 0.000000001
 * @max 0.1
 * @decimal 10
 * @increment .000000001
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(INA228C_SHUNT, 0.002f);
