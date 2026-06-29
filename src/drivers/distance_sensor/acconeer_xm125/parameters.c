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
 * Acconeer XM125 radar rangefinder (I2C)
 *
 * Enable the Acconeer XM125 Distance Detector driver at boot.
 *
 * @reboot_required true
 *
 * @boolean
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_EN_XM125, 0);

/**
 * XM125 minimum distance / near-range blanking
 *
 * Sets the detector START range. Reflections closer than this (e.g. the fuselage
 * wall/radome the sensor is mounted behind, and the radar's direct-leakage zone)
 * are not reported as peaks. Set this just beyond the mounting standoff.
 *
 * @min 0.1
 * @max 7.0
 * @unit m
 * @decimal 2
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(XM125_MIN_DIST, 0.30f);

/**
 * XM125 maximum distance
 *
 * Sets the detector END range and the reported max_distance. Measurements beyond
 * this are not trusted. The XM125 is only reliable at close range.
 *
 * @min 0.5
 * @max 7.0
 * @unit m
 * @decimal 2
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(XM125_MAX_DIST, 7.0f);

/**
 * XM125 reflector shape
 *
 * Selects how the detector weights peaks with range. Use Planar for flat
 * specular surfaces such as water (sorts by amplitude*range); use Generic for
 * everything else (sorts by amplitude*range^2).
 *
 * @value 1 Generic
 * @value 2 Planar
 * @min 1
 * @max 2
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_INT32(XM125_REFL_SHP, 2);

/**
 * XM125 minimum peak strength
 *
 * Peaks with a raw strength below this value are published with signal quality 0
 * (so EKF2 does not fuse them). 0 accepts any peak that passes the on-sensor CFAR
 * threshold. Raise to suppress weak/spurious returns.
 *
 * @min 0
 * @reboot_required true
 * @group Sensors
 */
PARAM_DEFINE_INT32(XM125_STR_MIN, 0);
