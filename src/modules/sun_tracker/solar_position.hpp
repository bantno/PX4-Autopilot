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
 * @file solar_position.hpp
 *
 * Self-contained low-precision solar position (sun azimuth/elevation) computation.
 * Based on the NOAA / "Astronomical Almanac" low-accuracy algorithm (accuracy ~0.1 deg
 * over the years 1950-2050), which is far better than required for solar panel pointing.
 */

#pragma once

#include <stdint.h>

namespace solar
{

/**
 * Compute the sun's apparent topocentric azimuth and elevation for an observer.
 *
 * @param utc_usec       UTC time in microseconds since the Unix epoch (1970-01-01).
 * @param lat_deg        Observer geodetic latitude  [deg], positive North.
 * @param lon_deg        Observer geodetic longitude [deg], positive East.
 * @param azimuth_rad    [out] Sun azimuth [rad], measured clockwise from true North
 *                             (North = 0, East = +PI/2), wrapped to [0, 2*PI).
 * @param elevation_rad  [out] Sun elevation [rad] above the horizon (negative = below horizon).
 */
void solar_position(uint64_t utc_usec, double lat_deg, double lon_deg,
		    float &azimuth_rad, float &elevation_rad);

} // namespace solar
