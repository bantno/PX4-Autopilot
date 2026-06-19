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

#include "solar_position.hpp"

#include <math.h>

namespace
{
constexpr double DEG2RAD_D = M_PI / 180.0;
constexpr float DEG2RAD_F = (float)(M_PI / 180.0);
constexpr float RAD2DEG_F = (float)(180.0 / M_PI);
constexpr float TWO_PI_F = (float)(2.0 * M_PI);
constexpr float PI_F = (float)M_PI;

// Wrap an angle in degrees to [0, 360). Done in double because the inputs carry a
// large-magnitude term (rate * days-since-J2000) whose fractional part must survive.
double wrap360(double deg)
{
	deg = fmod(deg, 360.0);

	if (deg < 0.0) {
		deg += 360.0;
	}

	return deg;
}

float constrainf(float v, float lo, float hi)
{
	return (v < lo) ? lo : (v > hi) ? hi : v;
}
} // namespace

namespace solar
{

void solar_position(uint64_t utc_usec, double lat_deg, double lon_deg,
		    float &azimuth_rad, float &elevation_rad)
{
	// Days since the J2000.0 epoch (2000-01-01 12:00 UTC = JD 2451545.0).
	// Unix epoch (1970-01-01 00:00 UTC) is JD 2440587.5.
	// The day count (~1e4) times the per-day rates requires double precision to keep the
	// fractional degree after the mod-360 reduction; do all of that reduction in double.
	const double unix_days = (double)utc_usec * 1e-6 / 86400.0;
	const double n = (unix_days + 2440587.5) - 2451545.0;

	const double L_deg = wrap360(280.460 + 0.9856474 * n);          // mean longitude [deg]
	const double g_deg = wrap360(357.528 + 0.9856003 * n);          // mean anomaly [deg]
	const double gmst_deg = wrap360(280.46061837 + 360.98564736629 * n); // sidereal time [deg]
	const double eps_deg = 23.439 - 0.0000004 * n;                  // obliquity [deg], ~23.4

	// All remaining angles are O(1)/bounded, so use single-precision trig. fmu-v5/v6x have a
	// single-precision FPU only; this avoids pulling in the large soft-double libm routines
	// (sin/cos/atan2/asin) that would otherwise overflow the fmu-v5 flash, and is faster.
	const float g = (float)(g_deg * DEG2RAD_D);

	// Apparent ecliptic longitude [rad], corrected for the equation of the center.
	const float lambda_deg = (float)L_deg + 1.915f * sinf(g) + 0.020f * sinf(2.0f * g);
	const float lambda = lambda_deg * DEG2RAD_F;
	const float epsilon = (float)eps_deg * DEG2RAD_F;

	// Right ascension and declination.
	const float alpha = atan2f(cosf(epsilon) * sinf(lambda), cosf(lambda)); // [rad]
	const float delta = asinf(constrainf(sinf(epsilon) * sinf(lambda), -1.f, 1.f)); // [rad]

	// Local hour angle [rad], positive toward the West.
	const float lst_deg = (float)wrap360(gmst_deg + lon_deg);
	const float H = (lst_deg - alpha * RAD2DEG_F) * DEG2RAD_F;

	const float phi = (float)lat_deg * DEG2RAD_F;

	// Elevation above the horizon.
	const float sin_elev = constrainf(sinf(phi) * sinf(delta) + cosf(phi) * cosf(delta) * cosf(H),
					  -1.f, 1.f);
	elevation_rad = asinf(sin_elev);

	// Azimuth measured westward from South (Meeus convention)...
	const float az_from_south = atan2f(sinf(H), cosf(H) * sinf(phi) - tanf(delta) * cosf(phi));

	// ...converted to clockwise-from-North in [0, 2*PI).
	float az_from_north = fmodf(az_from_south + PI_F, TWO_PI_F);

	if (az_from_north < 0.f) {
		az_from_north += TWO_PI_F;
	}

	azimuth_rad = az_from_north;
}

} // namespace solar
