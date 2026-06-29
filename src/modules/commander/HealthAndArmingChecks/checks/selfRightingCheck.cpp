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

#include "selfRightingCheck.hpp"

#include <parameters/param.h>
#include <matrix/math.hpp>

void SelfRightingChecks::checkAndReport(const Context &context, Report &reporter)
{
	// SR_EN / SR_INV_THR live in the self_right module; look them up dynamically so commander
	// builds on boards without that module. If self_right is absent, the mode stays blocked.
	float inv_thr = 0.7f;
	bool enabled = false;
	{
		const param_t en_h = param_find_no_notification("SR_EN");
		const param_t thr_h = param_find_no_notification("SR_INV_THR");

		if (en_h != PARAM_INVALID) {
			int32_t en = 0;
			param_get(en_h, &en);
			enabled = en != 0;
		}

		if (thr_h != PARAM_INVALID) {
			param_get(thr_h, &inv_thr);
		}
	}

	// "Inverted": the body +Z axis points up in the world (NED down component negative).
	bool inverted = false;
	vehicle_attitude_s att;

	if (_vehicle_attitude_sub.copy(&att)) {
		const matrix::Vector3f body_z_in_ned = matrix::Quatf(att.q).dcm_z();
		inverted = body_z_in_ned(2) < -inv_thr;
	}

	// "At rest": land detector at_rest plus a converged tilt estimate.
	bool at_rest = false;
	vehicle_land_detected_s land_detected;

	if (_vehicle_land_detected_sub.copy(&land_detected)) {
		at_rest = land_detected.at_rest;
	}

	estimator_status_flags_s estimator_flags;

	if (_estimator_status_flags_sub.copy(&estimator_flags)) {
		at_rest = at_rest && estimator_flags.cs_tilt_align;

	} else {
		at_rest = false;
	}

	const bool entry_ok = enabled && inverted && at_rest;
	const bool in_mode = context.status().nav_state == vehicle_status_s::NAVIGATION_STATE_SELF_RIGHT;

	// Latch: block switching INTO the mode unless inverted+at_rest, but never eject once active.
	if (!entry_ok && !in_mode) {
		reporter.clearCanRunBits(NavModes::SelfRight);
	}

	// Only allow arming in the mode while the entry condition holds (the meaningful gate when
	// disarmed, where mode selection itself is otherwise unrestricted).
	if (!entry_ok) {
		reporter.clearArmingBits(NavModes::SelfRight);
	}
}
