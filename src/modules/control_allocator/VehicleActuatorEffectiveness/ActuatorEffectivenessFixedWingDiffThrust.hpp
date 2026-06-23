/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#pragma once

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessControlSurfaces.hpp"

#include <uORB/topics/normalized_unsigned_setpoint.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/topics/vehicle_land_detected.h>

/**
 * Fixed-wing with differential thrust yaw gated on the airborne state.
 *
 * Identical to ActuatorEffectivenessFixedWing, but gates yaw authority from
 * differential thrust on whether the vehicle is on the ground:
 *   - On the ground (landed): yaw-by-differential-thrust is enabled by default,
 *     to help keep the takeoff/taxi ground roll straight.
 *   - Airborne: differential thrust is no longer used as a yaw source, unless
 *     the pilot explicitly re-enables it with the gear switch (RC_MAP_GEAR_SW).
 *
 * In short: enabled = landed || gear_switch_on.
 */
class ActuatorEffectivenessFixedWingDiffThrust : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessFixedWingDiffThrust(ModuleParams *parent);
	virtual ~ActuatorEffectivenessFixedWingDiffThrust() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	const char *name() const override { return "Fixed Wing (Diff Thrust)"; }

	void allocateAuxilaryControls(const float dt, int matrix_index, ActuatorVector &actuator_sp) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
			    const matrix::Vector<float, NUM_ACTUATORS> &actuator_max) override;

private:
	bool isDiffThrustYawEnabled();

	ActuatorEffectivenessRotors _rotors;
	ActuatorEffectivenessControlSurfaces _control_surfaces;

	uORB::Subscription _flaps_setpoint_sub{ORB_ID(flaps_setpoint)};
	uORB::Subscription _spoilers_setpoint_sub{ORB_ID(spoilers_setpoint)};
	uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	int _first_control_surface_idx{0};

	uint32_t _forwards_motors_mask{};

	bool _gear_switch_on{false};
	bool _landed{true}; // assume on the ground until the land detector says otherwise
	bool _diff_thrust_yaw_enabled{true};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CA_DTHR_SC>) _param_ca_dthr_sc
	)
};
