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

#include "ActuatorEffectivenessFixedWingDiffThrust.hpp"

using namespace matrix;

ActuatorEffectivenessFixedWingDiffThrust::ActuatorEffectivenessFixedWingDiffThrust(ModuleParams *parent)
	: ModuleParams(parent), _rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedForward),
	  _control_surfaces(this)
{
}

bool ActuatorEffectivenessFixedWingDiffThrust::isDiffThrustYawEnabled()
{
	manual_control_switches_s switches;

	if (_manual_control_switches_sub.update(&switches)) {
		_diff_thrust_yaw_enabled = (switches.gear_switch == manual_control_switches_s::SWITCH_POS_ON);
	}

	return _diff_thrust_yaw_enabled;
}

bool
ActuatorEffectivenessFixedWingDiffThrust::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	// Check for switch state change even on NO_EXTERNAL_UPDATE
	bool prev_enabled = _diff_thrust_yaw_enabled;
	bool now_enabled = isDiffThrustYawEnabled();
	bool switch_changed = (prev_enabled != now_enabled);

	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE && !switch_changed) {
		return false;
	}

	// Motors
	_rotors.enablePropellerTorque(false);
	_rotors.enableYawByDifferentialThrust(now_enabled);
	int rotor_start_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
	const bool rotors_added_successfully = _rotors.addActuators(configuration);
	_forwards_motors_mask = _rotors.getForwardsMotors();

	// Scale differential thrust yaw authority
	const float dthr_scale = _param_ca_dthr_sc.get();
	int rotor_end_idx = configuration.num_actuators_matrix[configuration.selected_matrix];

	for (int i = rotor_start_idx; i < rotor_end_idx; i++) {
		configuration.effectiveness_matrices[configuration.selected_matrix](2, i) *= dthr_scale;
	}

	// Control Surfaces
	_first_control_surface_idx = configuration.num_actuators_matrix[0];
	const bool surfaces_added_successfully = _control_surfaces.addActuators(configuration);

	return (rotors_added_successfully && surfaces_added_successfully);
}

void ActuatorEffectivenessFixedWingDiffThrust::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	stopMaskedMotorsWithZeroThrust(_forwards_motors_mask, actuator_sp);
}

void ActuatorEffectivenessFixedWingDiffThrust::allocateAuxilaryControls(const float dt, int matrix_index,
		ActuatorVector &actuator_sp)
{
	// apply flaps
	normalized_unsigned_setpoint_s flaps_setpoint;

	if (_flaps_setpoint_sub.copy(&flaps_setpoint)) {
		_control_surfaces.applyFlaps(flaps_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
	}

	// apply spoilers
	normalized_unsigned_setpoint_s spoilers_setpoint;

	if (_spoilers_setpoint_sub.copy(&spoilers_setpoint)) {
		_control_surfaces.applySpoilers(spoilers_setpoint.normalized_setpoint, _first_control_surface_idx, dt, actuator_sp);
	}
}
