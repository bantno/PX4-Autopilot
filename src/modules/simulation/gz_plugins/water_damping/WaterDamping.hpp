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
 * @file WaterDamping.hpp
 *
 * Water drag that acts only on the submerged part of a model (gz-sim's Hydrodynamics system acts regardless of
 * submersion, which would also drag a seaplane in flight). For every box collision of every link, the submerged
 * fraction below <water_level> is estimated from the box's world-frame vertical extent, and that box gets a
 * drag force at its centre, per submerged volume and anisotropic in the link frame:
 *
 *   F_i = -V_sub * (linear_i + quadratic_i * |v_i|) * v_i     (i = link x, y, z; v at the box centre)
 *
 * plus a rotational damping torque on each link, -(V_sub,link) * (angular + angular_quadratic * |w|) * w.
 * Out of the water every V_sub is zero and nothing is applied.
 *
 * SDF (model plugin):
 *   <water_level>0</water_level>                     m
 *   <linear>200 2000 10000</linear>                  N s / m^4   (per m^3 submerged, link x y z)
 *   <quadratic>300 3000 15000</quadratic>            N s^2 / m^5
 *   <angular>120</angular>                           N m s / m^3
 *   <angular_quadratic>0</angular_quadratic>         N m s^2 / m^3
 */

#pragma once

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <vector>

namespace water_damping
{

class WaterDamping:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &event_mgr) override;

	void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override;

private:
	struct Box {
		gz::math::Pose3d pose;   // in the link frame
		gz::math::Vector3d size;
		double volume;
	};

	struct LinkBoxes {
		gz::sim::Link link;
		std::vector<Box> boxes;
	};

	void collectBoxes(gz::sim::EntityComponentManager &ecm);

	gz::sim::Model _model{gz::sim::kNullEntity};
	std::vector<LinkBoxes> _links;
	bool _initialized{false};

	double _water_level{0.0};
	gz::math::Vector3d _linear{200.0, 2000.0, 10000.0};
	gz::math::Vector3d _quadratic{300.0, 3000.0, 15000.0};
	double _angular{120.0};
	double _angular_quadratic{0.0};
};

} // namespace water_damping
