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

#include "WaterDamping.hpp"

#include <gz/math/Helpers.hh>
#include <gz/math/Matrix3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Collision.hh>
#include <sdf/Box.hh>
#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>

#include <cmath>

using namespace water_damping;

void WaterDamping::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
			     gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &)
{
	_model = gz::sim::Model(entity);

	if (!_model.Valid(ecm)) {
		gzerr << "WaterDamping must be attached to a model" << std::endl;
		return;
	}

	_water_level = sdf->Get<double>("water_level", _water_level).first;
	_linear = sdf->Get<gz::math::Vector3d>("linear", _linear).first;
	_quadratic = sdf->Get<gz::math::Vector3d>("quadratic", _quadratic).first;
	_angular = sdf->Get<double>("angular", _angular).first;
	_angular_quadratic = sdf->Get<double>("angular_quadratic", _angular_quadratic).first;
}

void WaterDamping::collectBoxes(gz::sim::EntityComponentManager &ecm)
{
	for (const auto &link_entity : _model.Links(ecm)) {
		gz::sim::Link link(link_entity);
		LinkBoxes lb{link, {}};

		for (const auto &collision_entity : link.Collisions(ecm)) {
			const auto *collision = ecm.Component<gz::sim::components::CollisionElement>(collision_entity);

			if (collision == nullptr || collision->Data().Geom() == nullptr
			    || collision->Data().Geom()->Type() != sdf::GeometryType::BOX) {
				continue;
			}

			const gz::math::Vector3d size = collision->Data().Geom()->BoxShape()->Size();
			lb.boxes.push_back(Box{collision->Data().RawPose(), size, size.X() * size.Y() * size.Z()});
		}

		if (!lb.boxes.empty()) {
			link.EnableVelocityChecks(ecm, true);
			_links.push_back(lb);
		}
	}

	_initialized = true;
}

void WaterDamping::PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm)
{
	if (info.paused || !_model.Valid(ecm)) {
		return;
	}

	// Collisions of a spawned model are created after Configure; collect them on the first step
	if (!_initialized) {
		collectBoxes(ecm);
		return;
	}

	for (auto &lb : _links) {
		const auto link_pose = lb.link.WorldPose(ecm);
		const auto omega = lb.link.WorldAngularVelocity(ecm);

		if (!link_pose || !omega) {
			continue;
		}

		gz::math::Vector3d force{0, 0, 0};
		gz::math::Vector3d torque{0, 0, 0};
		double submerged = 0.0;

		for (const auto &box : lb.boxes) {
			const gz::math::Pose3d box_world = *link_pose * box.pose;
			const gz::math::Matrix3d R(box_world.Rot());

			// half the box's vertical extent in the world frame
			const double hz = 0.5 * (std::fabs(R(2, 0)) * box.size.X() + std::fabs(R(2, 1)) * box.size.Y()
						 + std::fabs(R(2, 2)) * box.size.Z());
			const double bottom = box_world.Pos().Z() - hz;
			const double fraction = gz::math::clamp((_water_level - bottom) / (2.0 * hz), 0.0, 1.0);

			if (fraction <= 0.0) {
				continue;
			}

			const double v_sub = fraction * box.volume;
			const auto vel = lb.link.WorldLinearVelocity(ecm, box.pose.Pos());

			if (!vel) {
				continue;
			}

			// drag in the link frame (anisotropic), back to world
			const gz::math::Vector3d v_link = link_pose->Rot().RotateVectorReverse(*vel);
			const gz::math::Vector3d f_link(
				-v_sub * (_linear.X() + _quadratic.X() * std::fabs(v_link.X())) * v_link.X(),
				-v_sub * (_linear.Y() + _quadratic.Y() * std::fabs(v_link.Y())) * v_link.Y(),
				-v_sub * (_linear.Z() + _quadratic.Z() * std::fabs(v_link.Z())) * v_link.Z());
			const gz::math::Vector3d f_world = link_pose->Rot().RotateVector(f_link);

			force += f_world;
			torque += (box_world.Pos() - link_pose->Pos()).Cross(f_world);
			submerged += v_sub;
		}

		if (submerged <= 0.0) {
			continue;
		}

		torque -= submerged * (_angular + _angular_quadratic * omega->Length()) * (*omega);

		// applied at the link origin; the torque above is taken about that point
		lb.link.AddWorldWrench(ecm, force, torque);
	}
}

GZ_ADD_PLUGIN(WaterDamping, gz::sim::System, WaterDamping::ISystemConfigure, WaterDamping::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(WaterDamping, "water_damping::WaterDamping")
