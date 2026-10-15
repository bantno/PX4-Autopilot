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

#include "WaterHydrodynamics.hpp"

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <sdf/Box.hh>
#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>

using namespace water_hydrodynamics;

namespace
{
constexpr double kWaterDensity = 1000.0;        // kg/m^3
constexpr double kWaterKinViscosity = 1.0e-6;   // m^2/s

struct Box {
	gz::math::Pose3d pose; // link frame
	gz::math::Vector3d size;
};

bool inside(const Box &box, const gz::math::Vector3d &p_link)
{
	const gz::math::Vector3d p = box.pose.Rot().RotateVectorReverse(p_link - box.pose.Pos());
	return std::fabs(p.X()) < 0.5 * box.size.X() && std::fabs(p.Y()) < 0.5 * box.size.Y()
	       && std::fabs(p.Z()) < 0.5 * box.size.Z();
}

// ITTC 1957 friction line, with Rn floored as in asv_wave_sim (the formula has a pole at Rn = 100)
double frictionCoefficient(double Rn)
{
	const double d = std::log10(std::max(1.0e3, Rn)) - 2.0;
	return 0.075 / (d * d);
}
} // namespace

void WaterHydrodynamics::Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
				   gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &)
{
	_model = gz::sim::Model(entity);

	if (!_model.Valid(ecm)) {
		gzerr << "WaterHydrodynamics must be attached to a model" << std::endl;
		return;
	}

	// panel file: model://<model>/<file> resolved on GZ_SIM_RESOURCE_PATH (PX4 spawns models from an SDF string,
	// so there is no model file path to resolve against), or an absolute path
	const std::string panels_file = sdf->Get<std::string>("panels_file", "").first;
	const std::string scheme = "model://";

	if (panels_file.rfind(scheme, 0) == 0) {
		const char *env = std::getenv("GZ_SIM_RESOURCE_PATH");
		std::stringstream dirs(env ? env : "");
		std::string dir;

		while (std::getline(dirs, dir, ':')) {
			const std::string candidate = dir + "/" + panels_file.substr(scheme.size());

			if (!dir.empty() && std::ifstream(candidate).good()) {
				_panels_path = candidate;
				break;
			}
		}

		if (_panels_path.empty()) {
			gzerr << "WaterHydrodynamics: " << panels_file << " not found on GZ_SIM_RESOURCE_PATH" << std::endl;
		}

	} else {
		_panels_path = panels_file;
	}

	_water_level = sdf->Get<double>("water_level", _water_level).first;
	_hydrostatics = sdf->Get<bool>("hydrostatics", _hydrostatics).first;
	_waterline_band = sdf->Get<double>("waterline_band", _waterline_band).first;
	_max_force = sdf->Get<double>("max_force", _max_force).first;
	_max_torque = sdf->Get<double>("max_torque", _max_torque).first;
	_panel_size = sdf->Get<double>("panel_size", _panel_size).first;
	_hull_length = sdf->Get<double>("hull_length", _hull_length).first;
	_c_pdrag1 = sdf->Get<double>("cPDrag1", _c_pdrag1).first;
	_c_pdrag2 = sdf->Get<double>("cPDrag2", _c_pdrag2).first;
	_f_pdrag = sdf->Get<double>("fPDrag", _f_pdrag).first;
	_c_sdrag1 = sdf->Get<double>("cSDrag1", _c_sdrag1).first;
	_c_sdrag2 = sdf->Get<double>("cSDrag2", _c_sdrag2).first;
	_f_sdrag = sdf->Get<double>("fSDrag", _f_sdrag).first;
	_v_rdrag = sdf->Get<double>("vRDrag", _v_rdrag).first;
	_c_damp_l1 = sdf->Get<double>("cDampL1", _c_damp_l1).first;
	_c_damp_l2 = sdf->Get<double>("cDampL2", _c_damp_l2).first;
	_c_damp_r1 = sdf->Get<double>("cDampR1", _c_damp_r1).first;
	_c_damp_r2 = sdf->Get<double>("cDampR2", _c_damp_r2).first;
}

void WaterHydrodynamics::readInertia(gz::sim::EntityComponentManager &ecm)
{
	for (auto &lp : _links) {
		const auto *inertial = ecm.Component<gz::sim::components::Inertial>(lp.link.Entity());

		if (inertial) {
			lp.mass = inertial->Data().MassMatrix().Mass();
			lp.moi = inertial->Data().Moi();
		}
	}
}

bool WaterHydrodynamics::loadPanels(gz::sim::EntityComponentManager &ecm)
{
	std::ifstream in(_panels_path);

	if (!in) {
		gzerr << "WaterHydrodynamics: cannot open " << _panels_path << ", using box-collision panels" << std::endl;
		return false;
	}

	// links by name, with their pose relative to the model (panels are given in the model frame)
	std::map<std::string, size_t> index;

	for (const auto &link_entity : _model.Links(ecm)) {
		gz::sim::Link link(link_entity);
		index[link.Name(ecm).value_or("")] = _links.size();
		_links.push_back(LinkPanels{link, {}, 0.0});
	}

	std::vector<gz::math::Pose3d> link_pose(_links.size());

	for (size_t i = 0; i < _links.size(); i++) {
		const auto *pose = ecm.Component<gz::sim::components::Pose>(_links[i].link.Entity());
		link_pose[i] = pose ? pose->Data() : gz::math::Pose3d::Zero;
	}

	std::string line;
	size_t n = 0;

	while (std::getline(in, line)) {
		if (line.empty() || line[0] == '#') {
			continue;
		}

		std::istringstream ss(line);
		std::string name;
		gz::math::Vector3d p, nrm;
		double area;

		if (!(ss >> name >> p.X() >> p.Y() >> p.Z() >> nrm.X() >> nrm.Y() >> nrm.Z() >> area) || !index.count(name)) {
			continue;
		}

		if (area <= 0.0 || nrm.Length() < 1.0e-6) {
			continue;
		}

		nrm.Normalize(); // stored rounded; |n_z| > 1 would make the waterline extent NaN
		const size_t i = index[name];
		const gz::math::Pose3d &lp = link_pose[i];
		_links[i].panels.push_back(Panel{lp.Rot().RotateVectorReverse(p - lp.Pos()), lp.Rot().RotateVectorReverse(nrm),
						 area});
		_links[i].total_area += area;
		n++;
	}

	// keep only links that have panels
	_links.erase(std::remove_if(_links.begin(), _links.end(), [](const LinkPanels & l) { return l.panels.empty(); }),
	_links.end());

	for (auto &lp : _links) {
		lp.link.EnableVelocityChecks(ecm, true);
	}

	gzmsg << "WaterHydrodynamics: " << n << " hull panels from " << _panels_path << " on " << _links.size()
	      << " links" << std::endl;
	_initialized = true;
	return true;
}

void WaterHydrodynamics::buildPanels(gz::sim::EntityComponentManager &ecm)
{
	size_t n_panels = 0;

	for (const auto &link_entity : _model.Links(ecm)) {
		gz::sim::Link link(link_entity);
		std::vector<Box> boxes;

		for (const auto &collision_entity : link.Collisions(ecm)) {
			const auto *collision = ecm.Component<gz::sim::components::CollisionElement>(collision_entity);

			if (collision != nullptr && collision->Data().Geom() != nullptr
			    && collision->Data().Geom()->Type() == sdf::GeometryType::BOX) {
				boxes.push_back(Box{collision->Data().RawPose(), collision->Data().Geom()->BoxShape()->Size()});
			}
		}

		if (boxes.empty()) {
			continue;
		}

		LinkPanels lp{link, {}, 0.0};
		const double eps = 1.0e-3;

		for (size_t b = 0; b < boxes.size(); b++) {
			const Box &box = boxes[b];

			// six faces: axis a, side s; the face spans the other two axes u, v
			for (int a = 0; a < 3; a++) {
				const int u = (a + 1) % 3;
				const int v = (a + 2) % 3;
				const int nu = std::max(1, (int)std::ceil(box.size[u] / _panel_size));
				const int nv = std::max(1, (int)std::ceil(box.size[v] / _panel_size));
				const double du = box.size[u] / nu;
				const double dv = box.size[v] / nv;

				for (int s = -1; s <= 1; s += 2) {
					gz::math::Vector3d n_box(0, 0, 0);
					n_box[a] = s;
					const gz::math::Vector3d n_link = box.pose.Rot().RotateVector(n_box);

					for (int i = 0; i < nu; i++) {
						for (int j = 0; j < nv; j++) {
							gz::math::Vector3d c_box(0, 0, 0);
							c_box[a] = 0.5 * s * box.size[a];
							c_box[u] = -0.5 * box.size[u] + (i + 0.5) * du;
							c_box[v] = -0.5 * box.size[v] + (j + 0.5) * dv;
							const gz::math::Vector3d c_link = box.pose.Pos() + box.pose.Rot().RotateVector(c_box);

							// exterior only: the point just outside must not be inside another box
							const gz::math::Vector3d probe = c_link + eps * n_link;
							bool covered = false;

							for (size_t k = 0; k < boxes.size() && !covered; k++) {
								covered = (k != b) && inside(boxes[k], probe);
							}

							if (!covered) {
								lp.panels.push_back(Panel{c_link, n_link, du * dv});
								lp.total_area += du * dv;
							}
						}
					}
				}
			}
		}

		link.EnableVelocityChecks(ecm, true);
		n_panels += lp.panels.size();
		_links.push_back(lp);
	}

	gzmsg << "WaterHydrodynamics: " << n_panels << " hull panels on " << _links.size() << " links" << std::endl;
	_initialized = true;
}

void WaterHydrodynamics::PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm)
{
	if (info.paused || !_model.Valid(ecm)) {
		return;
	}

	// Collisions of a spawned model are created after Configure; build the panels on the first step
	if (!_initialized) {
		if (_panels_path.empty() || !loadPanels(ecm)) {
			_links.clear();
			buildPanels(ecm);
		}

		readInertia(ecm);

		const auto gravity = gz::sim::World(gz::sim::worldEntity(ecm)).Gravity(ecm);

		if (gravity) {
			_gravity = gravity->Length();
		}

		if (_hydrostatics) {
			gzmsg << "WaterHydrodynamics: hydrostatic buoyancy from the panels (g = " << _gravity << ")" << std::endl;
		}

		return;
	}

	for (auto &lp : _links) {
		const auto pose = lp.link.WorldPose(ecm);
		const auto v_origin = lp.link.WorldLinearVelocity(ecm);
		const auto omega = lp.link.WorldAngularVelocity(ecm);

		if (!pose || !v_origin || !omega) {
			continue;
		}

		const double speed = v_origin->Length();
		const double c_f = frictionCoefficient(speed * _hull_length / kWaterKinViscosity);

		gz::math::Vector3d force(0, 0, 0);
		gz::math::Vector3d torque(0, 0, 0);
		gz::math::Vector3d hs_force(0, 0, 0);
		gz::math::Vector3d hs_torque(0, 0, 0);
		double submerged_area = 0.0;

		for (const auto &panel : lp.panels) {
			const gz::math::Vector3d r = pose->Rot().RotateVector(panel.position); // from the link origin, world axes
			const gz::math::Vector3d n = pose->Rot().RotateVector(panel.normal);

			// submerged fraction over the panel's vertical extent (with a minimum band for flat panels), so forces
			// fade in and out smoothly at the waterline instead of switching with the panel centre
			const double half_h = 0.5 * std::max(std::sqrt(panel.area * std::max(0.0, 1.0 - n.Z() * n.Z())), _waterline_band);
			const double fraction = gz::math::clamp((_water_level - (pose->Pos().Z() + r.Z()) + half_h) / (2.0 * half_h),
								0.0, 1.0);

			if (fraction <= 0.0) {
				continue;
			}

			const double area = fraction * panel.area;

			// hydrostatic pressure at the panel centre, against the outward normal (zero at the waterline)
			const double depth = _water_level - (pose->Pos().Z() + r.Z());

			if (_hydrostatics && depth > 0.0) {
				const gz::math::Vector3d f_hs = -(kWaterDensity * _gravity * depth * panel.area) * n;
				hs_force += f_hs;
				hs_torque += r.Cross(f_hs);
			}

			const gz::math::Vector3d vp = *v_origin + omega->Cross(r);
			const double vp_len = vp.Length();
			submerged_area += area;

			if (vp_len < 1.0e-6) {
				continue;
			}

			const double cos_theta = vp.Dot(n) / vp_len;
			const double v = vp_len / _v_rdrag;
			gz::math::Vector3d f(0, 0, 0);

			// pressure drag along the normal: positive pressure on panels facing the flow, suction on the lee side
			if (cos_theta >= 0.0) {
				f -= (_c_pdrag1 * v + _c_pdrag2 * v * v) * area * std::pow(cos_theta, _f_pdrag) * n;

			} else {
				f += (_c_sdrag1 * v + _c_sdrag2 * v * v) * area * std::pow(-cos_theta, _f_sdrag) * n;
			}

			// viscous drag opposing the tangential flow
			const gz::math::Vector3d vt = vp - vp.Dot(n) * n;
			const double vt_len = vt.Length();

			if (vt_len > 1.0e-6) {
				f -= 0.5 * kWaterDensity * c_f * area * vp_len * (vt / vt_len) * vp_len;
			}

			force += f;
			torque += r.Cross(f);
		}

		if (submerged_area <= 0.0) {
			continue;
		}

		// small whole-link damping scaled by the submerged-area fraction (numerical settling)
		const double rs = submerged_area / lp.total_area;
		force -= rs * (_c_damp_l1 + _c_damp_l2 * speed) * (*v_origin);
		torque -= rs * (_c_damp_r1 + _c_damp_r2 * omega->Length()) * (*omega);

		// Explicit-damping limiter: the drag may at most stop the link's motion in this step, never reverse it
		const double dt = std::chrono::duration<double>(info.dt).count();

		if (dt > 0.0 && lp.mass > 0.0 && speed > 1.0e-6) {
			const gz::math::Vector3d u = *v_origin / speed;
			const double along = force.Dot(u);
			const double limit = lp.mass * speed / dt;

			if (along < -limit) {
				force -= (along + limit) * u;
			}
		}

		const double w = omega->Length();

		if (dt > 0.0 && w > 1.0e-6) {
			const gz::math::Vector3d u = *omega / w;
			const gz::math::Vector3d u_link = pose->Rot().RotateVectorReverse(u);
			const double inertia = u_link.Dot(lp.moi * u_link);
			const double along = torque.Dot(u);
			const double limit = inertia * w / dt;

			if (inertia > 0.0 && along < -limit) {
				torque -= (along + limit) * u;
			}
		}

		// Safety limit for violent water impacts: the v^2 panel forces can grow faster than a 4 ms explicit step
		// can follow and diverge the physics. Only binds far beyond normal floating/planing loads.
		if (force.Length() > _max_force) {
			force *= _max_force / force.Length();
		}

		if (torque.Length() > _max_torque) {
			torque *= _max_torque / torque.Length();
		}

		// applied at the link origin; the torques above are taken about that point. Buoyancy is a conservative
		// force: never clipped by the drag limiter or the impact limits.
		lp.link.AddWorldWrench(ecm, force + hs_force, torque + hs_torque);
	}
}

GZ_ADD_PLUGIN(WaterHydrodynamics, gz::sim::System, WaterHydrodynamics::ISystemConfigure,
	      WaterHydrodynamics::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(WaterHydrodynamics, "water_hydrodynamics::WaterHydrodynamics")
