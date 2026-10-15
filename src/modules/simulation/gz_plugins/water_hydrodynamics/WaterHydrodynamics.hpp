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
 * @file WaterHydrodynamics.hpp
 *
 * Per-panel hull hydrodynamics on a flat water surface (Kerner, "Water interaction model for boats in video games",
 * Just Cause 3, 2015; same force set as the asv_wave_sim HydrodynamicsPlugin, Mainwaring). Buoyancy is left to
 * gz-sim's graded Buoyancy system on the same collisions, unless <hydrostatics> is true: then each submerged panel
 * also gets the hydrostatic pressure F = -rho g depth S n (at its centre, depth below <water_level>), which over a
 * closed panel surface integrates to the buoyant force and moment (divergence theorem) without any buoyancy
 * collisions. That is much cheaper in gz than hundreds of buoyancy boxes (each collision shape costs physics time
 * every step). The hydrostatic part is not subject to the drag limiter or the impact limits below.
 *
 * The hull surface comes from <panels_file> (next to model.sdf): one panel per line, "link x y z nx ny nz area" in the
 * model frame, generated from the real hull surface (true normals, so a raked bow gives lift). Without it, panels
 * are built from the model's box collisions: each box face is cut into panels of at most <panel_size> and kept only
 * if the point just outside it is not inside another box of the same link. Every step, each panel whose centre is
 * below <water_level> gets, at its centre (v = panel velocity, n = outward normal, S = area, cos t = v_hat . n):
 *
 *   pressure drag  cos t >= 0:  F = -(cPDrag1 (v/vRDrag) + cPDrag2 (v/vRDrag)^2) S cos(t)^fPDrag n
 *   suction        cos t <  0:  F = +(cSDrag1 (v/vRDrag) + cSDrag2 (v/vRDrag)^2) S |cos t|^fSDrag n
 *   viscous        F = 1/2 rho C_F(Rn) S |v| u_f,   u_f = -tangential direction, C_F = 0.075 / (log10 Rn - 2)^2
 *
 * plus a small damping on each link scaled by its submerged-area fraction r_s:
 *   F = -r_s (cDampL1 + cDampL2 |v|) v,   T = -r_s (cDampR1 + cDampR2 |w|) w
 *
 * Each panel's area is scaled by its submerged fraction over its vertical extent (at least <waterline_band>), so the
 * forces fade in and out at the waterline rather than switching with the panel centre (which drove porpoising at
 * planing speed). Pressure drag on panels facing the flow is what produces planing lift at speed; nothing is applied
 * out of the water. The forces use the previous step's velocity (explicit), so on a light link in strong damping
 * (the wing sweeping through the water when inverted) they can overshoot and reverse the motion every step, which
 * shows as jitter: the drag on each link is therefore limited to what stops its motion in one step (m |v| / dt along
 * v, I |w| / dt about w). The per-link total is also limited to <max_force> / <max_torque> so a violent impact cannot
 * diverge the physics step.
 */

#pragma once

#include <gz/math/Matrix3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <string>
#include <vector>

namespace water_hydrodynamics
{

class WaterHydrodynamics:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
		       gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &event_mgr) override;

	void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override;

private:
	struct Panel {
		gz::math::Vector3d position; // centre, link frame
		gz::math::Vector3d normal;   // outward unit normal, link frame
		double area;
	};

	struct LinkPanels {
		gz::sim::Link link;
		std::vector<Panel> panels;
		double total_area;
		double mass{0.0};
		gz::math::Matrix3d moi{gz::math::Matrix3d::Zero}; // about the CoM, link frame
	};

	void readInertia(gz::sim::EntityComponentManager &ecm);

	void buildPanels(gz::sim::EntityComponentManager &ecm);
	bool loadPanels(gz::sim::EntityComponentManager &ecm);

	gz::sim::Model _model{gz::sim::kNullEntity};
	std::vector<LinkPanels> _links;
	bool _initialized{false};
	bool _hydrostatics{false};
	double _gravity{9.8}; // m/s^2, magnitude, from the world

	std::string _panels_path;
	double _water_level{0.0};
	double _waterline_band{0.010}; // m, minimum vertical extent for the submerged-fraction fade
	double _max_force{500.0};      // N per link, impact safety limit
	double _max_torque{100.0};     // N m per link
	double _panel_size{0.025};
	double _hull_length{1.1};   // m, Reynolds length

	double _c_pdrag1{100.0};
	double _c_pdrag2{100.0};
	double _f_pdrag{1.0};
	double _c_sdrag1{100.0};
	double _c_sdrag2{100.0};
	double _f_sdrag{1.0};
	double _v_rdrag{1.0};

	double _c_damp_l1{1.0e-6};
	double _c_damp_l2{1.0e-6};
	double _c_damp_r1{1.0e-6};
	double _c_damp_r2{1.0e-6};
};

} // namespace water_hydrodynamics
