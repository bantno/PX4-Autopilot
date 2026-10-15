#!/usr/bin/env python3
"""Generate twin_tractor_seaplane/model.sdf (the AHAB Gazebo model) from its sources.

    python3 Tools/simulation/ahab_model/tools/build_model.py

Inputs (nothing in model.sdf is hand-edited; change these and re-run):
- geometry: Nick Trujillo's mesh-measured hinge/motor positions (constants below, from ahab_base.xacro,
  commit ca3852a831), his surface/prop meshes, and hull.glb / wing.glb meshed from the multibody STEP
  by tools/step_to_glb.py
- mass properties: mass_properties.yaml via tools/mass_properties.py (measured; placeholder until then)
- aero: AHAB_combined_betasym.stab via tools/stab_to_gz.py, rate derivatives from rate_derivatives.yaml
- collision/buoyancy boxes: collision_boxes.json via tools/step_to_collision.py. By default buoyancy comes from
  the closed hydro panel surface (WaterHydrodynamics <hydrostatics>, validated against the boxes by
  tools/panel_hydrostatics.py) and only thin ground-contact skids are emitted, because every collision shape costs
  gz physics time (the 310 boxes were ~60 % of the step). AHAB_BOX_BUOYANCY=1 emits the boxes instead (A/B checks).
- propulsion: JSBSim Cine66 925KV + 9x5x3 (Engines/9x5x3.xml): C_T 0.105, C_P 0.050, D 0.2286 m,
  17.3 kRPM full throttle -> 29.2 N, motorConstant 8.9e-6, momentConstant 0.0173

Model frame: x forward, y left, z up (Nick's mesh frame). Joints and PX4 servo outputs are + = trailing
edge up. The tilt joint is + = wing leading edge up (axis -y), so SR_TILT_SP = -1.57 rad points the props
toward the hull's belly, i.e. up while inverted, matching the hardware.
"""

import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import mass_properties  # noqa: E402
import stab_to_gz  # noqa: E402
import yaml  # noqa: E402

MODEL_DIR = os.path.normpath(os.path.join(HERE, '..', 'twin_tractor_seaplane'))

# --- Nick's geometry (ahab_base.xacro), model frame, metres / radians
AILERON = (-0.118, 0.6178, 0.135)
VTAIL = (-0.5955, 0.1508, 0.169)
VTAIL_DIHEDRAL = 0.6775
PROP = (0.092, 0.2373, 0.1408)
SURFACE_LIMIT = 0.52  # ~30 deg

# Spawn height: keel bottom is at z = -0.1415 in the model frame.
MODEL_Z = 0.16

# --- water (WaterHydrodynamics plugin). Kerner per-panel model; parameter names as in asv_wave_sim.
# Start: asv_wave_sim WAM-V coefficients (cPDrag 100/100), with fPDrag = 1 so planing lift ~ trim, matching
# Savitsky's C_L ~ tau^1.1: at 5 m/s, 5 deg trim the hull bottom (~0.15 m^2) gives ~33 N vs Savitsky ~32 N.
# Calibrated in ahab_lake with tools/water_scenarios.py (see README).
HYDRO = dict(panels_file='model://twin_tractor_seaplane/hydro_panels.txt', water_level=0.0, waterline_band=0.010,
             panel_size=0.025, hull_length=1.13,
             cPDrag1=100.0, cPDrag2=100.0, fPDrag=1.0,
             cSDrag1=20.0, cSDrag2=20.0, fSDrag=1.0, vRDrag=1.0,
             cDampL1=1.0, cDampL2=1.0, cDampR1=20.0, cDampR2=2.0,
             max_force=500.0, max_torque=100.0)   # ~10 g on 5 kg: impact safety limit only

# Buoyancy source: panel hydrostatics (default) or the legacy box collisions with the world's Buoyancy system
BOX_BUOYANCY = os.environ.get('AHAB_BOX_BUOYANCY', '') == '1'
SKID_THICKNESS = 0.002  # m; ground-contact skids under keel and floats (their Buoyancy-system volume is < 1 %)

# --- propulsion (see docstring)
MOTOR = dict(max_rot_velocity=1811, motor_constant=8.9e-06, moment_constant=0.0173)


def f(x):
    return x if isinstance(x, str) else '%.6g' % x


def inertial(mass, pose, I):
    return '''      <inertial>
        <pose>%s</pose>
        <mass>%s</mass>
        <inertia>
          <ixx>%s</ixx><ixy>%s</ixy><ixz>%s</ixz>
          <iyy>%s</iyy><iyz>%s</iyz>
          <izz>%s</izz>
        </inertia>
      </inertial>
''' % (' '.join(f(v) for v in pose), f(mass), f(I[0][0]), f(I[0][1]), f(I[0][2]), f(I[1][1]), f(I[1][2]), f(I[2][2]))


def visual(name, mesh, pose):
    return '''      <visual name="%s">
        <pose>%s</pose>
        <geometry><mesh><uri>model://twin_tractor_seaplane/meshes/%s</uri></mesh></geometry>
      </visual>
''' % (name, ' '.join(f(v) for v in pose), mesh)


def box(pose, size):
    # low friction so the hull can slide on the ground plane for takeoff
    return '''      <collision name="%s">
        <pose>%s</pose>
        <geometry><box><size>%s</size></box></geometry>
        <surface><friction><ode><mu>0.2</mu><mu2>0.2</mu2></ode></friction></surface>
      </collision>
''' % (pose[0], ' '.join(f(v) for v in pose[1]), ' '.join(f(v) for v in size))


def skids(boxes, select):
    """One thin box under the lowest layer of the selected boxes (model frame): ground contact without the
    collision count (or buoyancy volume) of the full box set."""
    sel = [b for b in boxes if select(b)]
    zmin = min(b['pos'][2] - b['size'][2] / 2 for b in sel)
    low = [b for b in sel if b['pos'][2] - b['size'][2] / 2 < zmin + 0.005]
    x0 = min(b['pos'][0] - b['size'][0] / 2 for b in low)
    x1 = max(b['pos'][0] + b['size'][0] / 2 for b in low)
    y0 = min(b['pos'][1] - b['size'][1] / 2 for b in low)
    y1 = max(b['pos'][1] + b['size'][1] / 2 for b in low)
    return {'pos': [(x0 + x1) / 2, (y0 + y1) / 2, zmin + SKID_THICKNESS / 2],
            'size': [x1 - x0, y1 - y0, SKID_THICKNESS]}


def child_link(name, pose, mass, ipose, I_diag, vis):
    I = [[I_diag[0], 0, 0], [0, I_diag[1], 0], [0, 0, I_diag[2]]]
    return '''    <link name="%s">
      <pose>%s</pose>
%s%s    </link>
''' % (name, ' '.join(f(v) for v in pose), inertial(mass, ipose, I), vis)


def revolute(name, parent, child, axis, lo, hi, effort=10, damping=0.001):
    return '''    <joint name="%s" type="revolute">
      <parent>%s</parent>
      <child>%s</child>
      <axis>
        <xyz>%s</xyz>
        <limit><lower>%s</lower><upper>%s</upper><effort>%s</effort><velocity>20</velocity></limit>
        <dynamics><damping>%s</damping></dynamics>
      </axis>
    </joint>
''' % (name, parent, child, ' '.join(f(v) for v in axis), f(lo), f(hi), f(effort), f(damping))


def sensors():
    with open(os.path.join(HERE, 'sensors.sdf.in')) as fp:
        return fp.read()


def build():
    # calibration sweeps: AHAB_HYDRO='{"cPDrag1": 30}' overrides HYDRO entries, AHAB_TILT_EFFORT the tilt torque limit
    HYDRO['hydrostatics'] = 'false' if BOX_BUOYANCY else 'true'
    HYDRO.update(json.loads(os.environ.get('AHAB_HYDRO', '{}')))

    with open(mass_properties.DEFAULT_YAML) as fp:
        mp_doc = yaml.safe_load(fp)

    mp, *_ = mass_properties.compute(mp_doc)
    aero = stab_to_gz.fit(stab_to_gz.DEFAULT_STAB, stab_to_gz.DEFAULT_RATES)
    parts = mp['parts']
    pivot = mp['pivot_model']
    hull = mp['hull_link']
    wing = mp['wing_link']
    tilt = mp['tilt_drive'] or {}

    with open(os.path.join(HERE, '..', 'collision_boxes.json')) as fp:
        cboxes = json.load(fp)

    notes = []

    if not all(mp['measured'].values()):
        notes.append('MASS PROPERTIES ARE PLACEHOLDERS (mass_properties.yaml not measured yet)')

    if any('FALLBACK' in s for s in aero['rate_source'].values()):
        notes.append('rate derivatives include VSPAERO fallbacks (rate_derivatives.yaml incomplete)')

    out = []
    out.append('''<?xml version="1.0" ?>
<!--
  AHAB twin-tractor tilt-wing seaplane. GENERATED by Tools/simulation/ahab_model/tools/build_model.py
  - do not edit; change the inputs and re-run.
%s-->
<sdf version="1.9">
  <model name="twin_tractor_seaplane">
    <pose>0 0 %s 0 0 0</pose>
''' % (''.join('  NOTE: %s\n' % n for n in notes), f(MODEL_Z)))

    # --- hull (base_link): hull, V-tail, tilt drive, battery, avionics, sensors
    # hull.glb / wing.glb are written in the model frame by tools/step_to_glb.py (no rotation needed)
    hull_vis = visual('hull_visual', 'hull.glb', (0, 0, 0, 0, 0, 0))
    out.append('    <link name="base_link">\n')
    out.append(inertial(hull['mass'], hull['cg'] + [0, 0, 0], hull['I']))
    out.append(hull_vis)

    # ventral fin (keel rudder) taped to the hull bottom behind the step (tools/fin_to_model.py)
    if os.path.exists(os.path.join(MODEL_DIR, 'meshes', 'fin.glb')):
        out.append(visual('fin_visual', 'fin.glb', (0, 0, 0, 0, 0, 0)))
    # Volume-preserving boxes from the STEP (tools/step_to_collision.py): they set buoyancy on water and
    # ground contact, so the hull's outer envelope and the panel volumes are what matter here.
    hull_boxes = cboxes['hull'] if BOX_BUOYANCY else [skids(cboxes['hull'], lambda b: True)]

    for i, b in enumerate(hull_boxes):
        out.append(box(('hull_%d' % i, tuple(b['pos']) + (0, 0, 0)), b['size']))
    out.append(sensors())
    out.append('    </link>\n')

    # --- wing (tilting): wing, nacelles, motors; origin at the tilt pivot
    out.append('    <link name="wing_link">\n      <pose>%s 0 0 0</pose>\n' % ' '.join(f(v) for v in pivot))
    out.append(inertial(wing['mass'], wing['cg'] + [0, 0, 0], wing['I']))

    out.append(visual('wing_visual', 'wing.glb', (-pivot[0], -pivot[1], -pivot[2], 0, 0, 0)))

    # wingtip floats hang from the wing (tools/float_to_model.py); their buoyancy boxes are in cboxes['wing']
    if os.path.exists(os.path.join(MODEL_DIR, 'meshes', 'floats.glb')):
        out.append(visual('floats_visual', 'floats.glb', (-pivot[0], -pivot[1], -pivot[2], 0, 0, 0)))

    # outer wing panels only (|y| 0.110..0.910 m; the centre section is hull) and the wingtip floats
    wing_boxes = cboxes['wing'] if BOX_BUOYANCY else [
        skids(cboxes['wing'], lambda b, s=s: s * b['pos'][1] > 0.6) for s in (1, -1)]

    for i, b in enumerate(wing_boxes):
        p = b['pos']
        out.append(box(('wing_%d' % i, (p[0] - pivot[0], p[1] - pivot[1], p[2] - pivot[2], 0, 0, 0)), b['size']))
    out.append('    </link>\n')

    effort = float(os.environ.get('AHAB_TILT_EFFORT', 0)) or tilt.get('stall_torque_Nm') or 10.0
    out.append(revolute('wing_tilt_joint', 'base_link', 'wing_link', (0, -1, 0), -math.pi, math.pi,
                        effort=effort, damping=0.01))

    # --- ailerons on the wing (servo_0 left, servo_1 right)
    a = parts['aileron']

    for i, (side, s) in enumerate((('left', 1), ('right', -1))):
        name = 'aileron_' + side
        out.append(child_link(name, (AILERON[0], s * AILERON[1], AILERON[2], 0, 0, 0), a['mass'], (-0.025, 0, 0, 0, 0, 0),
                              [a['mass'] * 0.325**2 / 12, a['mass'] * 0.055**2 / 12, a['mass'] * (0.325**2 + 0.055**2) / 12],
                              visual(name + '_visual', 'aileron.glb', (0, 0, 0, 0, 0, math.pi))))
        out.append(revolute('servo_%d' % i, 'wing_link', name, (0, 1, 0), -SURFACE_LIMIT, SURFACE_LIMIT))

    # --- ruddervators on the hull (servo_2 left, servo_3 right)
    r = parts['ruddervator']

    for i, (side, s) in enumerate((('left', 1), ('right', -1))):
        name = 'ruddervator_' + side
        roll = VTAIL_DIHEDRAL if s > 0 else math.pi - VTAIL_DIHEDRAL
        out.append(child_link(name, (VTAIL[0], s * VTAIL[1], VTAIL[2], roll, 0, 0), r['mass'], (-0.025, 0, 0, 0, 0, 0),
                              [r['mass'] * 0.255**2 / 12, r['mass'] * 0.051**2 / 12, r['mass'] * (0.255**2 + 0.051**2) / 12],
                              visual(name + '_visual', 'ruddervator.glb', (0, 0, 0, 0, math.pi / 2, 0))))
        out.append(revolute('servo_%d' % (i + 2), 'base_link', name, (0, s, 0), -SURFACE_LIMIT, SURFACE_LIMIT))

    # --- props on the wing (rotor_0 left = motor 1, rotor_1 right = motor 2); link z = thrust axis
    p = parts['prop']

    for i, s in enumerate((1, -1)):
        name = 'rotor_%d' % i
        out.append(child_link(name, (PROP[0], s * PROP[1], PROP[2], 0, math.pi / 2, 0), p['mass'], (0, 0, 0, 0, 0, 0),
                              [p['mass'] * 0.127**2 / 6, p['mass'] * 0.127**2 / 6, p['mass'] * 0.127**2 / 3],
                              visual(name + '_visual', 'prop_10x9x3.glb', (0, 0, 0, 0, 0, 0))))
        out.append('''    <joint name="%s_joint" type="revolute">
      <parent>wing_link</parent>
      <child>%s</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.0001</damping></dynamics>
      </axis>
    </joint>
''' % (name, name))

    # --- plugins
    out.append('''
    <!-- wing tilt: rate plant. PX4 function 301 (wing_tilt's DO_SET_ACTUATOR) -> servo_4, scaled by
         SIM_GZ_SV_MIN/MAXA5 to +/- full-command rate (deg/s) and applied here as a joint velocity -->
    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
      <joint_name>wing_tilt_joint</joint_name>
      <sub_topic>servo_4</sub_topic>
      <initial_velocity>0</initial_velocity>
    </plugin>
    <!-- hull hydrodynamics on the water (PX4 gz plugin, src/modules/simulation/gz_plugins/water_hydrodynamics):
         per-panel pressure drag / suction / viscous drag (Kerner 2015, as in asv_wave_sim) on the STEP hull surface
         (hydro_panels.txt); buoyancy from hydrostatic pressure on the same closed panels when <hydrostatics> is
         true, else from the world's graded Buoyancy system on the box collisions. Nothing applied out of the water. -->
    <plugin filename="libWaterHydrodynamicsPlugin.so" name="water_hydrodynamics::WaterHydrodynamics">
%s    </plugin>
    <!-- wing angle feedback -> gz_bridge -> sensor_encoder (SIM_GZ_EN_TILT) -->
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
      <joint_name>wing_tilt_joint</joint_name>
    </plugin>
''' % ''.join('      <%s>%s</%s>\n' % (k, f(v), k) for k, v in HYDRO.items()))

    for i in range(4):
        out.append('''    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>servo_%d</joint_name>
      <sub_topic>servo_%d</sub_topic>
      <use_velocity_commands>true</use_velocity_commands>
      <p_gain>10</p_gain>
      <cmd_max>8</cmd_max>
      <cmd_min>-8</cmd_min>
    </plugin>
''' % (i, i))

    for i, d in enumerate(('ccw', 'cw')):
        out.append('''    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>rotor_%d_joint</jointName>
      <linkName>rotor_%d</linkName>
      <turningDirection>%s</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>%s</maxRotVelocity>
      <motorConstant>%s</motorConstant>
      <momentConstant>%s</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic>
      <motorNumber>%d</motorNumber>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>
''' % (i, i, d, f(MOTOR['max_rot_velocity']), f(MOTOR['motor_constant']), f(MOTOR['moment_constant']), i))

    cg = mp['aircraft_cg_model']
    rates = ''.join('      <%s>%s</%s>\n' % (k, f(v), k) for k, v in aero['rates'].items())
    surf = ''

    for i, (name, c) in enumerate(sorted(aero['surfaces'].items())):
        surf += '''      <control_surface>
        <name>%s</name>
        <index>%d</index>
        <direction>1</direction>
%s      </control_surface>
''' % (name, i, ''.join('        <%s_ctrl>%s</%s_ctrl>\n' % (k, f(v), k) for k, v in c.items()))

    out.append('''
    <!-- Aero: whole-aircraft stability derivatives (wing level in flight), fitted from %s by
         tools/stab_to_gz.py; moment reference = aircraft CG. Control derivatives per degree, joint + = TE up. -->
    <plugin filename="gz-sim-advanced-lift-drag-system" name="gz::sim::systems::AdvancedLiftDrag">
      <link_name>base_link</link_name>
      <cp>%s</cp>
      <forward>1 0 0</forward>
      <upward>0 0 1</upward>
      <area>%s</area>
      <mac>%s</mac>
      <AR>%s</AR>
      <eff>%s</eff>
      <air_density>1.2041</air_density>
      <CL0>%s</CL0>
      <CLa>%s</CLa>
      <CD0>%s</CD0>
      <Cem0>%s</Cem0>
      <Cema>%s</Cema>
      <CYb>%s</CYb>
      <Cellb>%s</Cellb>
      <Cenb>%s</Cenb>
%s      <alpha_stall>%s</alpha_stall>
      <Cema_stall>%s</Cema_stall>
      <num_ctrl_surfaces>4</num_ctrl_surfaces>
%s    </plugin>
  </model>
</sdf>
''' % (aero['stab'], ' '.join(f(v) for v in cg), f(aero['area']), f(aero['mac']), f(aero['AR']), f(aero['eff']),
       f(aero['CL0']), f(aero['CLa']), f(aero['CD0']), f(aero['Cem0']), f(aero['Cema']),
       f(aero['CYb']), f(aero['Cellb']), f(aero['Cenb']), rates, f(aero['alpha_stall']), f(aero['Cema_stall']), surf))

    # hydrodynamic surface panels for the WaterHydrodynamics plugin: model frame, re-expressed per link there
    panels = [('base_link', q) for q in cboxes.get('panels_hull', []) + cboxes.get('panels_fin', [])] \
        + [('wing_link', q) for q in cboxes.get('panels_wing', []) + cboxes.get('panels_float', [])]

    with open(os.path.join(MODEL_DIR, 'hydro_panels.txt'), 'w') as fp:
        fp.write('# WaterHydrodynamics panels, generated by tools/build_model.py from collision_boxes.json\n'
                 '# link  x y z (model frame, m)  nx ny nz (outward unit normal)  area (m^2)\n')

        for link, q in panels:
            if not (q['area'] > 0 and all(math.isfinite(v) for v in q['pos'] + q['n'])):
                continue

            fp.write('%s %s %s %.7g\n' % (link, ' '.join(f(v) for v in q['pos']), ' '.join(f(v) for v in q['n']),
                                         q['area']))

    path = os.path.join(MODEL_DIR, 'model.sdf')

    with open(path, 'w') as fp:
        fp.write(''.join(out))

    print('wrote %s' % os.path.relpath(path))

    for n in notes + mp['warnings']:
        print('  NOTE: ' + n)

    return 0


if __name__ == '__main__':
    sys.exit(build())
