# Float/Fly Autonomy — MDP ↔ PX4 Interface Definition (v1)

> The mission-level MDP runs on a **companion computer** (Raspberry Pi, `ahab-companion`) and
> decides one thing at each stage: **FLOAT** (stay on the water, harvest solar energy) or **FLY**
> (spend energy on an aerial survey). A companion-side executor turns that into a sequence of
> existing PX4 skills; PX4 enforces its own safety envelope and logs everything. This document
> is the contract. v1 supersedes the v0 draft: the executor now lives on the companion as a
> PX4 external mode/executor, the observation is assembled on the companion, and FLY is a
> pre-uploaded survey mission.

## 1. Division of authority

| Layer | Owns | Never does |
|---|---|---|
| **MDP policy (`ahab_policy`, Pi)** | The float/fly decision and its cadence; the state estimate it decides on (coulomb count, wind, stage clock) | Vehicle commands of any kind |
| **Executor (`ahab_executor`, Pi)** | Sequencing PX4 modes/commands for FLY and FLOAT, the wing hand-off, the self-right contingency, accept/reject | Bypassing arming checks or failsafes; flying anything but PX4's own modes |
| **PX4 (firmware)** | Mode execution (Mission, RTL, SELF_RIGHT), arming checks, failsafes, RC override, logging | Second-guessing the float/fly choice while the envelope is satisfied |

Design rule: **the MDP proposes, the executor sequences, PX4 disposes.** Every action can be
rejected with a reason; the RC pilot and commander failsafes outrank both companion nodes.

The policy is a faithful port of the simulator's optimal arm
(`OptimalContinuousAnalyticalPolicySimulation.choose_action_batch` in Solar-Simulator); the
deployment exists to validate that simulator, so the decision rule is never re-derived on the
vehicle. See `Solar-Simulator/SolarSimulator/export/policy_runtime.py`.

## 2. The three messages

Defined as uORB messages in `msg/` (single source of truth), carried by uXRCE-DDS, and **logged
by PX4**: the companion publishes all three into `/fmu/in/...` so the ULog records exactly what
the policy saw and decided. All angles rad, SI units. `timestamp` is left 0 by the companion and
stamped by PX4 on receipt.

### 2.1 `AutonomyObservation` — assembled by `ahab_policy`, 1 Hz (`/fmu/in/autonomy_observation`)

| Field | Source | Why |
|---|---|---|
| `vehicle_state`, `mdp_mode` | echoed from `AutonomyActionStatus` (executor is authoritative) | MDP state[1]: 0 moored, 1 flying |
| `stage_index` | UTC clock vs bundle `start_datetime_utc`, `delta_t` | MDP stage t |
| `battery_energy_j`, `battery_soc_pct`, `battery_soc_binned` | Pi coulomb count (discharge from `battery_status`, charge from `battery_charging`), binned exactly like the solver | Energy state; `soc_binned` is what the lookahead uses |
| `battery_voltage_v`, `battery_current_a`, `charge_current_a`, `charge_power_w`, `net_power_w` | `battery_status`, `battery_charging` (INA226) | Health and harvest rate |
| `solar_gain_forecast_j` | `charge_power_w × delta_t` (persistence) | Lookahead input (the simulator uses the stage's realized gain) |
| `solar_gain_realized_j` | integrated over the stage just ended | Forecast-error analysis / offline replay |
| `wind_speed_mps`, `wind_source`, `wind_bin` | floating: averaged pitot TAS (`airspeed`) × height scale; flying: `|wind|`; else bundle Weibull mean | Lookahead input and chain regime index |
| `sun_elevation`, `sun_azimuth`, `sun_tracking` | `sun_tracker_status` | Harvest context |
| `lat`, `lon` | `vehicle_global_position` | Position |
| `airspeed_valid`, `position_valid`, `heading_valid` | estimator flags / `vehicle_local_position` | Whether FLY is well-posed |
| `inverted` | attitude, `dcm_z(2) < -SR_INV_THR` | Capsized |
| `at_rest` | `vehicle_land_detected.landed` | Known unreliable on water; informational |
| `failsafe_active`, `rc_override` | `vehicle_status`, `manual_control_setpoint` | PX4 / pilot has taken over |

### 2.2 `AutonomyAction` — `ahab_policy` → `ahab_executor`, 1 Hz (`/fmu/in/autonomy_action`)

| Field | Meaning |
|---|---|
| `action` | `ACTION_FLOAT = 0`, `ACTION_FLY = 1` (MDP action; mode-dependent: moored {stay, take off}, flying {land, continue}) |
| `decision_id` | Monotonic; increments only when a new decision is made (stage boundary or executor-reported mode change) |
| `valid_for` | **Lease** [s]; republished every second, default 15 s |
| `target_lat/lon` | Optional (NaN = none). Unused by the survey-mission executor in v1 |
| `stage_index`, `q_float`, `q_fly` | The stage and the two lookahead values, for replay |
| `policy_source` | `MDP`, `OVERRIDE` (operator/test parameter), `FALLBACK` (inputs invalid or stage out of range → FLOAT) |

Semantics are **level, not edge**: re-sending FLY while flying renews the lease. There is no
abort action; the policy changes its mind by publishing the other action.

### 2.3 `AutonomyActionStatus` — `ahab_executor` → policy/log, on change + 1 Hz (`/fmu/in/autonomy_action_status`)

`decision_id`, `result` (ACCEPTED / REJECTED / IN_PROGRESS / ACHIEVED), `reject_reason`
(NONE, LOW_BATTERY, NO_POSITION, FAILSAFE, RC_OVERRIDE, INVERTED, NOT_READY, LEASE_EXPIRED,
NOT_IN_CHARGE, NO_MISSION), `active_skill` (§3), `vehicle_state` (§2.4), `executor_in_charge`,
`lease_valid`, `suspended`.

### 2.4 `vehicle_state` (derived by the executor, in this precedence)

```
UNSAFE             failsafe | RC sticks moving | executor not in charge  — policy holds its last decision
SELF_RIGHTING      nav_state == SELF_RIGHT | self_right_status.active
INVERTED           disarmed & attitude inverted
TAKING_OFF         armed & Mission & (landed | current item is TAKEOFF)
LANDING            armed & (RTL | Mission with current item LAND)
FLYING             armed & !landed
FLOATING_CHARGING  disarmed, upright, sun tracker enabled+valid, wing owned by SOURCE_SUN_TRACKER
FLOATING           otherwise
```
`mdp_mode = 1` for FLYING / TAKING_OFF / LANDING, else 0.

## 3. Action → skill sequences (executor, BehaviorTree.CPP on the Pi)

**ACTION_FLY** (from floating):
1. `CHECK_PRECONDITIONS` — executor in charge, not inverted, estimator aligned with position,
   preflight checks pass, a valid mission is uploaded, energy ≥ executor battery floor
   (belt-and-braces duplicate of what the MDP knows), no failsafe. Failure → `REJECTED`.
2. `WING_TO_FLIGHT` — publish `wing_tilt_setpoint{SOURCE_AUTONOMY}` at 5 Hz (the controller
   releases a source after 0.5 s of silence); the sun tracker (priority 0) is displaced. Success
   when `wing_tilt_status.source == SOURCE_AUTONOMY` and the error is within tolerance.
3. `SELECT_MISSION` — `VEHICLE_CMD_SET_NAV_STATE(AUTO_MISSION)` while disarmed; poll `nav_state`.
4. `ARM` — arm; the mission's `NAV_TAKEOFF` item runs the water takeoff through the existing
   runway-takeoff path (`RWTO_TKOFF 1`).
5. `MISSION` — `IN_PROGRESS`; `ACHIEVED` once airborne. The survey mission ends with
   `DO_LAND_START` + loiter-to-altitude + `NAV_LAND` back at the mooring, so a FLY that runs to
   completion lands by itself.
6. On landing: `FORCE_DISARM` if auto-disarm did not fire (magic 21196, as `self_right` does),
   then `RELEASE_WING` (stop publishing → sun tracker takes the wing when `SUN_TRK_EN=1`).

**ACTION_FLOAT** (from flying): `RTL` with `RTL_TYPE 1` (return via the mission landing
pattern) → `WAIT_LANDED` → `FORCE_DISARM` → `RELEASE_WING` → `HOLD_FLOAT`.

**ACTION_FLOAT** (from floating): `HOLD_FLOAT` (`ACHIEVED`). Sun tracking is param-enabled
(`SUN_TRK_EN`), not commanded.

**Standing contingency:** `INVERTED` while floating → executor commands `DO_SET_MODE`
(custom main mode 12, SELF_RIGHT; nav_state 16), arms, waits for `self_right_status` to
finish (the module disarms itself), then resumes FLOAT. Automatic, not an MDP action; the MDP
feels the energy cost through the coulomb count.

Rejections are evaluated at the *transition*, not continuously. Once airborne, an energy dip is
the MDP's problem (it will command FLOAT), not an automatic executor abort — except where PX4
failsafes already act.

## 4. Liveness & safe defaults

- **Lease expiry** (`valid_for` not renewed: policy crash, node death): executor degrades to
  **FLOAT** — RTL if flying, hold if floating — with `REJECT_LEASE_EXPIRED`.
- **RC override:** any stick motion suspends the executor (`RC_OVERRIDE`); it resumes only on a
  **fresh `decision_id`** with sticks quiet. A pilot taking the mode switch deactivates the
  executor's mode; PX4's own behaviour applies.
- **Executor death:** PX4's external-mode liveness check removes the mode; a running Mission or
  RTL continues to its own landing. Floating is the absorbing safe state.
- **Failsafes:** untouched and authoritative; the policy observes `failsafe_active` and waits.
- **Battery floor for FLY:** kept as an executor parameter in addition to the MDP.

## 5. Transport and firmware footprint

- **uXRCE-DDS over Ethernet** (Pixhawk 6X ↔ Pi; `UXRCE_DDS_CFG 1000`, agent on the Pi at UDP
  8888). SITL uses the same client on UDP 8888.
- Firmware changes are limited to: the three `msg/` files; `WingTiltSetpoint.SOURCE_AUTONOMY`
  (priority between sun tracker and console); DDS exposure of `battery_charging`,
  `sun_tracker_status`, `self_right_status`, `wing_tilt_status`, `mission_result`, `airspeed`
  (out) and `wing_tilt_setpoint` + the three autonomy topics (in); logger entries; airframe
  params `RTL_TYPE 1`, `SDLOG_MODE 1` (log from boot so disarmed FLOAT decisions are recorded),
  `UXRCE_DDS_CFG/PRT`.
- The executor uses the PX4 ROS 2 interface library (`RegisterExtComponentRequest`): a registered
  "Autonomy" mode + mode executor, `ActivateAlways` so it stays in charge while disarmed.
- The message definitions do not change across transports; only the bridge configuration does.

## 6. Policy inputs: what is and is not faithful to the simulator

The decision rule is bit-for-bit the simulator's (parity-tested). The inputs differ in kind:

| Input | Simulator | Vehicle |
|---|---|---|
| Solar gain over the stage | realized draw, known before acting | `charge_power_w × delta_t` persistence forecast; realized value logged next stage |
| Wind | realized stage wind, bin known | averaged pitot TAS while floating (height-scaled), `|wind|` in flight; digitized into the bundle's bin edges |
| Mode | deterministic (take off → flying next stage) | executor-reported; a rejected FLY leaves mode 0 |
| Energy | modeled continuous energy | Pi coulomb count; both binned identically before the lookahead |

## 7. Open items

- Confirm in SITL that `SET_NAV_STATE(AUTO_MISSION)` is accepted from the external mode while
  disarmed; fallback is arm in the owned mode then schedule Mission.
- FW land detector on water (`LNDFW_*`) may report not-landed while moored; precondition and
  force-disarm paths cover it, but tuning is a flight-test item.
- Pitot wind while moored is only meaningful when weathervaned; the height scale is a
  calibration item.
- Coulomb-count anchoring: full-charge anchor only; long-float drift to be characterised.
- Whale series alignment: the simulator tiles the diurnal series from stage 0 regardless of the
  clock time of `start_datetime`; bundles record this (`whale_series_alignment`).

## 8. What exists today that this builds on

| Piece | Status |
|---|---|
| `self_right` (inverted recovery, ends disarmed) | flying on `self-right-mode` branch |
| `wing_tilt` (single wing owner, arbitration) | done; `SOURCE_AUTONOMY` added for the executor |
| `sun_tracker` (+ AS5600, INA226 `battery_charging`) | done, ground-tested |
| XM125 radar altimeter (water landing aid) | driver done, trusted to 7 m |
| Water takeoff (RWTO path) / landing tuning | exists for SITL twin_tractor; hardware tuning TBD |
| Policy bundle export + runtime + parity test | `Solar-Simulator/SolarSimulator/export/` |
| `ahab-companion` (policy node, executor, SITL harness) | scaffolded; nodes are the next phase |
