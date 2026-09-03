# Float/fly MDP observation snapshot, assembled once per second on the companion computer
# (ahab_policy) from PX4 topics plus policy-side state, and published back into PX4 via
# uXRCE-DDS (/fmu/in/autonomy_observation) purely so the ULog records exactly what the
# policy saw when it decided. See documentation/autonomy_mdp_interface.md.

uint64 timestamp                 # time since system start (microseconds); 0 from the companion, PX4 stamps

# Executor-reported vehicle state (authoritative source: AutonomyActionStatus.vehicle_state)
uint8 VEHICLE_STATE_FLYING = 0
uint8 VEHICLE_STATE_TAKING_OFF = 1
uint8 VEHICLE_STATE_LANDING = 2
uint8 VEHICLE_STATE_FLOATING = 3
uint8 VEHICLE_STATE_FLOATING_CHARGING = 4
uint8 VEHICLE_STATE_INVERTED = 5
uint8 VEHICLE_STATE_SELF_RIGHTING = 6
uint8 VEHICLE_STATE_UNSAFE = 7
uint8 vehicle_state
uint8 mdp_mode                   # MDP state[1]: 0 = moored/floating, 1 = flying

int32 stage_index                # MDP stage t; -1 before bundle start, >= horizon after the end

# Energy state (companion-side coulomb count; both quantities binned identically to the solver)
float32 battery_energy_j         # [J] continuous energy estimate
float32 battery_soc_pct          # [%] battery_energy_j / capacity
float32 battery_soc_binned       # [%] grid-aligned SoC actually fed to the lookahead
float32 battery_voltage_v        # [V] battery_status
float32 battery_current_a        # [A] discharge current (battery_status)
float32 charge_current_a         # [A] solar charge current into the battery (battery_charging)
float32 charge_power_w           # [W] battery_charging
float32 net_power_w              # [W] charge - discharge; positive = harvesting
float32 solar_gain_forecast_j    # [J] predicted harvest over the coming stage (input to the lookahead)
float32 solar_gain_realized_j    # [J] harvest actually integrated over the stage just ended (for forecast-error analysis)

# Wind input to the lookahead
float32 wind_speed_mps           # [m/s] value fed to the policy
uint8 WIND_SOURCE_NONE = 0
uint8 WIND_SOURCE_PITOT_MOORED = 1   # averaged pitot airspeed while floating (weathervaned)
uint8 WIND_SOURCE_WIND_ESTIMATE = 2  # |wind| topic while flying
uint8 WIND_SOURCE_FALLBACK = 3       # bundle Weibull mean for the stage (no measurement)
uint8 wind_source
uint8 wind_bin                   # Markov-chain regime index z = digitize(wind, edges)

# Harvest context
float32 sun_elevation            # [rad] sun_tracker_status
float32 sun_azimuth              # [rad]
bool sun_tracking                # sun tracker enabled and tracking

# Position
float64 lat                      # [deg]
float64 lon                      # [deg]

# Validity / safety flags (whether FLY is even well-posed)
bool airspeed_valid
bool position_valid
bool heading_valid
bool inverted                    # attitude says capsized
bool at_rest                     # vehicle_land_detected.landed (known unreliable on water)
bool failsafe_active             # vehicle_status.failsafe
bool rc_override                 # manual input present
