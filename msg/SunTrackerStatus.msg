uint64 timestamp        # time since system start (microseconds)

# Sun direction, computed from GPS position + UTC time (NaN until first valid fix/time).
float32 sun_azimuth     # sun azimuth [rad], NED frame
float32 sun_elevation   # sun elevation above horizon [rad]; <= 0 means the sun is down (parked)

# Vehicle heading (EKF2 yaw). Required for the azimuth projection: an unreferenced yaw (e.g. no
# magnetometer / no GPS heading) points the wing at the wrong bearing, so tracking gates on it.
float32 heading         # vehicle yaw [rad], NED frame (NaN until an estimate exists)
float32 heading_var     # estimator heading variance [rad^2]; gated against SUN_HDG_VAR_MAX

# Wing tilt position loop. All angles are real wing angle (encoder de-geared by 1/SUN_GEAR_RATIO),
# relative to the boot-time level zero.
float32 tilt_setpoint   # commanded wing tilt angle [rad]
float32 measured_angle  # de-geared encoder feedback [rad]
float32 error           # wrapped position error (setpoint - measured) [rad]
float32 output          # normalized ESC command applied this cycle [-1, 1]

# Gating / health. The four flags fully reconstruct the controller state:
# disabled (!enabled && !debug_sweep), waiting (no pose/encoder), sweeping, or tracking.
bool enabled            # SUN_TRK_EN
bool debug_sweep        # SUN_DBG_SWP active (bench sweep; overrides sun tracking)
bool pose_valid         # attitude + global position + UTC time all valid
bool heading_valid      # EKF2 yaw is referenced/aligned (heading_var <= SUN_HDG_VAR_MAX)
bool encoder_valid      # sensor_encoder present and valid
