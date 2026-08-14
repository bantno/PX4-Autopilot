# Diagnostics for the wing_tilt controller (the single owner of the tilt motor position loop).
# All angles are true wing angle (encoder de-geared by 1/TILT_GEAR), relative to the boot zero.

uint64 timestamp        # time since system start (microseconds)

uint8 SOURCE_NONE = 255
uint8 SOURCE_RECENTER = 254   # driving back to boot-zero after the ESC arming wiggle
uint8 source            # active setpoint source (WingTiltSetpoint SOURCE_*; 255 = none, wing stopped)

float32 setpoint        # active wing tilt setpoint [rad] (NaN when no source owns the wing)
float32 measured_angle  # de-geared encoder feedback [rad] (NaN without a valid encoder)
float32 error           # wrapped position error [rad] (NaN while idle)
float32 output          # normalized ESC command applied this cycle [-1, 1]

bool encoder_valid      # sensor_encoder present, valid and fresh
