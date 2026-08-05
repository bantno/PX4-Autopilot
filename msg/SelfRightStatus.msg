# Diagnostics for the autonomous self-righting controller (self_right module).
# All angles in radians. See documentation/self_right_architecture.md for the design.

uint64 timestamp        # time since system start (microseconds)

# State machine (matches SelfRight::State)
uint8 STATE_IDLE = 0        # mode not active (outputs inert)
uint8 STATE_VERIFY = 1      # checking preconditions, nothing moving
uint8 STATE_ROTATE_WING = 2 # driving the wing to the props-up setpoint
uint8 STATE_RIGHTING = 3    # ramped symmetric thrust toward over-center
uint8 STATE_CUT = 4         # throttle cut, parking the wing
uint8 STATE_DISARM = 5      # requesting forced disarm
uint8 state

# Abort reason (0 = none / nominal success)
uint8 ABORT_NONE = 0             # no abort (includes a successful flip)
uint8 ABORT_STICK = 1            # pilot moved the sticks
uint8 ABORT_TILT_TIMEOUT = 2     # wing failed to reach props-up within SR_TILT_TMO
uint8 ABORT_RIGHTING_TIMEOUT = 3 # no over-center within SR_TIMEOUT
uint8 ABORT_VERIFY_FAIL = 4      # preconditions not met within the verify window
uint8 abort_reason

float32 pitch_from_upright  # theta: angle of body +Z from world up [rad] (0 = upright, pi = inverted)
float32 pitch_rate          # q_body: body-Y angular rate [rad/s]
float32 tilt_angle          # measured wing tilt angle from sensor_encoder [rad]
float32 throttle            # commanded symmetric motor throttle this cycle [0, 1]

bool over_center            # CG has passed the unstable tipping point (flip will self-complete)
bool active                 # true while the mode owns the outputs
