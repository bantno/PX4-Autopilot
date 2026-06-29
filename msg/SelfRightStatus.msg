# Diagnostics for the autonomous self-righting controller (self_right module).
# All angles in radians. See .CLAUDE/self_right_architecture.md for the design.

uint64 timestamp        # time since system start (microseconds)

# State machine (matches SelfRight::State)
uint8 STATE_IDLE = 0        # mode not active (outputs inert)
uint8 STATE_ROTATE_WING = 1 # driving the wing to the props-up setpoint
uint8 STATE_RIGHTING = 2    # running the SR_STRATEGY righting law
uint8 STATE_CUT = 3         # throttle cut, parking the wing
uint8 STATE_DONE = 4        # handing back to MANUAL
uint8 state

# Selected righting strategy (SR_STRATEGY)
uint8 STRATEGY_OPEN_LOOP = 0
uint8 STRATEGY_ATTITUDE_PID = 1
uint8 strategy

# Abort reason (0 = none / nominal completion)
uint8 ABORT_NONE = 0
uint8 ABORT_STICK = 1       # operator moved the sticks
uint8 ABORT_RANGEFINDER = 2 # downward clearance below SR_RNG_MIN
uint8 ABORT_ESTIMATOR = 3   # accel clipping / bad-accel flag
uint8 ABORT_TIMEOUT = 4     # exceeded SR_TIMEOUT without completing
uint8 abort_reason

float32 pitch_from_upright  # theta: angle of body +Z from world up [rad] (0 = upright, pi = inverted)
float32 pitch_rate          # q_body: body-Y angular rate [rad/s]
float32 tilt_angle          # measured wing tilt angle from sensor_encoder [rad]
float32 throttle            # commanded symmetric motor throttle this cycle [0, 1]

bool over_center            # CG has passed the unstable tipping point (flip will self-complete)
bool active                 # true while the mode owns the outputs

# Adaptive thrust learning
float32 learned_thrust      # current SR_THR_LRN value used as the peak thrust
uint8 attempt               # adaptive attempt counter since boot
