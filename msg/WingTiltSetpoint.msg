# Wing tilt setpoint for the wing_tilt controller — the single owner of the tilt motor.
# Multiple modules publish here; the controller uses the freshest setpoint from the
# highest-priority source (higher `source` value wins) and treats a source as released
# once it has not republished for 0.5 s.

uint64 timestamp        # time since system start (microseconds)

uint8 SOURCE_SUN_TRACKER = 0 # solar tracking (lowest priority)
uint8 SOURCE_CONSOLE = 1     # bench hold (`self_right tilt` console command)
uint8 SOURCE_SELF_RIGHT = 2  # inverted-recovery maneuver (highest priority)
uint8 source

float32 angle           # wing tilt angle setpoint [rad], wing frame, relative to the boot-time zero

uint8 ORB_QUEUE_LENGTH = 8
