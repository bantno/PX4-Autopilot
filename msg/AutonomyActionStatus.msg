# Executor (ahab_executor, companion computer) feedback on the current AutonomyAction:
# accept/reject with a reason, progress, the skill being run, and the derived vehicle state.
# Published on change and at 1 Hz; mirrored into PX4 via uXRCE-DDS (/fmu/in/autonomy_action_status)
# for logging. See documentation/autonomy_mdp_interface.md.

uint64 timestamp                 # time since system start (microseconds); 0 from the companion, PX4 stamps

uint32 decision_id               # AutonomyAction this status refers to

uint8 RESULT_ACCEPTED = 0
uint8 RESULT_REJECTED = 1
uint8 RESULT_IN_PROGRESS = 2
uint8 RESULT_ACHIEVED = 3
uint8 result

uint8 REJECT_NONE = 0
uint8 REJECT_LOW_BATTERY = 1     # below the executor's FLY battery floor (belt-and-braces)
uint8 REJECT_NO_POSITION = 2     # estimator not ready for flight
uint8 REJECT_FAILSAFE = 3        # PX4 failsafe active; executor waits
uint8 REJECT_RC_OVERRIDE = 4     # pilot input present; executor suspended
uint8 REJECT_INVERTED = 5        # capsized; self-right contingency first
uint8 REJECT_NOT_READY = 6       # preflight / wing / arming not ready
uint8 REJECT_LEASE_EXPIRED = 7   # action not renewed within valid_for -> FLOAT
uint8 REJECT_NOT_IN_CHARGE = 8   # another mode/executor owns the vehicle
uint8 REJECT_NO_MISSION = 9      # no valid survey mission uploaded
uint8 reject_reason

uint8 SKILL_IDLE = 0
uint8 SKILL_CHECK_PRECONDITIONS = 1
uint8 SKILL_WING_TO_FLIGHT = 2
uint8 SKILL_SELECT_MISSION = 3
uint8 SKILL_ARM = 4
uint8 SKILL_MISSION = 5          # survey mission running (takeoff, survey, landing pattern)
uint8 SKILL_RTL = 6              # early landing via the mission landing pattern
uint8 SKILL_WAIT_LANDED = 7
uint8 SKILL_FORCE_DISARM = 8
uint8 SKILL_RELEASE_WING = 9     # hand the wing back to the sun tracker
uint8 SKILL_SELF_RIGHT = 10
uint8 SKILL_HOLD_FLOAT = 11
uint8 active_skill

uint8 vehicle_state              # AutonomyObservation VEHICLE_STATE_* (the executor is authoritative)
bool executor_in_charge          # PX4 reports this executor owns the vehicle
bool lease_valid                 # current action lease has not expired
bool suspended                   # RC override / failsafe / not in charge: no commands issued
