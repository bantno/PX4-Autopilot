# Float/fly decision from the mission-level MDP (companion computer, ahab_policy) to the
# skill executor (ahab_executor). Level semantics, not edge: the policy republishes its
# current choice every second; re-sending the same action renews the lease. There is no
# abort action - the policy changes its mind by publishing the other action.
# Published into PX4 via uXRCE-DDS (/fmu/in/autonomy_action) so the ULog records every decision.
# See documentation/autonomy_mdp_interface.md.

uint64 timestamp                 # time since system start (microseconds); 0 from the companion, PX4 stamps

uint8 ACTION_FLOAT = 0           # moored: stay; flying: land (MDP action 0)
uint8 ACTION_FLY = 1             # moored: take off; flying: continue the survey (MDP action 1)
uint8 action

uint32 decision_id               # monotonic; increments only when a new decision is made, echoed in every status
float32 valid_for                # [s] lease; the executor degrades to FLOAT if not renewed within this time

float64 target_lat               # [deg] optional destination (NaN = none / here)
float64 target_lon               # [deg]

int32 stage_index                # MDP stage t the decision was made at
float32 q_float                  # one-step lookahead value of action 0 (for replay/analysis)
float32 q_fly                    # one-step lookahead value of action 1

uint8 SOURCE_MDP = 0             # produced by the ported lookahead
uint8 SOURCE_OVERRIDE = 1        # forced by operator/test parameter
uint8 SOURCE_FALLBACK = 2        # policy inputs invalid or stage out of range -> safe FLOAT
uint8 policy_source
