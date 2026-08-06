uint64 timestamp        # time since system start (microseconds)

uint32 device_id        # unique device ID for the connected sensor

float32 angle           # measured shaft angle [rad] relative to the boot-time zero (wing level with
                        # the fuselage longitudinal/body-X axis). Continuous multi-turn accumulation,
                        # NOT wrapped: with the encoder geared up off the wing spar its travel can
                        # exceed one revolution over the wing's +/-180 deg range

uint16 raw_count        # raw absolute sensor count (AS5600: 0..4095), for diagnostics

bool valid              # true when the reading is trustworthy (magnet detected and zero reference latched)
bool zeroed             # true once the boot-time level reference has been latched
