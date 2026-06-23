# Battery charging information
#
# Charging power monitor.

uint64 timestamp        # time since system start (microseconds)

float32 voltage_v       # battery voltage in volts
float32 current_a       # charging current in amps, positive is current into the battery
float32 power_w         # charging power in watts

bool connected          # true if the charging sensor is read successfully
