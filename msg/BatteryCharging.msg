# Battery charging information
#
# Charging power monitor (ina226_charger or ina228_charger).
# Fields marked [INA228] are only filled by ina228_charger and stay zero/false for the INA226.

uint64 timestamp        # time since system start (microseconds)

float32 voltage_v       # battery voltage in volts
float32 current_a       # charging current in amps, positive is current into the battery
float32 power_w         # charging power in watts, positive is power into the battery

float32 energy_wh       # [INA228] hardware-accumulated energy in watt-hours since the accumulators were last reset. Integrates |power|, so reverse current also counts
float32 charge_ah       # [INA228] hardware-accumulated net charge in amp-hours since the accumulators were last reset, positive is into the battery
float32 temperature_c   # [INA228] monitor die temperature in degrees Celsius
uint16 diag_flags       # [INA228] raw DIAG_ALRT register
bool accum_overflow     # [INA228] true if the ENERGY or CHARGE accumulator overflowed since the last reset

bool connected          # true if the charging sensor is read successfully
