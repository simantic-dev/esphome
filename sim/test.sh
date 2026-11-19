#!/bin/sh
# Boots sim/firmware.factory.bin on the simulated ESP32-C3 for 12 s of virtual time and checks that ESPHome joined
# Wi-Fi and read the BME280 back as configured. Needs `sim` on PATH or SIM=/path/to/sim. Exit 0 = pass.
cd "$(dirname "$0")/.."
out=$(${SIM:-sim} run --scenario sim/scenario.yaml --ascii 2>&1)
fail=0
for want in "setup() finished successfully" "Connected" "temperature 21.5 C" "pressure 1013.2 hPa" "humidity 45.1 %"; do
  printf '%s\n' "$out" | grep -qF "$want" && echo "ok   $want" || { echo "FAIL $want"; fail=1; }
done
printf '%s\n' "$out" | grep -q "Guru Meditation" && { echo "FAIL Guru Meditation"; fail=1; }
exit $fail
