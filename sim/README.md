# ESPHome on a simulated ESP32-C3

Stock ESPHome (`home.yaml`, ESP-IDF framework) running on [Simantic](https://simantic.dev)'s ESP32-C3 simulator,
with a BME280 on I2C, a relay on GPIO10 and a push button on GPIO9 (`home.board`). The chip joins a simulated
Wi-Fi network and serves ESPHome's own web UI. Nothing in the firmware is changed for the simulator.

| | |
|---|---|
| `sim/test.sh` | Deterministic run: 12 s of virtual time, no host network. Checks ESPHome joined Wi-Fi and read the BME280 back through its own compensation (21.5 C, 1013.2 hPa, 45.1 %). Exit 0 = pass. |
| `sim/demo` | Interactive: relays laptop port 8080 to the device's web server and opens `sim/home.html`. The left side moves the simulated parts (temperature, humidity, pressure, the button) over the control port (5810); the right side is ESPHome's web UI served by the simulated chip; the lamp follows GPIO10. |
| `sim/build.sh` | Rebuilds `sim/firmware.factory.bin` from `home.yaml` (needs ESPHome on Python 3.10-3.14). |

```
SIM=/path/to/sim sim/test.sh
SIM=/path/to/sim sim/demo
```

The simulator (`sim`) is from Simantic and is not part of this repo. The BME280 and the button/relay are Python
peers in `sim/peers/`. The device uses a static address (10.0.2.15) so the deterministic run needs no DHCP server.
ESPHome's web UI loads its script from oi.esphome.io, so the browser needs internet access.
