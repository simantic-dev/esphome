#!/bin/sh
# Builds sim/home.yaml with ESPHome (Python 3.10-3.14) and copies the merged flash image to sim/firmware.factory.bin.
set -e
cd "$(dirname "$0")"
esphome compile home.yaml
cp .esphome/build/c3home/.pioenvs/c3home/firmware.factory.bin firmware.factory.bin 2>/dev/null \
  || cp .esphome/build/c3home/build/firmware.factory.bin firmware.factory.bin
