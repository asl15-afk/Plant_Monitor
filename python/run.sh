#!/bin/bash
set -e

cd /var/lib/cloud9/EDES301/project_1/python

# Configure pins
./configure_pins.sh

echo "Waiting for soil sensor (I2C-2 @ 0x36)..."

# Wait up to ~30 seconds for device 0x36 to appear on bus 2
for i in {1..30}; do
  if i2cdetect -y -r 2 | grep -q "36"; then
    echo "Soil sensor detected!"
    break
  fi
  sleep 1
done

echo "Plant monitor starting main loop..."
exec /usr/bin/python3 /var/lib/cloud9/EDES301/project_1/python/main.py
