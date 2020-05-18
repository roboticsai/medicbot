#!/bin/bash
arduino-cli compile --fqbn arduino:avr:uno joy_publisher_direct
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno joy_publisher_direct

