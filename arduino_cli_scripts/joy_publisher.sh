#!/bin/bash
arduino-cli compile --fqbn arduino:avr:uno joy_publisher
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno joy_publisher

