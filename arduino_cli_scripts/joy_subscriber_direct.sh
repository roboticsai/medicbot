#!/bin/bash
arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328oldi joy_subscriber_direct
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano:cpu=atmega328old joy_subscriber_direct
