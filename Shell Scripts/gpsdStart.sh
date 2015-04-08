#!/bin/bash
#starts gpsd in uart mode

sudo killall gpsd
sudo gpsd /dev/ttyAMA0 -F /var/run/gpsd.sock

echo "gpsd started in UART mode"