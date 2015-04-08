#!/bin/bash
#starts ahrs until output

sudo minimu9-ahrs --mode raw -b /dev/i2c-1 | head -n1

exit 0