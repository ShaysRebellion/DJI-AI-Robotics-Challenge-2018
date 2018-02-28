#!/bin/bash
v4l2-ctl -d /dev/video2 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video2 --set-ctrl=exposure_absolute=40
#v4l2-ctl -d /dev/video0 --set-ctrl=contrast=50



#                      brightness (int)    : min=-64 max=64 step=1 default=-8193 value=0
#                        contrast (int)    : min=0 max=95 step=1 default=57343 value=32
#                      saturation (int)    : min=0 max=128 step=1 default=57343 value=50
#                             hue (int)    : min=-2000 max=2000 step=1 default=-8193 value=0
#  white_balance_temperature_auto (bool)   : default=1 value=1
#                           gamma (int)    : min=100 max=300 step=1 default=57343 value=100
#                            gain (int)    : min=0 max=100 step=1 default=57343 value=0
#            power_line_frequency (menu)   : min=0 max=2 default=1 value=1
#       white_balance_temperature (int)    : min=2800 max=6500 step=1 default=57343 value=4600 flags=inactive
#                       sharpness (int)    : min=1 max=7 step=1 default=57343 value=2
#          backlight_compensation (int)    : min=0 max=3 step=1 default=57343 value=1
#                   exposure_auto (menu)   : min=0 max=3 default=0 value=3
#               exposure_absolute (int)    : min=1 max=5000 step=1 default=625 value=625 flags=inactive
#          exposure_auto_priority (bool)   : default=0 value=1
