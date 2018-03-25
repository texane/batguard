#!/usr/bin/env sh
sudo avrdude \
-patmega328p -carduino -P/dev/ttyUSB0 -b57600 -D \
-Uflash:w:main.hex:i
