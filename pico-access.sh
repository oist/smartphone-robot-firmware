#!/bin/bash

# Find the device by vendor and product ID
DEVICE_PATH=$(lsusb | grep -i "2e8a:000c" | awk '{print "/dev/bus/usb/" $2 "/" $4}' | sed 's/://')

if [ -n "$DEVICE_PATH" ]; then
    ln -sf "$DEVICE_PATH" ~/pico-probe
    echo "Symlink created: ~/pico-probe -> $DEVICE_PATH"
else
    echo "Device not found"
fi
