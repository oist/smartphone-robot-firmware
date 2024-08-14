#!/bin/bash

# Usage information
usage() {
    echo "Usage: $0 {build|debug|flash}"
    exit 1
}

# Check if the user has provided exactly one argument
if [ "$#" -ne 1 ]; then
    usage
fi

# Determine the script to run based on the argument
case "$1" in
    build)
        SCRIPT="./b.sh"
        ;;
    debug)
        SCRIPT="./debug.sh"
        ;;
    flash)
        SCRIPT="./flash.sh"
        ;;
    *)
        usage
        ;;
esac

# Find the device by vendor and product ID
DEVICE_PATH=$(lsusb | grep -i "2e8a:000c" | awk '{print "/dev/bus/usb/" $2 "/" $4}' | sed 's/://')

if [ -n "$DEVICE_PATH" ]; then
    echo "Device found at $DEVICE_PATH"

    # Export the environment variable
    export PICO_PROBE_DEVICE="$DEVICE_PATH"

    # Run the Docker Compose with the specified script
    docker compose -f docker/docker-compose.yml run --rm smartphone-robot-build-env bash -c "$SCRIPT"
else
    echo "Device not found"
    exit 1
fi
