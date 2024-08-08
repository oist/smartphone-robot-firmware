#!/bin/bash

# Debug the program on the RP2040

## Start the OpenOCD server
nohup openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" &

## Wait for the OpenOCD server to start
sleep 2

## Start the GDB client
gdb-multiarch build/robot.elf
