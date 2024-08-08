#!/bin/bash

# Flash the program to the RP2040
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program build/robot.elf verify reset exit"
