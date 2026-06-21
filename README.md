This repo hosts the firmware used by the [rp2040](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html#software-development) powered [PCB](https://github.com/oist/smartphone-robot-cad/tree/pcb) for the OIST smartphone robot project.

# Updating firmware

1. Download the most recent firmware release [robot.uf2](https://github.com/oist/smartphone-robot-firmware/releases/latest)
2. Take a small screwdriver and flip the External/Phone USB switch toward External USB Side (See image below)
3. Use the screwdriver to push and hold the BOOTSEL Button
4. Attach a USB cable between the external USB port and your PC.
5. Release the BOOTSEL Button. You should now see the board appear as an external USB device.
6. Drag and drop robot.uf2 to this external USB device. The device will automatically restart and disconnect from your PC. The new firmware should now be running.
7. Return the External/Phone USB switch to Phone USB Side.
![robot bottom](media/robotBottom.png)
## Toggle Switch Legend:
| External Phone/USB Switch | MAX77958EWV USB Gate | State                                                                                                                                       |
|----------------------------|----------------------|--------------------------------------------------------------------------------------------------------------------------------------------|
| UP                         | UP                   | RP2040 D+/- lines disconnected from everything (i.e. nothing will work)                                                                    |
| DOWN                       | DOWN                 | RP2040 D+/- lines connected to external USB, phone-side USB lines connected to MAX77958, but not connected to RP2040                        |
| UP                         | DOWN                 | RP2040 D+/- lines connected to phone-side USB                                                                                              |
| DOWN                       | UP                   | RP2040 D+/- lines connected to external USB, phone-side USB lines bypass MAX77958 by shorting USB lines with MAX77958 DN1 and DP2 pins, not sure if this is harmful or even useful ever |

![robot_back](media/robotBack.png)

# Building, Flashing, and Debugging Firmware via Debugging Port

![firmware wiring](media/firmwareWiring.png)

The above wiring diagram assumes you have the debugprobe firmware running on your Raspberry Pi Pico. See [here](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html#debugging-using-another-pico-series-device) for documenation on how to do this.

The recommended way to build, flash, and debug the firmware is by using the provided Docker image and Makefile. The Docker image provides the toolchain and SDK, while project dependencies under `include/` are managed as git submodules.

## Pulling the Docker Image
First, pull the Docker image:
```bash
docker pull topher217/smartphone-robot-firmware:latest
```

## Cloning With Submodules
Third-party dependencies in `include/` are tracked as git submodules and are part of the build.

Clone the repository with:
```bash
git clone --recursive <repo-url>
```

If you already cloned the repository, initialize the submodules with:
```bash
git submodule update --init --recursive
```

## Building the Firmware
Build the firmware using the Makefile (from the `firmware` directory):
```bash
make firmware
```

## Flashing the Firmware
Flash the firmware to the device:
```bash
make flash
```

## Logging and Diagnostics
Firmware logging defaults to the USB CDC interface:
```bash
make firmware LOGGER=USB
```

To route firmware logs over the debug-probe UART instead, build with:
```bash
make firmware LOGGER=UART
make flash
```

UART logs can be captured from the host with:
```bash
tools/capture_uart_log.py --flash --timeout 15
```

Use a longer `--timeout` when you need the log to include normal startup plus manual detach/reattach cycles.

For phone charging plus Android-side motor control, the desired MAX77958 diagnostic state is:
```text
pcb_power=SOURCE pcb_data=UFP_DEVICE pd_ready=yes vbus_enabled=yes
desired phone-control state: yes
```

In raw `PD1` logs, `data=0` means the robot/RP2040 is the USB device (`UFP`) and Android is the USB host (`DFP`), which is required for Android to discover the RP2040 USB serial interface. The board-relative `pcb_power=SOURCE` state and GPIO4/GPIO5 VBUS enable indicate that the robot is charging the phone.

For isolated MAX77958 prototype testing, build the external I2C1 diagnostic firmware with:
```bash
make firmware LOGGER=UART EXTERNAL_MAX77958_TEST=1
make flash
tools/capture_uart_log.py --flash --timeout 15
```

`EXTERNAL_MAX77958_TEST=1` skips normal board bring-up and probes an external MAX77958 on I2C1. Leave it unset, or set it to `0`, for normal firmware:
```bash
make firmware LOGGER=UART
```

Forced VBUS diagnostics are disabled by default. To intentionally force MAX77958 GPIO4/GPIO5 high during bring-up, build with `MAX77958_FORCE_VBUS_DIAGNOSTIC=1`.

## Debugging the Firmware
Debugging is a two-step process:

1. **Start OpenOCD in one terminal:**
    ```bash
    make openocd
    ```
    This will start OpenOCD in a Docker container and show its output. Leave this terminal open.

2. **Start GDB in a second terminal:**
    ```bash
    make debug
    ```
    This will connect GDB to the running OpenOCD server in the same container.

- To stop OpenOCD, simply press `Ctrl+C` in the terminal where you ran `make openocd`.
- If you ever need to forcibly clean up the debug container (e.g., after an unexpected exit), run:
    ```bash
    make docker-clean
    ```

## Interactive Shell
To open an interactive shell in the Docker environment:
```bash
make shell
```

# Editing Firmware

The firmware source is in the `src/` directory. Header files are in `include/`.

After making changes, rebuild the firmware using:
```bash
make firmware
```

# Updating the Docker Image

If you need to update or change the Docker image, run the following command from the `docker` directory:
```bash
cd docker && docker build -t topher217/smartphone-robot-firmware:latest .
```

All Docker files are located in the `docker/` directory.
