This repo hosts the firmware used by the [rp2040](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html#software-development) powered [PCB](https://github.com/oist/smartphone-robot-cad/tree/pcb) for the OIST smartphone robot project.

# Building, Flashing, and Debugging Firmware

The recommended way to build, flash, and debug the firmware is by using the provided Docker image and Makefile. All dependencies are managed via Docker for a reproducible and easy workflow.

## Pulling the Docker Image
First, pull the Docker image:
```bash
docker pull topher217/smartphone-robot-firmware:latest
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
