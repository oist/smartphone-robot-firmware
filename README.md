This repo hosts the firmware used by the [rp2040](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html#software-development) powered [PCB](https://github.com/oist/smartphone-robot-cad/tree/pcb) for the OIST smartphone robot project.

# Building, Flashing, and Debugging Firmware

If you're interested, you can set up the entire [C/C++ pico-sdk](https://github.com/raspberrypi/pico-sdk) on your computer to build the firmware. However, the easiest way to build, flash, or debug the firmware is by using the provided Docker image. The Docker image is based on the official pico-sdk Docker image and adds additional project dependencies.

## Pulling the Docker Image
First, pull the Docker image:
```bash
docker pull topher217/smartphone-robot-firmware:latest
```

## Running the Scripts
Use the provided `run_container.sh` script to build, flash, or debug the firmware.

1. **Building the Firmware**:
    ```bash
    ./run_container.sh build
    ```

2. **Flashing the Firmware**:
    ```bash
    ./run_container.sh flash
    ```

3. **Debugging the Firmware**:
    ```bash
    ./run_container.sh debug
    ```

This script will automatically detect the Pico Probe device and pass it to the Docker container, running the specified script inside the container.

# Editing Firmware

The firmware is located in the /src directory. You can edit the firmware by modifying the files in the /src directory. Header files are located in the /include directory.

After making changes, rebuild the firmware using:

`./run_container.sh build`

# Updating the Docker Image

If you need to update or change the Docker image, you can do so by running the following command from the docker directory:

`docker compose up --build`

All Docker files are located in the /docker directory.
