This repo hosts the firmware used by the [rp2040](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html#software-development) powered [PCB](https://github.com/oist/smartphone-robot-cad/tree/pcb) for the OIST smartphone robot project.


# Building firmware
If interested you can setup the entire [C/C++ pico-sdk](https://github.com/raspberrypi/pico-sdk) on your computer to build the firmware. However, the easiest way to build the firmware is to use the provided docker image. The docker image is based on the official pico-sdk docker image and adds additional project dependencies.

```
docker pull topher217/smartphone-robot-firmware:latest
docker compose -f docker/docker-compose.yml run smartphone-robot-build-env ./b.sh

```

# Editing firmware
The firmware is located in the /src directory. You can edit the firmware by modifying the files in the /src directory.
Header files are located in the /include directory.
Rebuld with `docker compose -f docker/docker-compose.yml run smartphone-robot-build-env ./b.sh`.

# Docker image
If you desire to update or change the docker image, you can do so by running the following command
`docker compose up --build` from the docker directory.

All docker files are in /docker
The container is based on [lukstep](https://github.com/lukstep/raspberry-pi-pico-docker-sdk)
but adds additional project external dependencies
