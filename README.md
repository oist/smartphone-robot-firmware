# Building firmware
```
docker pull topher217/smartphone-robot-firmware:latest
cd docker
docker-compose run smartphone-robot-build-env ./b.sh
```

# Editing firmware
The firmware is located in the /src directory. You can edit the firmware by modifying the files in the /src directory.
Header files are located in the /include directory.
Rebuld with `docker-compose run smartphone-robot-build-env ./b.sh`.

# Docker image
If you desire to update or change the docker image, you can do so by running the following command
`docker-compose up --build` from the docker directory.

All docker files are in /docker
The container is based on [lukstep](https://github.com/lukstep/raspberry-pi-pico-docker-sdk)
but adds additional project external dependencies
