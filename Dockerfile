FROM lukstep/raspberry-pi-pico-sdk:v0.0.2

# Define a variable for the working directory
ARG WORKDIR_PATH=/src

# Add metadata labels
LABEL maintainer="topherbuckley@gmail.com"
LABEL description="Smartphone Robot Firmware Build Environment"
LABEL version="1.0"

# Set the working directory inside the container
WORKDIR $WORKDIR_PATH

RUN mkdir -p ${WORKDIR_PATH}/external

# Download and extract CException release
RUN wget -q https://github.com/ThrowTheSwitch/CException/archive/refs/tags/v1.3.4.zip -O ${WORKDIR_PATH}/CException.zip \
    && unzip ${WORKDIR_PATH}/CException.zip -d ${WORKDIR_PATH}/external \
    && rm ${WORKDIR_PATH}/CException.zip

# Download and extract max77958_rp2040 release
RUN wget -q https://github.com/topherbuckley/max77958_rp2040/archive/refs/tags/v0.0.1.zip -O ${WORKDIR_PATH}/max77958_rp2040.zip \
    && unzip ${WORKDIR_PATH}/max77958_rp2040.zip -d ${WORKDIR_PATH}/external \
    && rm ${WORKDIR_PATH}/max77958_rp2040.zip
