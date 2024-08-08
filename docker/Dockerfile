FROM ubuntu:22.04

# Add metadata labels
LABEL maintainer="topherbuckley@gmail.com"
LABEL description="Smartphone Robot Firmware Build Environment"
LABEL version="1.0"

# Install Pico-SDK Toolchain
RUN apt-get update && apt-get install -y \
	gcc-arm-none-eabi=15:10.3-2021.07-4 \
	libnewlib-arm-none-eabi=3.3.0-1.3 \
	build-essential=12.9ubuntu3 \
        libstdc++-arm-none-eabi-newlib=15:10.3-2021.07-4+17\
        git=1:2.34.1-1ubuntu1.11 \
        openocd=0.11.0-1 \
        wget=1.21.2-2ubuntu1.1 \
        tar=1.34+dfsg-1ubuntu0.1.22.04.2 \
        python3=3.10.6-1~22.04 \
        libssl-dev=3.0.2-0ubuntu1.17

# Download and install CMake (only version 3.23 available on apt on ubuntu 22.04 and >3.24 needed for DOWNLOAD_EXTRACT_TIMESTAMP used in CMakelists.txt)
RUN wget https://github.com/Kitware/CMake/releases/download/v3.30.2/cmake-3.30.2.tar.gz
RUN tar -xzf cmake-3.30.2.tar.gz
RUN cd cmake-3.30.2 && ./bootstrap && make -j4 && make install

# Set environment variable for external libraries directory
ENV EXTERNAL_LIBRARIES_DIR=/opt/external

# Install Pico-SDK checking out version 1.5.1
RUN git clone --branch master https://github.com/raspberrypi/pico-sdk.git $EXTERNAL_LIBRARIES_DIR/pico-sdk && \
    cd $EXTERNAL_LIBRARIES_DIR/pico-sdk && \
    git checkout 6a7db34 && \
    git submodule update --init
ENV PICO_SDK_PATH=$EXTERNAL_LIBRARIES_DIR/pico-sdk

# Create a directory for external libraries
RUN mkdir -p $EXTERNAL_LIBRARIES_DIR

# Download and extract CException release
RUN mkdir -p $EXTERNAL_LIBRARIES_DIR/CException
RUN wget -q https://github.com/ThrowTheSwitch/CException/archive/refs/tags/v1.3.4.tar.gz -O $EXTERNAL_LIBRARIES_DIR/CException.tar.gz \
    && tar -xzf $EXTERNAL_LIBRARIES_DIR/CException.tar.gz --strip-components=1 -C $EXTERNAL_LIBRARIES_DIR/CException \
    && rm $EXTERNAL_LIBRARIES_DIR/CException.tar.gz

# Download and extract max77958_rp2040 release
RUN mkdir -p $EXTERNAL_LIBRARIES_DIR/max77958_rp2040
RUN wget -q https://github.com/topherbuckley/max77958_rp2040/archive/refs/tags/v0.0.1.tar.gz -O $EXTERNAL_LIBRARIES_DIR/max77958_rp2040.tar.gz \
    && tar -xzf $EXTERNAL_LIBRARIES_DIR/max77958_rp2040.tar.gz --strip-components=1 -C $EXTERNAL_LIBRARIES_DIR/max77958_rp2040 \
    && rm $EXTERNAL_LIBRARIES_DIR/max77958_rp2040.tar.gz
