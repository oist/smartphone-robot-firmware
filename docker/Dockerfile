FROM ubuntu:24.04@sha256:04f510bf1f2528604dc2ff46b517dbdbb85c262d62eacc4aa4d3629783036096

# Add metadata labels
LABEL maintainer="topherbuckley@gmail.com"
LABEL description="Smartphone Robot Firmware Build Environment"
LABEL version="v1.0.1"

# Set environment variable for external libraries directory
ENV EXTERNAL_LIBRARIES_DIR=/opt/external

# Create a directory for external libraries
RUN mkdir -p $EXTERNAL_LIBRARIES_DIR

# add the project path to the gdbinit file so can automate gdb startup
RUN echo "add-auto-load-safe-path /project/.gdbinit" >> /root/.gdbinit

# Install build tools and dependencies for CMake and OpenOCD
RUN apt-get update && apt-get install -y \
    build-essential=12.10ubuntu1 \
    git=1:2.43.0-1ubuntu7.2 \
    wget=1.21.4-1ubuntu4.1 \
    libtool=2.4.7-7build1 \
    pkg-config=1.8.1-2build1 \
    texinfo=7.1-3build2 \
    cmake=3.28.3-1build7 \
    gcc-arm-none-eabi=15:13.2.rel1-2 \
    libnewlib-arm-none-eabi=4.4.0.20231231-2 \
    libstdc++-arm-none-eabi-newlib=15:13.2.rel1-2+26 \
    tar=1.35+dfsg-3build1 \
    python3=3.12.3-0ubuntu2 \
    python3-pip=24.0+dfsg-1ubuntu1.1 \
    libssl-dev=3.0.13-0ubuntu3.5 \
    libusb-1.0-0-dev=2:1.0.27-1 \
    libhidapi-dev=0.14.0-1build1 \
    gdb-multiarch=15.0.50.20240403-0ubuntu1

# Install Pico-SDK checking out version 1.5.1
RUN git clone --branch master https://github.com/raspberrypi/pico-sdk.git $EXTERNAL_LIBRARIES_DIR/pico-sdk && \
    cd $EXTERNAL_LIBRARIES_DIR/pico-sdk && \
    git checkout 6a7db34 && \
    git submodule update --init
ENV PICO_SDK_PATH=$EXTERNAL_LIBRARIES_DIR/pico-sdk

# Clone openocd and build from source checking out v0.12.0 (else cannot get >0.11.0)
RUN git clone --branch sdk-2.0.0 https://github.com/raspberrypi/openocd.git $EXTERNAL_LIBRARIES_DIR/openocd && \
    cd $EXTERNAL_LIBRARIES_DIR/openocd && \
    git checkout cf9c0b41c && \
    ./bootstrap && \
    ./configure && \
    make -j4 && \
    make install

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
