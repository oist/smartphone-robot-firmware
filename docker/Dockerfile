FROM lukstep/raspberry-pi-pico-sdk:v0.0.2

# Add metadata labels
LABEL maintainer="topherbuckley@gmail.com"
LABEL description="Smartphone Robot Firmware Build Environment"
LABEL version="1.0"

# Set environment variable for external libraries directory
ENV EXTERNAL_LIBRARIES_DIR=/opt/external

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
