# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

# Logger interface
set(LOGGER "USB" CACHE STRING "Logging interface")
if(NOT (LOGGER STREQUAL "USB" OR LOGGER STREQUAL "UART"))
  message(FATAL_ERROR "invalid logging interface: ${LOGGER}")
endif()

# Enable colored output
set(CMAKE_COLOR_MAKEFILE ON)

# Build options
option(USE_DOCKER "Build using Docker container" ON)
option(USE_LOCAL "Build locally instead of Docker" OFF)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

# Define external dependencies
set(EXTERNAL_DEPS
    CException
    max77958_rp2040
)

# Set paths for external dependencies
set(EXTERNAL_LIBRARIES_DIR "$ENV{EXTERNAL_LIBRARIES_DIR}" CACHE PATH "Directory containing external libraries")
set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}" CACHE PATH "Path to Pico SDK")

# Check for required environment variables
if("${EXTERNAL_LIBRARIES_DIR}" STREQUAL "")
    message(FATAL_ERROR "EXTERNAL_LIBRARIES_DIR is not defined or empty.\nYou must use the Docker container to build this project.")
endif()

if("${PICO_SDK_PATH}" STREQUAL "")
    message(FATAL_ERROR "PICO_SDK_PATH is not defined or empty.\nYou must use the Docker container to build this project.")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3 -Wall -Wno-unused-function -Wno-unused-variable")
set(CMAKE_C_FLAGS_RELEASE "-O3 -Wall -Wno-unused-function -Wno-unused-variable")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_STDIO_USB_DEFAULT_CRLF 0)
set(PICO_STDIO_DEFAULT_CRLF 0)
set(PICO_STDIO_ENABLE_CRLF_SUPPORT 0)
set(PICO_UART_ENABLE_CRLF_SUPPORT 0)

# Set board type
set(PICO_BOARD pico CACHE STRING "Board type")

# Include Pico SDK
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(robot
        VERSION 0.1
        LANGUAGES C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

file(GLOB SOURCES "src/*.c")
message(STATUS "SOURCES: ${SOURCES}")

# Add executable
add_executable(robot
    ${SOURCES}
    $ENV{EXTERNAL_LIBRARIES_DIR}/CException/lib/CException.c
)

pico_generate_pio_header(robot ${CMAKE_CURRENT_LIST_DIR}/src/quadrature_encoder.pio)

pico_set_program_name(robot "robot")
pico_set_program_version(robot "0.1")

target_compile_definitions(robot PUBLIC
    PICO_DEFAULT_UART=0
    PICO_DEFAULT_UART_TX_PIN=16
    PICO_DEFAULT_UART_RX_PIN=17
    PICO_DEFAULT_UART_BAUD_RATE=115200
)

if(LOGGER STREQUAL "UART")
    target_compile_definitions(robot PUBLIC LOGGER_UART=1)
endif()

pico_enable_stdio_uart(robot 0)
pico_enable_stdio_usb(robot 0)
if(LOGGER STREQUAL "UART")
    pico_enable_stdio_uart(robot 1)
elseif(LOGGER STREQUAL "USB")
    pico_enable_stdio_usb(robot 1)
endif()

# Add the standard library to the build
target_link_libraries(robot pico_stdlib pico_multicore hardware_pwm)

# Add include directories
target_include_directories(robot PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/..
    $ENV{EXTERNAL_LIBRARIES_DIR}/CException/lib
    $ENV{EXTERNAL_LIBRARIES_DIR}/max77958_rp2040
)

# Add additional libraries
target_link_libraries(robot
    hardware_adc
    hardware_i2c
    pico_stdlib
    pico_multicore
    hardware_pio
)

pico_add_extra_outputs(robot)

# Custom targets for Docker/local builds
if(USE_DOCKER AND NOT USE_LOCAL)
    add_custom_target(docker-build
        COMMAND docker run --rm -it
            -v ${CMAKE_CURRENT_SOURCE_DIR}:/project
            -w /project
            topher217/smartphone-robot-firmware:latest
            cmake --build ${CMAKE_BINARY_DIR}
        COMMENT "Building in Docker container"
    )
    add_dependencies(docker-build robot)
endif()

# Flash target
add_custom_target(flash
    COMMAND openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program ${CMAKE_BINARY_DIR}/robot.elf verify reset exit"
    COMMENT "Flashing firmware to device"
    DEPENDS robot
)

# Debug target
add_custom_target(debug
    COMMAND openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program ${CMAKE_BINARY_DIR}/robot.elf verify reset exit"
    COMMAND gdb-multiarch ${CMAKE_BINARY_DIR}/robot.elf -x .gdbinit
    COMMENT "Starting debug session"
    DEPENDS robot
)

include(ExternalProject)

# Define external dependencies
set(EXTERNAL_DEPS
    CException
    max77958_rp2040
)

foreach(dep IN LISTS EXTERNAL_DEPS)
    ExternalProject_Add(
        ${dep}
        SOURCE_DIR $ENV{EXTERNAL_LIBRARIES_DIR}/${dep}
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
endforeach()
