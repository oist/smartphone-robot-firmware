# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3 -Wall -Wno-unused-function -Wno-unused-variable")
set(CMAKE_C_FLAGS_RELEASE "-O3 -Wall -Wno-unused-function -Wno-unused-variable")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_STDIO_USB_DEFAULT_CRLF 0)
set(PICO_STDIO_DEFAULT_CRLF 0)
set(PICO_STDIO_ENABLE_CRLF_SUPPORT 0)
set(PICO_UART_ENABLE_CRLF_SUPPORT 0)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

set(PICO_BOARD pico CACHE STRING "Board type")

# echo $EXTERNAL_LIBRARIES_DIR
message("EXTERNAL_LIBRARIES_DIR: $ENV{EXTERNAL_LIBRARIES_DIR}")

if("$ENV{EXTERNAL_LIBRARIES_DIR}" STREQUAL "")
    message(FATAL_ERROR "EXTERNAL_LIBRARIES_DIR is not defined or empty. \
You need to either use the `run_container.sh build`, or \
`export EXTERNAL_LIBRARIES_DIR=$PWD/include` from the root of the project. \
See README")
endif()

include(ExternalProject)

ExternalProject_Add(
    CException
    SOURCE_DIR $ENV{EXTERNAL_LIBRARIES_DIR}/CException
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )

ExternalProject_Add(
    max77958_rp2040
    SOURCE_DIR $ENV{EXTERNAL_LIBRARIES_DIR}/max77958_rp2040
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )

# Pull in Raspberry Pi Pico SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# set(CException ${CMAKE_SOURCE_DIR}/external/CException/lib/CException.h
#    ${CMAKE_SOURCE_DIR}/external/CException/lib/CException.c)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(robot
        VERSION 0.1
        LANGUAGES C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()


file(GLOB SOURCES "src/*.c")
message("SOURCES: ${SOURCES}")
# Add executable. Default name is the project name, version 0.1
add_executable(robot
	       ${SOURCES}
	       $ENV{EXTERNAL_LIBRARIES_DIR}/CException/lib/CException.c
	       )

pico_generate_pio_header(robot ${CMAKE_CURRENT_LIST_DIR}/src/quadrature_encoder.pio)

pico_set_program_name(robot "robot")
pico_set_program_version(robot "0.1")

pico_enable_stdio_uart(robot 0)
pico_enable_stdio_usb(robot 1)

# Add the standard library to the build
target_link_libraries(robot pico_stdlib pico_multicore hardware_pwm)

message("target_include_directories to include : ${CMAKE_CURRENT_LIST_DIR}\n")
message("CMAKE_SOURCE_DIR is set to: ${CMAKE_SOURCE_DIR}")
# Add the standard include files to the build
target_include_directories(robot PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
  $ENV{EXTERNAL_LIBRARIES_DIR}/CException/lib
  $ENV{EXTERNAL_LIBRARIES_DIR}/max77958_rp2040
)

# Add any user requested libraries
target_link_libraries(robot
    hardware_adc
    hardware_i2c
    pico_stdlib
    pico_multicore
    hardware_pio
    )

pico_add_extra_outputs(robot)
