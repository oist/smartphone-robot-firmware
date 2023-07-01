# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_C_FLAGS_DEBUG "-O0 -g3 -Wall -Wno-unused-function -Wno-unused-variable")
set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

## Adding for Ycm debugging
MACRO(INCLUDE_SUBMODULES)
    FILE(GLOB_RECURSE new_list "${CMAKE_SOURCE_DIR}/include/*.h")
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
ENDMACRO() 

INCLUDE_SUBMODULES()
message("submodule_list after to dup removal: ${dir_list}\n")


if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(robot 
        VERSION 0.1
        LANGUAGES C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(include)

file(GLOB SOURCES "*.c")
message("SOURCES: ${SOURCES}")
message("add_executable to include : ${CMAKE_SOURCE_DIR}/include/max77958/max77958_driver.h\n")
# Add executable. Default name is the project name, version 0.1
add_executable(robot 
               robot.c
	       ${SOURCES}
	       ${CMAKE_SOURCE_DIR}/include/max77958/max77958_driver.h
	       ${CMAKE_SOURCE_DIR}/include/unity/src
	       )
       
pico_set_program_name(robot "robot")
pico_set_program_version(robot "0.1")

pico_enable_stdio_uart(robot 0)
pico_enable_stdio_usb(robot 1)

# Add the standard library to the build
target_link_libraries(robot pico_stdlib pico_multicore)

message("target_include_directories to include : ${CMAKE_SOURCE_DIR}/include/max77958\n")
message("target_include_directories to include : ${CMAKE_CURRENT_LIST_DIR}\n")
# Add the standard include files to the build
target_include_directories(robot PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
  ${dir_list}
)

# Add any user requested libraries
target_link_libraries(robot 
    hardware_adc 
    hardware_i2c
    CException)

pico_add_extra_outputs(robot)

