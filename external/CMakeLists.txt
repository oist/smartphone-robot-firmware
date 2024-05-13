cmake_minimum_required(VERSION 3.13)

project(external_dependencies)

include(ExternalProject)

ExternalProject_Add(
    CException
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/CException
    GIT_REPOSITORY https://github.com/ThrowTheSwitch/CException.git
    GIT_TAG v1.3.4
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )

ExternalProject_Add(
    max77958_rp2040
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/max77958_rp2040
    GIT_REPOSITORY https://github.com/topherbuckley/max77958_rp2040.git
    GIT_TAG v0.0.1
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
