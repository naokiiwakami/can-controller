cmake_minimum_required(VERSION 3.10)
project(can_controller C)

# Source files
set(SOURCES
    api.c
    lib.c
    mcp2515.c
)

# Platform selection via CMake variable
# Usage: cmake -DPLATFORM=RASPBERRY_PI ..
if(NOT DEFINED PLATFORM)
    if(DEFINED ENV{PLATFORM})
        set(PLATFORM $ENV{PLATFORM})
    else()
        message(FATAL_ERROR "PLATFORM not defined. Set -DPLATFORM=raspberry-pi or -DPLATFORM=psoc")
    endif()
endif()

message(STATUS "Selected platform: ${PLATFORM}")

# Set compile definitions
if(PLATFORM STREQUAL "raspberry-pi")
    add_definitions(-DCONTROLLER_PLATFORM_RASPBERRY_PI)
elseif(PLATFORM STREQUAL "psoc")
    add_definitions(-DCONTROLLER_PLATFORM_PSOC)
else()
    message(FATAL_ERROR "Unknown PLATFORM: ${PLATFORM}")
endif()

if (SUPPORT_CALLBACK_INJECTION STREQUAL "true")
    add_definitions(-DSUPPORT_CALLBACK_INJECTION)
endif()

# Create static library
add_library(can-controller STATIC ${SOURCES})
target_include_directories(
    can-controller PRIVATE ${CMAKE_SOURCE_DIR}/..)
set_target_properties(can-controller PROPERTIES OUTPUT_NAME "can-controller")
