cmake_minimum_required(VERSION 3.10)
project(can_controller C)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Check DEVICE selection
if(NOT DEFINED DEVICE)
    if(DEFINED ENV{DEVICE})
        set(DEVICE $ENV{DEVICE})
    else()
        message(FATAL_ERROR "DEVICE not defined. Set -DDEVICE=mcp2518fd or -DDEVICE=mcp2515")
    endif()
endif()

message(STATUS "Selected device: ${DEVICE}")

# Source files
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(SOURCES
    ${SRC_DIR}/api.c
    ${SRC_DIR}/lib.c
    ${SRC_DIR}/bit-time.c
)

# Append device-specific source
if(DEVICE STREQUAL "mcp2518fd")
    list(APPEND SOURCES ${SRC_DIR}/mcp25xxfd.c)
    add_definitions(-DMCP2518FD)
elseif(DEVICE STREQUAL "mcp2517fd")
    list(APPEND SOURCES ${SRC_DIR}/mcp25xxfd.c)
    add_definitions(-DMCP2517FD)
elseif(DEVICE STREQUAL "mcp2515")
    list(APPEND SOURCES ${SRC_DIR}/mcp2515.c)
    add_definitions(-DMCP2515)
else()
    message(FATAL_ERROR "Unknown DEVICE: ${DEVICE}")
endif()

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

# set compile options
add_compile_options(-Wall -Werror)

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
    can-controller PRIVATE ${CMAKE_SOURCE_DIR}/include)
set_target_properties(can-controller PROPERTIES OUTPUT_NAME "can-controller")
