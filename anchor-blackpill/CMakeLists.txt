cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME anchor-blackpill)

# Bring-up test mode: 0 = production, 1 = board-level bring-up, 2 = module-level bring-up.
# Override at configure time: cmake -B build -DBRINGUP_MODE=2 ..
set(BRINGUP_MODE 0 CACHE STRING "SX1280 bring-up test mode (0/1/2)")

# AnchorBridge.cpp: ANCHOR_LOG() diagnostics on/off. Override: cmake -B build -DDEBUG_ANCHOR=ON ..
option(DEBUG_ANCHOR "Enable AnchorBridge ANCHOR_LOG() diagnostics" ON)

# Per-anchor identity, baked in as compile definitions (consumed by AnchorBridge.cpp) so nothing in the source has
# to be edited per board. ADDRESS comes from the shared discovery block (0xA19 upward, see PROTOCOL.md); X/Y/Z is
# this anchor's position in cm from the shared site origin -- the same frame and axes on every anchor, and it has to
# fit int16 (+-327 m) because it is sent as int16 in the wake ack.
# Override: cmake -B build/anchor2 -DANCHOR_ADDRESS=0x00000A1A -DANCHOR_X_CM=500 -DANCHOR_Y_CM=0 -DANCHOR_Z_CM=150 ..
# The defaults below are placeholders, not a real deployment position.
# NOTE: -D values are cached in the build directory -- use a separate build dir per anchor, otherwise a stale value
# from another board's configure gets flashed silently.
set(ANCHOR_ADDRESS 0x00000A20 CACHE STRING "Anchor ranging address (from the discovery block)")
set(ANCHOR_X_CM 0 CACHE STRING "Anchor X position in cm from the site origin")
set(ANCHOR_Y_CM 0 CACHE STRING "Anchor Y position in cm from the site origin")
set(ANCHOR_Z_CM 0 CACHE STRING "Anchor Z position in cm from the site origin")
message(STATUS "Anchor identity: address=${ANCHOR_ADDRESS} position=(${ANCHOR_X_CM}, ${ANCHOR_Y_CM}, ${ANCHOR_Z_CM}) cm")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Shared SX1280 protocol layer (also consumed by beacon/)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common/Sx1280Device ${CMAKE_CURRENT_BINARY_DIR}/Sx1280Device)
target_link_libraries(Sx1280Device PRIVATE stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
     Core/Src/device/AnchorBridge.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc/device
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    BRINGUP_MODE=${BRINGUP_MODE}
    $<$<BOOL:${DEBUG_ANCHOR}>:DEBUG_ANCHOR>  # AnchorBridge.cpp: ANCHOR_LOG() diagnostics on/off
    ANCHOR_ADDRESS=${ANCHOR_ADDRESS}
    ANCHOR_X_CM=${ANCHOR_X_CM}
    ANCHOR_Y_CM=${ANCHOR_Y_CM}
    ANCHOR_Z_CM=${ANCHOR_Z_CM}
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    Sx1280Device
)
