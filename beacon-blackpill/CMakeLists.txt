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
set(CMAKE_PROJECT_NAME beacon-blackpill)

# Bring-up test mode: 0 = production, 1 = board-level bring-up, 2 = module-level bring-up.
# Override at configure time: cmake -B build -DBRINGUP_MODE=2 ..
set(BRINGUP_MODE 2 CACHE STRING "SX1280 bring-up test mode (0/1/2)")

# SX1280Bridge.cpp: per-step SPI hal/cmd_status/busy print on every execute_step() call. Off by default -- too
# noisy for normal use. Override: cmake -B build -DDEBUG_PINS=ON ..
option(DEBUG_PINS "Enable SX1280Bridge.cpp per-step SPI trace" OFF)
# SX1280Device.cpp: prints every GPIO_Pin (BUSY/NSS/NRESET/TCXOEN) level change. Off by default -- same reason.
option(DEBUG_PIN_TRACE "Enable SX1280Device.cpp per-GPIO-edge trace" OFF)

# BeaconBridge.cpp: BEACON_LOG() diagnostics on/off. Override: cmake -B build -DDEBUG_BEACON=OFF ..
option(DEBUG_BEACON "Enable BeaconBridge BEACON_LOG() diagnostics" ON)

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
# DEBUG_PIN_TRACE is consumed inside this library's own SX1280Device.cpp -- setting it only on the executable
# target below would NOT propagate here (target_compile_definitions PRIVATE never crosses a
# target_link_libraries edge), so it must be set on this target too for the tracing to actually activate.
target_compile_definitions(Sx1280Device PRIVATE $<$<BOOL:${DEBUG_PIN_TRACE}>:DEBUG_PIN_TRACE>)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
     Core/Src/device/SX1280Bridge.cpp
     Core/Src/device/BeaconBridge.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Core/Inc/device
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    $<$<BOOL:${DEBUG_PINS}>:DEBUG_PINS>
    $<$<BOOL:${DEBUG_BEACON}>:DEBUG_BEACON>  # BeaconBridge.cpp: BEACON_LOG() diagnostics on/off
    BRINGUP_MODE=${BRINGUP_MODE}
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    Sx1280Device
)
