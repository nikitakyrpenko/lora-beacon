# SX1280 protocol layer (SPI/opcode plumbing) shared between anchor/ and rover/.
# Built from source by each consuming project via add_subdirectory — not a single
# prebuilt binary shared across the two firmware images.

add_library(Sx1280Device STATIC
    Src/SX1280Device.cpp
)

target_include_directories(Sx1280Device PUBLIC
    Inc
)
