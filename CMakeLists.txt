add_library(stmlib INTERFACE)

# base library

target_sources(stmlib
    INTERFACE
        stmlib/eth.cpp
        stmlib/nvic.cpp
        stmlib/rng.cpp
        stmlib/trace.cpp
)

target_include_directories(stmlib
    INTERFACE
        .
)

# ethernet phy drivers

add_library(dp83640 INTERFACE)

target_sources(dp83640
    INTERFACE
        stmlib/dp83640.cpp
)

# eth submodules

add_subdirectory(stmlib/eth)
