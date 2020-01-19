add_library(stmlib_lwip INTERFACE)

target_sources(stmlib_lwip
    INTERFACE
        lwip/descriptors.cpp
        lwip/driver.cpp
)

target_include_directories(stmlib_lwip
    INTERFACE
        lwip
        .
)

add_library(stmlib_lwip_onethread INTERFACE)

target_sources(stmlib_lwip_onethread
    INTERFACE
        lwip_onethread/dma.cpp
        lwip_onethread/driver.cpp
        lwip_onethread/lwip_thread.cpp
)

target_include_directories(stmlib_lwip_onethread
    INTERFACE
        lwip_onethread
        .
)
