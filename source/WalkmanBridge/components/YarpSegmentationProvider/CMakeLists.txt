armarx_component_set_name("YarpSegmentationProvider")

find_package(Eigen3 QUIET)
armarx_build_if(Eigen3_FOUND "Eigen3 not available")

if(Eigen3_FOUND)
    include_directories(${Eigen3_INCLUDE_DIR})
endif()

set(COMPONENT_LIBS ArmarXInterfaces ArmarXCore)

set(SOURCES
./YarpSegmentationProvider.cpp
)
set(HEADERS
./YarpSegmentationProvider.h
)


set(COMPONENT_LIBS VisionXInterfaces VisionXCore MemoryXMemoryTypes ArmarXInterfaces ArmarXCoreObservers ArmarXCore)


armarx_add_component("${SOURCES}" "${HEADERS}")

# add unit tests
add_subdirectory(test)
