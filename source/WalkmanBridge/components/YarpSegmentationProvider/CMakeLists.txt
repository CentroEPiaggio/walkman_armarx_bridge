armarx_component_set_name("YarpSegmentationProvider")

find_package(YARP QUIET)
find_package(Eigen3 QUIET)

armarx_build_if(Eigen3_FOUND "Eigen3 not available")
armarx_build_if(YARP_FOUND "YARP not available")

if(Eigen3_FOUND)
    include_directories(${Eigen3_INCLUDE_DIR})
endif()

if(YARP_FOUND)
    include_directories(${YARP_INCL1UDE_DIRS})
endif()

check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(SOURCES ./YarpSegmentationProvider.cpp)

set(HEADERS
./YarpSegmentationProvider.h
./YarpPointCloudSegmentation.h)

#if(ARMARX_BUILD)
    set(COMPONENT_LIBS VisionXInterfaces VisionXCore MemoryXMemoryTypes ArmarXInterfaces ArmarXCoreObservers ArmarXCore ${YARP_LIBRARIES})
#endif()

armarx_add_component("${SOURCES}" "${HEADERS}")

# add unit tests
add_subdirectory(test)
