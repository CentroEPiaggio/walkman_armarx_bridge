armarx_component_set_name("YarpSegmentationProviderApp")

find_package(Eigen3 QUIET)

armarx_build_if(Eigen3_FOUND "Eigen3 not available")
armarx_build_if(YARP_FOUND "YARP not available")

if(Eigen3_FOUND)
    include_directories(${Eigen3_INCLUDE_DIR})
endif()


set(COMPONENT_LIBS ArmarXInterfaces ArmarXCore YarpSegmentationProvider)

set(EXE_SOURCE YarpSegmentationProviderApp.h main.cpp)

armarx_add_component_executable("${EXE_SOURCE}")
