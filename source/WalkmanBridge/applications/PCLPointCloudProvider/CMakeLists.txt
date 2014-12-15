armarx_component_set_name("PCLPointCloudProviderApp")

find_package(roscpp QUIET)
find_package(PCL QUIET)

armarx_build_if(roscpp_FOUND "roscpp not available")
armarx_build_if(PCL_FOUND "PCL not available")

if(roscpp_FOUND AND PCL_FOUND)
    include_directories(${roscpp_INCLUDE_DIRS})
    include_directories(${PCL_INCLUDE_DIRS})
endif()

set(COMPONENT_LIBS ArmarXCore PCLPointCloudProvider ${roscpp_LIBRARIES})

set(EXE_SOURCE PCLPointCloudProviderApp.h main.cpp)

armarx_add_component_executable("${EXE_SOURCE}")
