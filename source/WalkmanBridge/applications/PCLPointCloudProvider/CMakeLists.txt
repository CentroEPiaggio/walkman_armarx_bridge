armarx_component_set_name("PCLPointCloudProviderApp")

find_package(roscpp QUIET)

armarx_build_if(roscpp_FOUND "roscpp not available")

if(roscpp_FOUND)
    include_directories(${roscpp_INCLUDE_DIRS})
endif()

set(COMPONENT_LIBS ArmarXCore PCLPointCloudProvider ${roscpp_LIBRARIES})

set(EXE_SOURCE PCLPointCloudProviderApp.h main.cpp)

armarx_add_component_executable("${EXE_SOURCE}")
