armarx_component_set_name("PCLPointCloudProvider")

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
pcl_ros
)

armarx_build_if(PCL_FOUND "PCL not available")
armarx_build_if(catkin_FOUND "catkin not available")

if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
endif()

set(COMPONENT_LIBS ArmarXInterfaces ArmarXCore ${PCL_LIBRARIES})

set(SOURCES
./PCLPointCloudProvider.cpp
#@TEMPLATE_LINE@@COMPONENT_PATH@/@COMPONENT_NAME@.cpp
)
set(HEADERS
./PCLPointCloudProvider.h
#@TEMPLATE_LINE@@COMPONENT_PATH@/@COMPONENT_NAME@.h
)

armarx_add_component("${SOURCES}" "${HEADERS}")

# add unit tests
add_subdirectory(test)
