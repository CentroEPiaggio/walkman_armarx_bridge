armarx_component_set_name("PCLPointCloudProvider")

# find_package(PCL COMPONENTS common QUIET)

# find_package(catkin COMPONENTS
# pcl_ros
# pcl_conversions
# QUIET)

# armarx_build_if(PCL_FOUND "PCL not available")
# armarx_build_if(${catkin_ALL_FOUND_COMPONENTS} "ros packages not available")

# if (PCL_FOUND AND catkin_ALL_FOUND_COMPONENTS)
#     include_directories(${PCL_INCLUDE_DIRS})
#     include_directories(${catkin_INCLUDE_DIRS})
# endif()

set(SOURCES
./PCLPointCloudProvider.cpp
#@TEMPLATE_LINE@@COMPONENT_PATH@/@COMPONENT_NAME@.cpp
)
set(HEADERS
./PCLPointCloudProvider.h
#@TEMPLATE_LINE@@COMPONENT_PATH@/@COMPONENT_NAME@.h
)

set(COMPONENT_LIBS VisionXInterfaces VisionXCore ArmarXInterfaces ArmarXCore ${catkin_LIBRARIES} ${PCL_LIBRARIES})

armarx_add_component("${SOURCES}" "${HEADERS}")

