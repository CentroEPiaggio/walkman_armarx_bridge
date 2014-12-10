armarx_component_set_name("PCLPointCloudProvider")

find_package(PCL COMPONENTS common Quiet)

#find_package(catkin Quiet COMPONENTS
#pcl_ros
#pcl_conversions
#)

set(SOURCES
./PCLPointCloudProvider.cpp
#@TEMPLATE_LINE@@COMPONENT_PATH@/@COMPONENT_NAME@.cpp
)
set(HEADERS
./PCLPointCloudProvider.h
#@TEMPLATE_LINE@@COMPONENT_PATH@/@COMPONENT_NAME@.h
)



set(COMPONENT_LIBS VisionXInterfaces VisionXCore ArmarXInterfaces ArmarXCore ${catkin_LIBRARIES} ${PCL_LIBRARIES})

set(EXE_SOURCES main.cpp "${SOURCES}" "${HEADERS}")

armarx_add_component_executable("${EXE_SOURCES}")

