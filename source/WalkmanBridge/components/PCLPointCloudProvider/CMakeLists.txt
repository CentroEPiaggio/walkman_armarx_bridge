armarx_component_set_name("PCLPointCloudProvider")

find_package(PCL 1.7 QUIET)
find_package(catkin COMPONENTS pcl_ros pcl_conversions QUIET)
find_package(roscpp QUIET)
find_package(rosconsole QUIET)

armarx_build_if(PCL_FOUND "PCL not available")
armarx_build_if(roscpp_FOUND "roscpp not available")
armarx_build_if(rosconsole_FOUND "rosconsole not available")
armarx_build_if(catkin_FOUND "catkin not available")

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
endif(COMMAND cmake_policy)

include(ExternalProject)
include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


set(ROS_LIB_FOLDER $ENV{ROS_ROOT}/../../lib CACHE STRING "ros lib folder")

LINK_DIRECTORIES(${ROS_LIB_FOLDER} )

if(catkin_FOUND)
catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib rosconsole pcl_ros pcl_conversions
  DEPENDS system_lib
)

set(catkin_LIBRARIES "${roscpp_LIBRARIES}" "${rosconsole_LIBRARIES}")

endif()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

if(PCL_FOUND AND catkin_FOUND)
	include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS} -DBOOST_NO_CXX11_EXPLICIT_CONVERSION_OPERATORS)
endif()


set(SOURCES
./PCLPointCloudProvider.cpp
)
set(HEADERS
./PCLPointCloudProvider.h
)

set(COMPONENT_LIBS VisionXInterfaces VisionXCore ArmarXInterfaces ArmarXCore ${catkin_LIBRARIES} ${PCL_LIBRARIES})

armarx_add_component("${SOURCES}" "${HEADERS}")
