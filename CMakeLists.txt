# WalkmanBridge

cmake_minimum_required(VERSION 2.8.3)

find_package("ArmarXCore" REQUIRED)

# Include provides all necessary ArmarX CMake macros
include(${ArmarXCore_USE_FILE})

# Name for the project

armarx_project("WalkmanBridge")
depends_on_armarx_package(ArmarXGui)
depends_on_armarx_package(MemoryX)
depends_on_armarx_package(RobotAPI)
depends_on_armarx_package(VisionX)
depends_on_armarx_package(Armar3)

# Specify each ArmarX Package dependency with the following macro
# depends_on_armarx_package(ArmarXGui "OPTIONAL")

add_subdirectory(etc)

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
## System dependencies are found with CMake's conventions
find_package(Boost COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
# FIND_PACKAGE(tf REQUIRED)
# FIND_PACKAGE(tf_conversions REQUIRED)
FIND_PACKAGE(catkin REQUIRED)
FIND_PACKAGE(roscpp REQUIRED)
FIND_PACKAGE(rosconsole REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib rosconsole
  DEPENDS system_lib
)

set(catkin_LIBRARIES "${roscpp_LIBRARIES}" "${rosconsole_LIBRARIES}")

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )

link_directories(
        ${PCL_LIBRARY_DIRS}
        ${catkin_LIBRARY_DIRS}
        )

add_definitions(
        ${PCL_DEFINITIONS} -DBOOST_NO_CXX11_EXPLICIT_CONVERSION_OPERATORS
        )

        
add_subdirectory(source)

#############
## Install ##
#############


## Mark executables and/or libraries for installation
# install(TARGETS foot_planner
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# 
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
#   
# install(DIRECTORY resources/
# DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources)


install_project()

add_subdirectory(scenarios)
