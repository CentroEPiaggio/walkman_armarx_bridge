# WalkmanBridge

cmake_minimum_required(VERSION 2.8.3)

find_package("ArmarXCore" REQUIRED)

# Include provides all necessary ArmarX CMake macros
include(${ArmarXCore_USE_FILE})

# Name for the project
armarx_project("WalkmanBridge")

depends_on_armarx_package(MemoryX)
depends_on_armarx_package(RobotAPI)
depends_on_armarx_package(VisionX)
depends_on_armarx_package(Armar3)

add_subdirectory(etc)
add_subdirectory(scenarios)
add_subdirectory(source)

install_project()
