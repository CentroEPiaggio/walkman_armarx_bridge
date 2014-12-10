# WalkmanBridge

cmake_minimum_required(VERSION 2.8)

find_package("ArmarXCore" REQUIRED)

# Include provides all necessary ArmarX CMake macros
include(${ArmarXCore_USE_FILE})

# Name for the project

armarx_project("WalkmanBridge")
depends_on_armarx_package(ArmarXGui)
depends_on_armarx_package(MemoryX)
depends_on_armarx_package(VisionX)

# Specify each ArmarX Package dependency with the following macro
# depends_on_armarx_package(ArmarXGui "OPTIONAL")

add_subdirectory(etc)
add_subdirectory(source)

install_project()

add_subdirectory(scenarios)