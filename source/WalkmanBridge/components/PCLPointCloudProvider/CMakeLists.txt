armarx_component_set_name("PCLPointCloudProvider")

set(COMPONENT_LIBS ArmarXInterfaces ArmarXCore ${PCL_LIBRARIES} ${catkin_LIBRARIES}
)

set(SOURCES
./PCLPointCloudProvider.cpp
#@TEMPLATE_LINE@@COMPONENT_PATH@/@COMPONENT_NAME@.cpp
)
set(HEADERS
./PCLPointCloudProvider.h
#@TEMPLATE_LINE@@COMPONENT_PATH@/@COMPONENT_NAME@.h
)

set(EXE_SOURCES main.cpp "${SOURCES}" "${HEADERS}")
armarx_add_component_executable("${EXE_SOURCES}")

# add unit tests
add_subdirectory(test)
