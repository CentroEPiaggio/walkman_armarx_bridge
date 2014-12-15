armarx_component_set_name("YarpSegmentationProviderApp")

set(COMPONENT_LIBS ArmarXInterfaces ArmarXCore YarpSegmentationProvider)

set(EXE_SOURCE YarpSegmentationProviderApp.h main.cpp)

armarx_add_component_executable("${EXE_SOURCE}")
