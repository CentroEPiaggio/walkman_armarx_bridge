
# Libs required for the tests
find_package(YARP QUIET)

armarx_build_if(YARP_FOUND "Yarp not available")

SET(LIBS ${LIBS} ArmarXCore YarpSegmentationProvider ${YARP_LIBRARIES})
 
armarx_add_test(YarpSegmentationProviderTest YarpSegmentationProviderTest.cpp "${LIBS}")

add_executable(test_serialization YarpSegmentationTestStandAlone.cpp)

target_link_libraries(test_serialization ${LIBS})
