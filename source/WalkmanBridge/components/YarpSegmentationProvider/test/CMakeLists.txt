
# Libs required for the tests
SET(LIBS ${LIBS} ArmarXCore YarpSegmentationProvider)
 
armarx_add_test(YarpSegmentationProviderTest YarpSegmentationProviderTest.cpp "${LIBS}")