
# Libs required for the tests
SET(LIBS ${LIBS} ArmarXCore PCLPointCloudProvider)
 
armarx_add_test(PCLPointCloudProviderTest PCLPointCloudProviderTest.cpp "${LIBS}")