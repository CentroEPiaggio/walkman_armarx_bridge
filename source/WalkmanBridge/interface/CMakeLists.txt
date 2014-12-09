###
### CMakeLists.txt file for WalkmanBridge Interfaces
###

# Dependencies on interface libraries to other ArmarX Packages must be specified
# in the following variable separated by whitespaces
# set(WalkmanBridge_INTERFACE_DEPEND ArmarXCore)

# List of slice files to include in the interface library
set(SLICE_FILES
)

# generate the interface library
armarx_interfaces_generate_library(WalkmanBridge 0.1.0 0 "${WalkmanBridge_INTERFACE_DEPEND}")
