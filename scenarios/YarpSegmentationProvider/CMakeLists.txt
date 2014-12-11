# Add your components below as shown in the following example:
#
# set(SCENARIO_COMPONENTS
#    ConditionHandler
#    Observer
#    PickAndPlaceComponent)

set(SCENARIO_COMPONENTS
    LongtermMemory
    PriorKnowledge
    CommonStorage
    WorkingMemory
    PointCloudSegmentation
)

# optional 3rd parameter: "path/to/global/config.cfg"
armarx_scenario("YarpSegmentationProvider" "${SCENARIO_COMPONENTS}")

#set(SCENARIO_CONFIGS
#    config/ComponentName.optionalString.cfg
#    )
# optional 3rd parameter: "path/to/global/config.cfg"
#armarx_scenario_from_configs("YarpSegmentationProvider" "${SCENARIO_CONFIGS}")
