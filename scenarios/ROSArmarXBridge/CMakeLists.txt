# Add your components below as shown in the following example:
#
# set(SCENARIO_COMPONENTS
#    ConditionHandler
#    Observer
#    PickAndPlaceComponent)

set(SCENARIO_COMPONENTS
PCLPointCloudProviderApp
    )

# optional 3rd parameter: "path/to/global/config.cfg"
armarx_scenario("ROSArmarXBridge" "${SCENARIO_COMPONENTS}")

#set(SCENARIO_CONFIGS
#    config/ComponentName.optionalString.cfg
#    )
# optional 3rd parameter: "path/to/global/config.cfg"
#armarx_scenario_from_configs("ROSArmarXBridge" "${SCENARIO_CONFIGS}")
