add_library(pursuer_behavior_decision
  pursuer_behavior_decision.cpp
)
add_dependencies(pursuer_behavior_decision ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pursuer_behavior_decision
  ${catkin_LIBRARIES}
)

install(TARGETS pursuer_behavior_decision 
        # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        # LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES pbd_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/pursuer_behavior_decision
)
