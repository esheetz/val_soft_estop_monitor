#-----------------------------------------------------------------------
# Joint Monitor Node:
# for monitoring joint velocities and torques
#-----------------------------------------------------------------------
add_executable(joint_monitor_node joint_monitor_node.cpp)
target_link_libraries(joint_monitor_node ${catkin_LIBRARIES})
add_dependencies(joint_monitor_node ${PROJECT_NAME}_gencfg)
