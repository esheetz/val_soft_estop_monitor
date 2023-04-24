#-----------------------------------------------------------------------
# Joint Monitor Node:
# for monitoring joint velocities and torques
#-----------------------------------------------------------------------
add_executable(joint_monitor_node joint_monitor_node.cpp)
target_link_libraries(joint_monitor_node val_soft_estop_monitors ${catkin_LIBRARIES})
add_dependencies(joint_monitor_node ${PROJECT_NAME}_gencfg)

#-----------------------------------------------------------------------
# End-Effector Monitor Node:
# for monitoring desired and actual end-effector poses
#-----------------------------------------------------------------------
add_executable(end_effector_monitor_node end_effector_monitor_node.cpp)
target_link_libraries(end_effector_monitor_node val_soft_estop_monitors ${catkin_LIBRARIES})
add_dependencies(end_effector_monitor_node ${PROJECT_NAME}_gencfg)
