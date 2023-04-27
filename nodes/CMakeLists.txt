#-----------------------------------------------------------------------
# Joint State Monitor Node:
# for monitoring joint velocities and torques
#-----------------------------------------------------------------------
add_executable(joint_state_monitor_node joint_state_monitor_node.cpp)
target_link_libraries(joint_state_monitor_node val_soft_estop_monitors ${catkin_LIBRARIES})
add_dependencies(joint_state_monitor_node ${PROJECT_NAME}_gencfg)

#-----------------------------------------------------------------------
# Joint State Delta Monitor Node:
# for monitoring joint velocities and torques
#-----------------------------------------------------------------------
add_executable(joint_state_delta_monitor_node joint_state_delta_monitor_node.cpp)
target_link_libraries(joint_state_delta_monitor_node val_soft_estop_monitors ${catkin_LIBRARIES})
add_dependencies(joint_state_delta_monitor_node ${PROJECT_NAME}_gencfg)

#-----------------------------------------------------------------------
# End-Effector Streaming Monitor Node:
# for monitoring desired and actual end-effector poses
# where desired poses come from IHMC Kinematics Streaming Toolbox
#-----------------------------------------------------------------------
add_executable(end_effector_streaming_monitor_node end_effector_streaming_monitor_node.cpp)
target_link_libraries(end_effector_streaming_monitor_node val_soft_estop_monitors ${catkin_LIBRARIES})
add_dependencies(end_effector_streaming_monitor_node ${PROJECT_NAME}_gencfg)
