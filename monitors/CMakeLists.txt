FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)
add_library(val_soft_estop_monitors SHARED ${sources} ${headers})
target_link_libraries(val_soft_estop_monitors
					  ${catkin_LIBRARIES})
install(TARGETS val_soft_estop_monitors DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/monitors")
