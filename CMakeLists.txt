#---------------------------------------------------------------------------
# Dependencies on ROS and IHMC controller_msgs
#---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.1)
project(val_soft_estop_monitor)

set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)

#------------------------------------------------------------------------
#     Build Setup
#------------------------------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  tf
  geometry_msgs
  sensor_msgs
  std_msgs
  controller_msgs
  val_vr_ros
)

find_package(Eigen3 REQUIRED)

#------------------------------------------------------------------------
#     dynamic_reconfigure Param Config Setup
#------------------------------------------------------------------------
generate_dynamic_reconfigure_options(
  cfg/JointStateMonitorParams.cfg
  cfg/JointStateDeltaMonitorParams.cfg
  cfg/EndEffectorMonitorParams.cfg
)
message ("generated param config files!")

#------------------------------------------------------------------------
#     catkin Setup
#------------------------------------------------------------------------
catkin_package(
  CATKIN_DEPENDS roscpp rospy tf geometry_msgs sensor_msgs std_msgs controller_msgs val_vr_ros
  DEPENDS EIGEN3
)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
message("Eigen include directories: ${EIGEN3_INCLUDE_DIRS}")

#------------------------------------------------------------------------
#     Include Directories
#------------------------------------------------------------------------
include_directories ("${PROJECT_SOURCE_DIR}/..")
include_directories ("${PROJECT_SOURCE_DIR}")
include_directories ("${PROJECT_SOURCE_DIR}/monitors")
include_directories ("${PROJECT_SOURCE_DIR}/nodes")

#---------------------------------------------------------------------------
#     Directory Selection
#---------------------------------------------------------------------------
add_subdirectory (monitors)
add_subdirectory (nodes)
