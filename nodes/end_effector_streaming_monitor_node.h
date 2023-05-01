/**
 * End-Effector Streaming Monitor Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#ifndef _END_EFFECTOR_STREAMING_MONITOR_NODE_H_
#define _END_EFFECTOR_STREAMING_MONITOR_NODE_H_

#include <map>
#include <utility> // std::pair

#include <ros/ros.h>
#include <controller_msgs/KinematicsToolboxRigidBodyMessage.h>
#include <controller_msgs/KinematicsStreamingToolboxInputMessage.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <val_vr_ros/ControlState.h>
#include <val_safety_exception_reporter/SoftEStop.h>
#include <val_safety_exception_reporter/SoftEStopEndEffectorStreaming.h>

#include <dynamic_reconfigure/server.h>
#include <val_soft_estop_monitor/EndEffectorMonitorParamsConfig.h>

#include <monitors/end_effector_monitor.h>

// IHMC Hashes for controllable end effectors
#define WORLD_FRAME_HASH 83766130
#define MIDFEET_Z_UP_AVERAGE_YAW_FRAME_HASH -1691429831
#define WALKING_FRAME_HASH  1996516988
#define LEFT_PALM_HASH -1403058235
#define RIGHT_PALM_HASH -20689266
#define TORSO_HASH -1300704193
#define PELVIS_HASH -878626891
#define LEFT_FOOT_HASH 532235394
#define RIGHT_FOOT_HASH -284345206
#define UPPER_NECK_HASH 144630990
// from val_vr_ros/teleop_control.h

class EndEffectorStreamingMonitorNode : public EndEffectorMonitor
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    EndEffectorStreamingMonitorNode();
    ~EndEffectorStreamingMonitorNode();

    // INITIALIZATION
    void initializeMonitor(const ros::NodeHandle& nh) override;

    // CONNECTIONS
    bool initializeConnections() override;

    // CALLBACKS
    void desiredEEPoseCallback(const controller_msgs::KinematicsStreamingToolboxInputMessage& msg);

    // GETTERS/SETTERS
    std::string getNodeName() override;

    // HELPERS
    void publishFreezeControlStateMessage();
    void publishAllSoftEStopMessages() override;
    void publishSoftEStopReportMessage() override;

private:
    std::string desired_ee_pose_topic_; // topic for listening to desired end-effector poses
    ros::Subscriber desired_ee_pose_sub_; // subscriber for listening to desired end-effector poses
    ros::Publisher control_state_pub_; // publisher for streaming control state
    ros::Publisher safety_reporter_pub_; // publisher for safety reporter

    // internal storage for monitored end-effectors
    std::map<int, std::string> ee_hash_to_name_;
};

#endif
