/**
 * End-Effector Monitor Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#ifndef _END_EFFECTOR_MONITOR_NODE_H_
#define _END_EFFECTOR_MONITOR_NODE_H_

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

#include <dynamic_reconfigure/server.h>
#include <val_soft_estop_monitor/EndEffectorMonitorParamsConfig.h>

#include <monitors/generic_monitor_node.h>

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

class EndEffectorMonitorNode : public GenericMonitorNode
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    EndEffectorMonitorNode();
    ~EndEffectorMonitorNode();

    // INITIALIZATION
    void initializeMonitor(const ros::NodeHandle& nh) override;

    // CONNECTIONS
    bool initializeConnections() override;

    // DYNAMIC RECONFIGURE SERVER
    void initializeDynamicReconfigureServer() override;

    // CALLBACKS
    void paramReconfigureCallback(val_soft_estop_monitor::EndEffectorMonitorParamsConfig &config, uint32_t level);
    void desiredEEPoseCallback(const controller_msgs::KinematicsStreamingToolboxInputMessage& msg);

    // GETTERS/SETTERS
    std::string getNodeName() override;

    // HELPERS
    void publishFreezeControlStateMessage();
    void publishAllSoftEStopMessages() override;

    // MONITOR FUNCTIONS
    bool checkMonitorCondition() override;
    bool checkEndEffectorDeltas();
    bool checkEndEffectorPositionLimit(double pos_dist, std::string ee_name);
    bool checkEndEffectorPositionLimit(double pos_dist, double dist_limit, std::string ee_name);
    bool checkEndEffectorRotationLimit(double rot_dist, std::string ee_name);
    bool checkEndEffectorRotationLimit(double rot_dist, double dist_limit, std::string ee_name);

private:
    std::string desired_ee_pose_topic_; // topic for listening to desired end-effector poses
    ros::Subscriber desired_ee_pose_sub_; // subscriber for listening to desired end-effector poses
    ros::Publisher control_state_pub_; // publisher for streaming control state

    // dynamic reconfigure server
    dynamic_reconfigure::Server<val_soft_estop_monitor::EndEffectorMonitorParamsConfig> reconfigure_server_;

    // internal storage for monitored end-effectors
    std::map<int, std::string> ee_hash_to_name_;

    // internal storage for desired end-effector poses
    std::map<std::string, geometry_msgs::Pose> desired_ee_poses_;
    bool desired_ee_poses_received_;

    // limits for distances between hand/head position and orientation; can be dynamically reconfigured
    double EE_HAND_POS_DELTA_LIMIT_;
    double EE_HAND_ORI_DELTA_LIMIT_;
    double EE_HEAD_POS_DELTA_LIMIT_;
    double EE_HEAD_ORI_DELTA_LIMIT_;
};

#endif
