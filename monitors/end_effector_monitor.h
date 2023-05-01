/**
 * End-Effector Monitor
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#ifndef _END_EFFECTOR_MONITOR_H_
#define _END_EFFECTOR_MONITOR_H_

#include <map>
#include <utility> // std::pair

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>

#include <dynamic_reconfigure/server.h>
#include <val_soft_estop_monitor/EndEffectorMonitorParamsConfig.h>

#include <monitors/generic_monitor.h>

// STRUCT FOR TRACKING END-EFFECTOR POSE INFO
struct EndEffectorInfo {
    std::string ee_name;
    double position_distance;
    double position_distance_threshold;
    double rotation_distance;
    double rotation_distance_threshold;
    bool limit_found;
    geometry_msgs::Point desired_position_in_world;
    geometry_msgs::Quaternion desired_orientation_in_world;
    geometry_msgs::Point current_position_in_world;
    geometry_msgs::Quaternion current_orientation_in_world;
};

class EndEffectorMonitor : public GenericMonitor
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    EndEffectorMonitor();
    virtual ~EndEffectorMonitor();

    // INITIALIZATION
    virtual void initializeMonitor(const ros::NodeHandle& nh) override;

    // CONNECTIONS
    virtual bool initializeConnections() override;

    // DYNAMIC RECONFIGURE SERVER
    void initializeDynamicReconfigureServer() override;

    // CALLBACKS
    void paramReconfigureCallback(val_soft_estop_monitor::EndEffectorMonitorParamsConfig &config, uint32_t level);

    // GETTERS/SETTERS
    virtual std::string getNodeName() = 0;

    // HELPERS
    double getEndEffectorPositionLimit(std::string ee_name);
    double getEndEffectorRotationLimit(std::string ee_name);
    void updateEndEffectorInfo(std::string ee_name,
                               geometry_msgs::Point desired_position_in_world, geometry_msgs::Quaternion desired_orientation_in_world,
                               geometry_msgs::Point current_position_in_world, geometry_msgs::Quaternion current_orientation_in_world,
                               double dist_pos, double dist_rot,
                               bool limit_pos, bool limit_rot);
    virtual void publishAllSoftEStopMessages() = 0;
    virtual void publishSoftEStopReportMessage() = 0;

    // MONITOR FUNCTIONS
    bool checkMonitorCondition() override;
    bool checkEndEffectorDeltas();
    bool checkEndEffectorPositionLimit(double pos_dist, std::string ee_name);
    bool checkEndEffectorPositionLimit(double pos_dist, double dist_limit, std::string ee_name);
    bool checkEndEffectorRotationLimit(double rot_dist, std::string ee_name);
    bool checkEndEffectorRotationLimit(double rot_dist, double dist_limit, std::string ee_name);

    /*
     * NOTE most important implementation differences for any derived class:
     *      initializeMonitor() : read any needed params from node handle
     *      initializeConnections() : initialize connections, specifically to where desired ee poses are coming from
     *      callback for desired end-effector poses : should convert from message type into geometry_msgs::Pose, which is stored in desired_ee_poses_ map
     *      getNodeName() : appropriate node name
     *      publishAllSoftEStopMessages() : publish appropriate messages to perform soft e-stop
     */

protected:
    // dynamic reconfigure server
    dynamic_reconfigure::Server<val_soft_estop_monitor::EndEffectorMonitorParamsConfig> reconfigure_server_;

    // internal storage for desired end-effector poses
    std::map<std::string, geometry_msgs::Pose> desired_ee_poses_;
    bool desired_ee_poses_received_;
    std::map<std::string, EndEffectorInfo> ee_info_;

    // limits for distances between hand/head position and orientation; can be dynamically reconfigured
    double EE_HAND_POS_DELTA_LIMIT_;
    double EE_HAND_ORI_DELTA_LIMIT_;
    double EE_HEAD_POS_DELTA_LIMIT_;
    double EE_HEAD_ORI_DELTA_LIMIT_;

    // flags for monitoring hands/head
    bool MONITOR_HANDS_;
    bool MONITOR_HEAD_;
};

#endif
