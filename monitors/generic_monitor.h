/**
 * Generic Monitor
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#ifndef _GENERIC_MONITOR_H_
#define _GENERIC_MONITOR_H_

#include <cmath> // pow()
#include <map>
#include <utility> // std::pair
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>

class GenericMonitor
{
public:
    // CONSTRUCTORS/DESTRUCTORS
    GenericMonitor();
    virtual ~GenericMonitor();

    // INITIALIZATION
    virtual void initializeMonitor(const ros::NodeHandle& nh);

    // CONNECTIONS
    virtual bool initializeConnections();

    // DYNAMIC RECONFIGURE SERVER
    virtual void initializeDynamicReconfigureServer() = 0;

    // GETTERS/SETTERS
    double getLoopRate();
    virtual std::string getNodeName() = 0;

    // HELPERS
    double computePositionDistance(geometry_msgs::Point pos1, geometry_msgs::Point pos2);
    double computeRotationDistance(geometry_msgs::Quaternion quat1, geometry_msgs::Quaternion quat2);
    bool lookupEEWorldPose(std::string ee_name, geometry_msgs::Pose& ee_pose);
    void initializeMessageCounter();
    void decrementMessageCounter();
    void publishPauseWalkingMessage();
    void publishStopWalkingMessage();
    virtual void publishAllSoftEStopMessages() = 0;

    // MONITOR FUNCTIONS
    virtual bool checkMonitorCondition() = 0;
    void performSoftEStop();

protected:
    ros::NodeHandle nh_; // node handler
    tf::TransformListener tf_; // transforms between frames

    ros::Publisher ihmc_interface_status_pub_; // publisher for IHMC pause/stop commands
    
    int ihmc_interface_pause_stop_msg_counter_; // counter for how many times to publish pause/stop commands

    double loop_rate_;

    bool debug_;
};

#endif
