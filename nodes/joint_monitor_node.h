/**
 * Joint Monitor Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#ifndef _JOINT_MONITOR_NODE_H_
#define _JOINT_MONITOR_NODE_H_

#include <map>
#include <utility> // std::pair

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <dynamic_reconfigure/server.h>
#include <val_soft_estop_monitor/JointMonitorParamsConfig.h>

class JointMonitorNode
{
public:
	// CONSTRUCTORS/DESTRUCTORS
	JointMonitorNode(const ros::NodeHandle& nh);
	~JointMonitorNode();

	// CONNECTIONS
	bool initializeConnections();

	// DYNAMIC RECONFIGURE SERVER
	void initializeDynamicReconfigureServer();

	// CALLBACKS
	void paramReconfigureCallback(val_soft_estop_monitor::JointMonitorParamsConfig &config, uint32_t level);
	void jointStateCallback(const sensor_msgs::JointState& msg);

	// GETTERS/SETTERS
	double getLoopRate();

	// HELPERS
	void initializeMessageCounter();
	void decrementMessageCounter();
	void publishPauseWalkingMessage();
	void publishStopWalkingMessage();

	// MONITOR FUNCTIONS
	bool checkVelocityTorqueLimits();
	void performSoftEStop();

private:
	ros::NodeHandle nh_; // node handler

	ros::Publisher ihmc_interface_status_pub_; // publisher for IHMC pause/stop commands
	std::string joint_state_topic_; // topic for listening to joint velocities/torques
	ros::Subscriber joint_state_sub_; // subscriber for listening to joint velocities/torques

	// dynamic reconfigure server
	dynamic_reconfigure::Server<val_soft_estop_monitor::JointMonitorParamsConfig> reconfigure_server_;

	// internal storage for velocities and torques
	std::map<std::string, double> joint_velocities_;
	std::map<std::string, double> joint_torques_;

	int ihmc_interface_pause_stop_msg_counter_; // counter for how many times to publish pause/stop commands

	double loop_rate_;

	// limits for velocity/torque; can be dynamically reconfigured
	double JOINT_TORQUE_LIMIT_;
	double JOINT_VELOCITY_LIMIT_;
};

#endif