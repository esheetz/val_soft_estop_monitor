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

#include <monitors/generic_monitor_node.h>

class JointMonitorNode : public GenericMonitorNode
{
public:
	// CONSTRUCTORS/DESTRUCTORS
	JointMonitorNode();
	~JointMonitorNode();

	// INITIALIZATION
    void initializeMonitor(const ros::NodeHandle& nh) override;

	// CONNECTIONS
	bool initializeConnections() override;

	// DYNAMIC RECONFIGURE SERVER
	void initializeDynamicReconfigureServer() override;

	// CALLBACKS
	void paramReconfigureCallback(val_soft_estop_monitor::JointMonitorParamsConfig &config, uint32_t level);
	void jointStateCallback(const sensor_msgs::JointState& msg);

	// GETTERS/SETTERS
	std::string getNodeName() override;

	// HELPERS
	void publishAllSoftEStopMessages() override;

	// MONITOR FUNCTIONS
	bool checkMonitorCondition() override;
	bool checkVelocityTorqueLimits();

private:
	std::string joint_state_topic_; // topic for listening to joint velocities/torques
	ros::Subscriber joint_state_sub_; // subscriber for listening to joint velocities/torques

	// dynamic reconfigure server
	dynamic_reconfigure::Server<val_soft_estop_monitor::JointMonitorParamsConfig> reconfigure_server_;

	// internal storage for velocities and torques
	std::map<std::string, double> joint_velocities_;
	std::map<std::string, double> joint_torques_;

	// limits for velocity/torque; can be dynamically reconfigured
	double JOINT_TORQUE_LIMIT_;
	double JOINT_VELOCITY_LIMIT_;
};

#endif
