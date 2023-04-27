/**
 * Joint State Delta Monitor Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#ifndef _JOINT_STATE_DELTA_MONITOR_NODE_H_
#define _JOINT_STATE_DELTA_MONITOR_NODE_H_

#include <map>
#include <utility> // std::pair

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <dynamic_reconfigure/server.h>
#include <val_soft_estop_monitor/JointStateDeltaMonitorParamsConfig.h>

#include <monitors/generic_monitor.h>

// STRUCT FOR TRACKING JOINT VELOCITY DELTAS
struct JointVelocityInfo {
	std::string joint_name;
	double prev_velocity;
	double prev_timestamp;
	double curr_velocity;
	double curr_timestamp;
};

// STRUCT FOR TRACKING JOINT TORQUE DELTAS
struct JointTorqueInfo {
	std::string joint_name;
	double prev_torque;
	double prev_timestamp;
	double curr_torque;
	double curr_timestamp;
};

class JointStateDeltaMonitorNode : public GenericMonitor
{
public:
	// CONSTRUCTORS/DESTRUCTORS
	JointStateDeltaMonitorNode();
	~JointStateDeltaMonitorNode();

	// INITIALIZATION
	void initializeMonitor(const ros::NodeHandle& nh) override;

	// CONNECTIONS
	bool initializeConnections() override;

	// DYNAMIC RECONFIGURE SERVER
	void initializeDynamicReconfigureServer() override;

	// CALLBACKS
	void paramReconfigureCallback(val_soft_estop_monitor::JointStateDeltaMonitorParamsConfig &config, uint32_t level);
	void jointStateCallback(const sensor_msgs::JointState& msg);

	// GETTERS/SETTERS
	std::string getNodeName() override;

	// HELPERS
	void publishAllSoftEStopMessages() override;

	// MONITOR FUNCTIONS
	bool checkMonitorCondition() override;
	bool checkVelocityTorqueDeltaLimits();

private:
	std::string joint_state_topic_; // topic for listening to joint velocities/torques
	ros::Subscriber joint_state_sub_; // subscriber for listening to joint velocities/torques

	// dynamic reconfigure server
	dynamic_reconfigure::Server<val_soft_estop_monitor::JointStateDeltaMonitorParamsConfig> reconfigure_server_;

	// internal storage for velocities and torques
	std::map<std::string, JointVelocityInfo> joint_velocities_;
	std::map<std::string, JointTorqueInfo> joint_torques_;

	// limits for changes in velocity/torque; can be dynamically reconfigured
	double JOINT_VELOCITY_DELTA_LIMIT_;
	double JOINT_VELOCITY_TIMEOUT_;
	double JOINT_TORQUE_DELTA_LIMIT_;
	double JOINT_TORQUE_TIMEOUT_;

	// flags for monitoring velocity/torque
	bool MONITOR_VELOCITY_;
	bool MONITOR_TORQUE_;
};

#endif
