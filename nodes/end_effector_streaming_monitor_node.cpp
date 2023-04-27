/**
 * End-Effector Streaming Monitor Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#include <nodes/end_effector_streaming_monitor_node.h>

// CONSTRUCTORS/DESTRUCTORS
EndEffectorStreamingMonitorNode::EndEffectorStreamingMonitorNode() {
    // create map of hashes to end-effector names
    ee_hash_to_name_[LEFT_PALM_HASH] = "leftPalm";
    ee_hash_to_name_[RIGHT_PALM_HASH] = "rightPalm";
    ee_hash_to_name_[UPPER_NECK_HASH] = "upperNeckPitchLink";

    ROS_INFO("[%s] Constructed", getNodeName().c_str());
}

EndEffectorStreamingMonitorNode::~EndEffectorStreamingMonitorNode() {
    ROS_INFO("[%s] Destroyed", getNodeName().c_str());
}

// INITIALIZATION
void EndEffectorStreamingMonitorNode::initializeMonitor(const ros::NodeHandle& nh) {
    EndEffectorMonitor::initializeMonitor(nh);

    // set up parameters
    nh_.param("ee_streaming_topic", desired_ee_pose_topic_, std::string("/ihmc/valkyrie/toolbox/ik_streaming/input/kinematics_streaming_toolbox_input"));

    // initialize connections
    initializeConnections();

    ROS_INFO("[%s] Initialized", getNodeName().c_str());

    return;
}

// CONNECTIONS
bool EndEffectorStreamingMonitorNode::initializeConnections() {
    EndEffectorMonitor::initializeConnections();

    // end-effector streaming subscriber
    desired_ee_pose_sub_ = nh_.subscribe(desired_ee_pose_topic_, 1, &EndEffectorStreamingMonitorNode::desiredEEPoseCallback, this);

    // streaming control state publisher
    control_state_pub_ = nh_.advertise<val_vr_ros::ControlState>("/vr/control_state", 1);

    return true;
}

// CALLBACKS
void EndEffectorStreamingMonitorNode::desiredEEPoseCallback(const controller_msgs::KinematicsStreamingToolboxInputMessage& msg) {
    // check if streaming to controller
    if( msg.stream_to_controller ) {
        // streaming to controller, clear out map and loop through received inputs
        desired_ee_poses_.clear();
        for( int i = 0 ; i < msg.inputs.size() ; i++ ) {
            // get rigid body message
            controller_msgs::KinematicsToolboxRigidBodyMessage rigid_body_msg = msg.inputs[i];
            // get end-effector hash in input message
            int ee_hash = rigid_body_msg.end_effector_hash_code;
            // try to find received end-effector has in map of end-effectors being monitored
            std::map<int, std::string>::iterator it = ee_hash_to_name_.find(ee_hash);
            if( it != ee_hash_to_name_.end() ) {
                // get end-effector name
                std::string ee_name = it->second;
                // set desired end-effector pose
                geometry_msgs::Pose desired_ee_pose;
                desired_ee_pose.position = rigid_body_msg.desired_position_in_world;
                desired_ee_pose.orientation = rigid_body_msg.desired_orientation_in_world;
                // received end-effector being monitored, store desired pose in map
                desired_ee_poses_[ee_name] = desired_ee_pose;
                desired_ee_poses_received_ = true;
            }
        }
    }
    else {
        // not streaming to controller, no desired poses received
        desired_ee_poses_received_ = false;
    }

    return;
}

// GETTERS/SETTERS
std::string EndEffectorStreamingMonitorNode::getNodeName() {
    return std::string("End-Effector Steraming Monitor Node");
}

// HELPERS
void EndEffectorStreamingMonitorNode::publishFreezeControlStateMessage() {
    // initialize message
    val_vr_ros::ControlState freeze_msg;

    // freeze everything
    freeze_msg.left_hand_streaming = false;
    freeze_msg.right_hand_streaming = false;
    freeze_msg.headset_streaming = false;
    // next fields are not currently used, but just to be safe, make sure they are also not streaming
    freeze_msg.left_fingers_streaming = false;
    freeze_msg.right_fingers_streaming = false;

    // publish message
    control_state_pub_.publish(freeze_msg);

    ros::spinOnce(); // make sure messages go through

    // everything now frozen, meaning any stored desired pose info is out of date; clear everything
    desired_ee_poses_received_ = false;
    desired_ee_poses_.clear();

    return;
}

void EndEffectorStreamingMonitorNode::publishAllSoftEStopMessages() {
    // publish pause walking and stop trajectory messages
    publishPauseWalkingMessage();
    publishStopWalkingMessage();

    // publish freeze everything message
    publishFreezeControlStateMessage();

    return;
}

int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "ValkyrieEndEffectorStreamingMonitorNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create and initialize node
    EndEffectorStreamingMonitorNode ee_monitor;
    ee_monitor.initializeMonitor(nh);
    ROS_INFO("[%s] Node started!", ee_monitor.getNodeName().c_str());

    // get loop rate
    ros::Rate rate(ee_monitor.getLoopRate());

    // run node
    while( ros::ok() ) {
        // perform soft estop if necessary
        ee_monitor.performSoftEStop();

        // spin and sleep
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[%s] Node stopped, all done!", ee_monitor.getNodeName().c_str());

    return 0;
}
