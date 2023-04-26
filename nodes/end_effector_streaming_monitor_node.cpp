/**
 * End-Effector Streaming Monitor Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#include <nodes/end_effector_streaming_monitor_node.h>

// CONSTRUCTORS/DESTRUCTORS
EndEffectorStreamingMonitorNode::EndEffectorStreamingMonitorNode() {
    // initialize hand and head delta limits to some default
    EE_HAND_POS_DELTA_LIMIT_ = 10.0;
    EE_HAND_ORI_DELTA_LIMIT_ = 10.0;
    EE_HEAD_POS_DELTA_LIMIT_ = 10.0;
    EE_HEAD_ORI_DELTA_LIMIT_ = 10.0;
    // NOTE initialization does not matter, will immediately be reconfigured to defaults set in cfg/EndEffectorMonitorParams.cfg

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
    GenericMonitor::initializeMonitor(nh);

    // set up parameters
    nh_.param("ee_streaming_topic", desired_ee_pose_topic_, std::string("/ihmc/valkyrie/toolbox/ik_streaming/input/kinematics_streaming_toolbox_input"));

    // initialize connections
    initializeConnections();

    // initialize dynamic reconfigure server
    initializeDynamicReconfigureServer();

    ROS_INFO("[%s] Initialized", getNodeName().c_str());

    return;
}

// CONNECTIONS
bool EndEffectorStreamingMonitorNode::initializeConnections() {
    GenericMonitor::initializeConnections();

    // end-effector streaming subscriber
    desired_ee_pose_sub_ = nh_.subscribe(desired_ee_pose_topic_, 1, &EndEffectorStreamingMonitorNode::desiredEEPoseCallback, this);

    // streaming control state publisher
    control_state_pub_ = nh_.advertise<val_vr_ros::ControlState>("/vr/control_state", 1);

    return true;
}

// DYNAMIC RECONFIGURE SERVER
void EndEffectorStreamingMonitorNode::initializeDynamicReconfigureServer() {
    // initialize callback type
    dynamic_reconfigure::Server<val_soft_estop_monitor::EndEffectorMonitorParamsConfig>::CallbackType f;

    // bind function to callback
    f = boost::bind(&EndEffectorStreamingMonitorNode::paramReconfigureCallback, this, _1, _2);

    // set callback for server
    reconfigure_server_.setCallback(f);

    return;
}

// CALLBACKS
void EndEffectorStreamingMonitorNode::paramReconfigureCallback(val_soft_estop_monitor::EndEffectorMonitorParamsConfig &config, uint32_t level) {
    // take params from reconfigure request and store them internally
    debug_ = config.debug;
    EE_HAND_POS_DELTA_LIMIT_ = config.ee_hand_pos_delta_limit;
    EE_HAND_ORI_DELTA_LIMIT_ = config.ee_hand_ori_delta_limit;
    EE_HEAD_POS_DELTA_LIMIT_ = config.ee_head_pos_delta_limit;
    EE_HEAD_ORI_DELTA_LIMIT_ = config.ee_head_ori_delta_limit;

    if( debug_ ) {
        ROS_INFO("[%s] ENTERED DEBUG MODE", getNodeName().c_str());
    }
    ROS_INFO("[%s] Reconfigured hand position limit to %f meters and hand orientation limit to %f radians",
             getNodeName().c_str(), EE_HAND_POS_DELTA_LIMIT_, EE_HAND_ORI_DELTA_LIMIT_);
    ROS_INFO("[%s] Reconfigured head position limit to %f meters and head orientation limit to %f radians",
             getNodeName().c_str(), EE_HEAD_POS_DELTA_LIMIT_, EE_HEAD_ORI_DELTA_LIMIT_);

    return;
}

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
    return std::string("End-Effector Monitor Node");
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

// MONITOR FUNCTIONS
bool EndEffectorStreamingMonitorNode::checkMonitorCondition() {
    return checkEndEffectorDeltas();
}

bool EndEffectorStreamingMonitorNode::checkEndEffectorDeltas() {
    // verify some desired pose information has been received
    if( !desired_ee_poses_received_ ) {
        // no information received, no limits reached
        return false;
    }

    // initialize flag for limit
    bool limit_found = false;

    // look through desired end-effector poses
    for( const std::pair<std::string, geometry_msgs::Pose>& ee_pose : desired_ee_poses_ ) {
        // get name and desired position/orientation of end-effector
        std::string ee_name = ee_pose.first;
        geometry_msgs::Point desired_ee_pos = ee_pose.second.position;
        geometry_msgs::Quaternion desired_ee_rot = ee_pose.second.orientation;

        // get actual pose of end-effector
        geometry_msgs::Pose actual_ee_pose;
        bool tf_succ = lookupEEWorldPose(ee_name, actual_ee_pose);

        if( !tf_succ ) {
            ROS_ERROR("[%s] Cannot get actual end-effector pose for %s, cannot monitor end-effector; no limit detected",
                      getNodeName().c_str(), ee_name.c_str());
            return false;
        }

        // get actual position and orientation of end-effector
        geometry_msgs::Point actual_ee_pos = actual_ee_pose.position;
        geometry_msgs::Quaternion actual_ee_rot = actual_ee_pose.orientation;

        // compute distances
        double dist_pos = computePositionDistance(desired_ee_pos, actual_ee_pos);
        double dist_rot = computeRotationDistance(desired_ee_rot, actual_ee_rot);

        // print end-effector distances
        if( debug_ ) {
            ROS_INFO("[%s] DEBUG MODE: End-effector %s is %f meters from desired position and %f radians from desired orientation",
                     getNodeName().c_str(), ee_name.c_str(), dist_pos, dist_rot);
        }

        // check for limits
        bool limit_pos = checkEndEffectorPositionLimit(dist_pos, ee_name);
        bool limit_rot = checkEndEffectorRotationLimit(dist_rot, ee_name);
        if( limit_pos || limit_rot ) {
            // update flag
            limit_found = true;
        }
    }

    return limit_found;
}

bool EndEffectorStreamingMonitorNode::checkEndEffectorPositionLimit(double pos_dist, std::string ee_name) {
    // check for end-effector
    if( (ee_name == std::string("leftPalm")) || (ee_name == std::string("rightPalm")) ) {
        // check for hand position limit
        return checkEndEffectorPositionLimit(pos_dist, EE_HAND_POS_DELTA_LIMIT_, ee_name);
    }
    else if( ee_name == std::string("upperNeckPitchLink") ) {
        // check for head position limit
        return checkEndEffectorPositionLimit(pos_dist, EE_HEAD_POS_DELTA_LIMIT_, ee_name);
    }
    else {
        // should never get here
        ROS_ERROR("[%s] End-effector %s not recognized; no limit detected", getNodeName().c_str(), ee_name.c_str());
        return false;
    }
}

bool EndEffectorStreamingMonitorNode::checkEndEffectorPositionLimit(double pos_dist, double dist_limit, std::string ee_name) {
    // check for limit
    if( pos_dist >= dist_limit ) {
        ROS_WARN("[%s] End-effector %s is %f meters away from desired position, which exceeds limit %f",
                 getNodeName().c_str(), ee_name.c_str(), pos_dist, dist_limit);
        return true;
    }
    else {
        return false;
    }
}

bool EndEffectorStreamingMonitorNode::checkEndEffectorRotationLimit(double rot_dist, std::string ee_name) {
    // check for end-effector
    if( (ee_name == std::string("leftPalm")) || (ee_name == std::string("rightPalm")) ) {
        // check for hand rotation limit
        return checkEndEffectorRotationLimit(rot_dist, EE_HAND_ORI_DELTA_LIMIT_, ee_name);
    }
    else if( ee_name == std::string("upperNeckPitchLink") ) {
        // check for head rotation limit
        return checkEndEffectorRotationLimit(rot_dist, EE_HEAD_ORI_DELTA_LIMIT_, ee_name);
    }
    else {
        // should never get here
        ROS_ERROR("[%s] End-effector %s not recognized; no limit detected", getNodeName().c_str(), ee_name.c_str());
        return false;
    }
}

bool EndEffectorStreamingMonitorNode::checkEndEffectorRotationLimit(double rot_dist, double dist_limit, std::string ee_name) {
    // check for limit
    if( rot_dist >= dist_limit ) {
        ROS_WARN("[%s] End-effector %s is %f radians away from desired orientation, which exceeds limit %f",
                 getNodeName().c_str(), ee_name.c_str(), rot_dist, dist_limit);
        return true;
    }
    else {
        return false;
    }
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
