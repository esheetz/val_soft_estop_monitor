/**
 * End-Effector Monitor
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#include <monitors/end_effector_monitor.h>

// CONSTRUCTORS/DESTRUCTORS
EndEffectorMonitor::EndEffectorMonitor() {
    // initialize hand and head delta limits to some default
    EE_HAND_POS_DELTA_LIMIT_ = 10.0;
    EE_HAND_ORI_DELTA_LIMIT_ = 10.0;
    EE_HEAD_POS_DELTA_LIMIT_ = 10.0;
    EE_HEAD_ORI_DELTA_LIMIT_ = 10.0;
    // NOTE initialization does not matter, will immediately be reconfigured to defaults set in cfg/EndEffectorMonitorParams.cfg

    ROS_INFO("[End-Effector Monitor] Constructed");
}

EndEffectorMonitor::~EndEffectorMonitor() {
    ROS_INFO("[End-Effector Monitor] Destroyed");
}

// INITIALIZATION
void EndEffectorMonitor::initializeMonitor(const ros::NodeHandle& nh) {
    GenericMonitor::initializeMonitor(nh);

    // initialize connections
    initializeConnections();

    // initialize dynamic reconfigure server
    initializeDynamicReconfigureServer();

    ROS_INFO("[End-Effector Monitor] Initialized");

    return;
}

// CONNECTIONS
bool EndEffectorMonitor::initializeConnections() {
    GenericMonitor::initializeConnections();

    return true;
}

// DYNAMIC RECONFIGURE SERVER
void EndEffectorMonitor::initializeDynamicReconfigureServer() {
    // initialize callback type
    dynamic_reconfigure::Server<val_soft_estop_monitor::EndEffectorMonitorParamsConfig>::CallbackType f;

    // bind function to callback
    f = boost::bind(&EndEffectorMonitor::paramReconfigureCallback, this, _1, _2);

    // set callback for server
    reconfigure_server_.setCallback(f);

    return;
}

// CALLBACKS
void EndEffectorMonitor::paramReconfigureCallback(val_soft_estop_monitor::EndEffectorMonitorParamsConfig &config, uint32_t level) {
    // take params from reconfigure request and store them internally
    debug_ = config.debug;
    MONITOR_HANDS_ = config.monitor_hands;
    EE_HAND_POS_DELTA_LIMIT_ = config.ee_hand_pos_delta_limit;
    EE_HAND_ORI_DELTA_LIMIT_ = config.ee_hand_ori_delta_limit;
    MONITOR_HEAD_ = config.monitor_head;
    EE_HEAD_POS_DELTA_LIMIT_ = config.ee_head_pos_delta_limit;
    EE_HEAD_ORI_DELTA_LIMIT_ = config.ee_head_ori_delta_limit;

    if( debug_ ) {
        ROS_INFO("[%s] ENTERED DEBUG MODE", getNodeName().c_str());
    }

    // hand monitor
    if( !MONITOR_HANDS_ ) {
        ROS_INFO("[%s] NOT MONITORING HANDS", getNodeName().c_str());
    }
    else {
        ROS_INFO("[%s] Reconfigured hand position limit to %f meters and hand orientation limit to %f radians",
                 getNodeName().c_str(), EE_HAND_POS_DELTA_LIMIT_, EE_HAND_ORI_DELTA_LIMIT_);
    }

    // head monitor
    if( !MONITOR_HEAD_ ) {
        ROS_INFO("[%s] NOT MONITORING HEAD", getNodeName().c_str());
    }
    else {
        ROS_INFO("[%s] Reconfigured head position limit to %f meters and head orientation limit to %f radians",
                 getNodeName().c_str(), EE_HEAD_POS_DELTA_LIMIT_, EE_HEAD_ORI_DELTA_LIMIT_);
    }

    return;
}

// MONITOR FUNCTIONS
bool EndEffectorMonitor::checkMonitorCondition() {
    return checkEndEffectorDeltas();
}

bool EndEffectorMonitor::checkEndEffectorDeltas() {
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

bool EndEffectorMonitor::checkEndEffectorPositionLimit(double pos_dist, std::string ee_name) {
    // check for end-effector
    if( (ee_name == std::string("leftPalm")) || (ee_name == std::string("rightPalm")) ) {
        // check if monitoring hands
        if( !MONITOR_HANDS_ ) {
            // ignore hands, no limits reached
            return false;
        }

        // check for hand position limit
        return checkEndEffectorPositionLimit(pos_dist, EE_HAND_POS_DELTA_LIMIT_, ee_name);
    }
    else if( ee_name == std::string("upperNeckPitchLink") ) {
        // check if monitoring head
        if( !MONITOR_HEAD_ ) {
            // ignore head, no limits reached
            return false;
        }

        // check for head position limit
        return checkEndEffectorPositionLimit(pos_dist, EE_HEAD_POS_DELTA_LIMIT_, ee_name);
    }
    else {
        // should never get here
        ROS_ERROR("[%s] End-effector %s not recognized; no limit detected", getNodeName().c_str(), ee_name.c_str());
        return false;
    }
}

bool EndEffectorMonitor::checkEndEffectorPositionLimit(double pos_dist, double dist_limit, std::string ee_name) {
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

bool EndEffectorMonitor::checkEndEffectorRotationLimit(double rot_dist, std::string ee_name) {
    // check for end-effector
    if( (ee_name == std::string("leftPalm")) || (ee_name == std::string("rightPalm")) ) {
        // check if monitoring hands
        if( !MONITOR_HANDS_ ) {
            // ignore hands, no limits reached
            return false;
        }

        // check for hand rotation limit
        return checkEndEffectorRotationLimit(rot_dist, EE_HAND_ORI_DELTA_LIMIT_, ee_name);
    }
    else if( ee_name == std::string("upperNeckPitchLink") ) {
        // check if monitoring head
        if( !MONITOR_HEAD_ ) {
            // ignore head, no limits reached
            return false;
        }

        // check for head rotation limit
        return checkEndEffectorRotationLimit(rot_dist, EE_HEAD_ORI_DELTA_LIMIT_, ee_name);
    }
    else {
        // should never get here
        ROS_ERROR("[%s] End-effector %s not recognized; no limit detected", getNodeName().c_str(), ee_name.c_str());
        return false;
    }
}

bool EndEffectorMonitor::checkEndEffectorRotationLimit(double rot_dist, double dist_limit, std::string ee_name) {
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
