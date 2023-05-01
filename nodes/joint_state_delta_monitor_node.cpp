/**
 * Joint State Delta Monitor Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#include <nodes/joint_state_delta_monitor_node.h>

// CONSTRUCTORS/DESTRUCTORS
JointStateDeltaMonitorNode::JointStateDeltaMonitorNode() {
    // initialize change in velocity and torque limits to some default
    JOINT_VELOCITY_DELTA_LIMIT_ = 10.0;
    JOINT_VELOCITY_TIMEOUT_     = 10.0;
    JOINT_TORQUE_DELTA_LIMIT_   = 10.0;
    JOINT_TORQUE_TIMEOUT_       = 10.0;
    MONITOR_VELOCITY_ = true;
    MONITOR_TORQUE_ = true;
    // NOTE initialization does not matter, will immediately be reconfigured to defaults set in cfg/JointStateDeltaMonitorParams.cfg

    ROS_INFO("[%s] Constructed", getNodeName().c_str());
}

JointStateDeltaMonitorNode::~JointStateDeltaMonitorNode() {
    ROS_INFO("[%s] Destroyed", getNodeName().c_str());
}

// INITIALIZATION
void JointStateDeltaMonitorNode::initializeMonitor(const ros::NodeHandle& nh) {
    GenericMonitor::initializeMonitor(nh);

    // set up parameters
    nh_.param("joint_state_topic", joint_state_topic_, std::string("/ihmc_ros/valkyrie/output/joint_states"));

    // initialize connections
    initializeConnections();

    // initialize dynamic reconfigure server
    initializeDynamicReconfigureServer();

    ROS_INFO("[%s] Initialized", getNodeName().c_str());

    return;
}

// CONNECTIONS
bool JointStateDeltaMonitorNode::initializeConnections() {
    GenericMonitor::initializeConnections();

    // joint state subscriber
    joint_state_sub_ = nh_.subscribe(joint_state_topic_, 1, &JointStateDeltaMonitorNode::jointStateCallback, this);

    // safety reporter publisher
    safety_reporter_pub_ = nh_.advertise<val_safety_exception_reporter::SoftEStop>("/valkyrie_safety_reporter/soft_estop", 10);

    return true;
}

// DYNAMIC RECONFIGURE SERVER
void JointStateDeltaMonitorNode::initializeDynamicReconfigureServer() {
    // initialize callback type
    dynamic_reconfigure::Server<val_soft_estop_monitor::JointStateDeltaMonitorParamsConfig>::CallbackType f;

    // bind function to callback
    f = boost::bind(&JointStateDeltaMonitorNode::paramReconfigureCallback, this, _1, _2);

    // set callback for server
    reconfigure_server_.setCallback(f);

    return;
}

// CALLBACKS
void JointStateDeltaMonitorNode::paramReconfigureCallback(val_soft_estop_monitor::JointStateDeltaMonitorParamsConfig &config, uint32_t level) {
    // take params from reconfigure request and store them internally
    debug_ = config.debug;
    MONITOR_VELOCITY_ = config.monitor_velocity;
    JOINT_VELOCITY_DELTA_LIMIT_ = config.joint_velocity_delta_limit;
    JOINT_VELOCITY_TIMEOUT_ = config.joint_velocity_timeout;
    MONITOR_TORQUE_ = config.monitor_torque;
    JOINT_TORQUE_DELTA_LIMIT_ = config.joint_torque_delta_limit;
    JOINT_TORQUE_TIMEOUT_ = config.joint_torque_timeout;

    if( debug_ ) {
        ROS_INFO("[%s] ENTERED DEBUG MODE", getNodeName().c_str());
    }

    // velocity monitor
    if( !MONITOR_VELOCITY_ ) {
        ROS_INFO("[%s] NOT MONITORING VELOCITY", getNodeName().c_str());
    }
    else {
        ROS_INFO("[%s] Reconfigured change in velocity limit to %f with timeout of %f seconds",
                 getNodeName().c_str(), JOINT_VELOCITY_DELTA_LIMIT_, JOINT_VELOCITY_TIMEOUT_);
    }

    // torque monitor
    if( !MONITOR_TORQUE_ ) {
        ROS_INFO("[%s] NOT MONITORING TORQUE", getNodeName().c_str());
    }
    else {
        ROS_INFO("[%s] Reconfigured change in torque limit to %f with timeout of %f seconds",
                 getNodeName().c_str(), JOINT_TORQUE_DELTA_LIMIT_, JOINT_TORQUE_TIMEOUT_);
    }

    // ROS_INFO("[%s] Reconfigured change in velocity limit to %f and change in torque limit to %f",
    //          getNodeName().c_str(), JOINT_VELOCITY_DELTA_LIMIT_, JOINT_TORQUE_DELTA_LIMIT_);

    return;
}

void JointStateDeltaMonitorNode::jointStateCallback(const sensor_msgs::JointState& msg) {
    // loop through joint state message
    for( int i = 0 ; i < msg.name.size() ; i++ ) {
        // ***** PROCESS JOINT VELOCITY INFO *****

        // initialize velocity info
        JointVelocityInfo vel_info;

        // check if joint already exists in velocity map
        std::map<std::string, JointVelocityInfo>::iterator it_vel = joint_velocities_.find(msg.name[i]);

        // update velocity info accordingly
        if( it_vel == joint_velocities_.end() ) {
            // joint does not exist in map; set previous and current info to be the same
            vel_info.joint_name = msg.name[i];
            vel_info.prev_velocity = msg.velocity[i];
            vel_info.prev_timestamp = msg.header.stamp.toSec();
            vel_info.curr_velocity = msg.velocity[i];
            vel_info.curr_timestamp = msg.header.stamp.toSec();
        }
        else { // it_vel != joint_velocities_.end()
            // joint exists in map; update previous and set current based on message
            vel_info = joint_velocities_[msg.name[i]];
            vel_info.prev_velocity = vel_info.curr_velocity;
            vel_info.prev_timestamp = vel_info.curr_timestamp;
            vel_info.curr_velocity = msg.velocity[i];
            vel_info.curr_timestamp = msg.header.stamp.toSec();
        }

        // add or update info in map
        joint_velocities_[msg.name[i]] = vel_info;

        // ***** PROCESS JOINT TORQUE INFO *****

        // initialize torque info
        JointTorqueInfo trq_info;

        // check if joint already exists in torque map
        std::map<std::string, JointTorqueInfo>::iterator it_trq = joint_torques_.find(msg.name[i]);

        // update torque info accordingly
        if( it_trq == joint_torques_.end() ) {
            // joint does not exist in map; set previous and current info to be the same
            trq_info.joint_name = msg.name[i];
            trq_info.prev_torque = msg.effort[i];
            trq_info.prev_timestamp = msg.header.stamp.toSec();
            trq_info.curr_torque = msg.effort[i];
            trq_info.curr_timestamp = msg.header.stamp.toSec();
        }
        else { // it_trq != joint_torques_.end()
            // joint exists in map; update previous and set current based on message
            trq_info = joint_torques_[msg.name[i]];
            trq_info.prev_torque = trq_info.curr_torque;
            trq_info.prev_timestamp = trq_info.curr_timestamp;
            trq_info.curr_torque = msg.effort[i];
            trq_info.curr_timestamp = msg.header.stamp.toSec();
        }

        // add or update info in map
        joint_torques_[msg.name[i]] = trq_info;
    }

    return;
}

// GETTERS/SETTERS
std::string JointStateDeltaMonitorNode::getNodeName() {
    return std::string("Joint State Delta Monitor Node");
}

// HELPERS
void JointStateDeltaMonitorNode::publishAllSoftEStopMessages() {
    // publish pause walking and stop trajectory messages
    publishPauseWalkingMessage();
    publishStopWalkingMessage();

    return;
}

void JointStateDeltaMonitorNode::publishSoftEStopReportMessage() {
    // only publish with the last round of soft e-stop messages
    if( ihmc_interface_pause_stop_msg_counter_ == 1 ) {
        // create soft e-stop report message
        val_safety_exception_reporter::SoftEStop soft_estop_report_msg;

        // add joint state delta to soft e-stop causes
        soft_estop_report_msg.soft_estop_causes.clear();
        soft_estop_report_msg.soft_estop_causes.push_back(soft_estop_report_msg.JOINT_STATE_DELTA);

        // clear out joint state delta messages
        soft_estop_report_msg.joint_state_delta_msg.clear();

        // look through set of joints that reached limits
        for( const std::string& joint_name : limited_joints_ ) {
            // get velocity and torque info
            JointVelocityInfo vel_info = joint_velocities_[joint_name];
            JointTorqueInfo trq_info = joint_torques_[joint_name];

            // create joint state delta soft e-stop message
            val_safety_exception_reporter::SoftEStopJointStateDelta j_msg;

            // set message fields
            j_msg.joint_name = joint_name;
            j_msg.prev_joint_velocity = vel_info.prev_velocity;
            j_msg.prev_joint_velocity_timestamp = vel_info.prev_timestamp;
            j_msg.curr_joint_velocity = vel_info.curr_velocity;
            j_msg.curr_joint_velocity_timestamp = vel_info.curr_timestamp;
            j_msg.velocity_delta_threshold = JOINT_VELOCITY_DELTA_LIMIT_;
            j_msg.prev_joint_torque = trq_info.prev_torque;
            j_msg.prev_joint_torque_timestamp = trq_info.prev_timestamp;
            j_msg.curr_joint_torque = trq_info.curr_torque;
            j_msg.curr_joint_torque_timestamp = trq_info.curr_timestamp;
            j_msg.torque_delta_threshold = JOINT_TORQUE_DELTA_LIMIT_;

            // add to soft e-stop message
            soft_estop_report_msg.joint_state_delta_msg.push_back(j_msg);
        }

        // publish message
        safety_reporter_pub_.publish(soft_estop_report_msg);

        // any stored info is out of date; clear everything
        limited_joints_.clear();
    }

    return;
}

// MONITOR FUNCTIONS
bool JointStateDeltaMonitorNode::checkMonitorCondition() {
    return checkVelocityTorqueDeltaLimits();
}

bool JointStateDeltaMonitorNode::checkVelocityTorqueDeltaLimits() {
    // verify some velocity/torque information has been received
    // both maps are always updated together, so if one is empty, the other will be as well
    if( joint_velocities_.empty() || joint_torques_.empty() ) {
        // no information received, no limits reached
        return false;
    }

    // initialize flag for limit
    bool limit_found = false;

    // check if monitoring velocity
    if( MONITOR_VELOCITY_ ) {
        // look through available velocity information
        for( const std::pair<std::string, JointVelocityInfo>& joint_vel : joint_velocities_ ) {
            // get joint info
            std::string joint_name = joint_vel.first;
            JointVelocityInfo vel_info = joint_vel.second;
            double vel_delta = abs(vel_info.curr_velocity - vel_info.prev_velocity);
            double vel_timestep = vel_info.curr_timestamp - vel_info.prev_timestamp;

            // print joint velocity
            if( debug_ ) {
                ROS_INFO("[%s] DEBUG MODE: Joint %s has previous velocity %f and current velocity %f; change in velocity %f over %f seconds",
                         getNodeName().c_str(), joint_name.c_str(), vel_info.prev_velocity, vel_info.curr_velocity, vel_delta, vel_timestep);
            }

            // check for timeout
            if( vel_timestep >= JOINT_VELOCITY_TIMEOUT_ ) {
                // velocity points are too far apart, do not check velocity delta
                continue;
            }

            // check for change in velocity limit
            if( vel_delta >= JOINT_VELOCITY_DELTA_LIMIT_ ) {
                // update flag
                limit_found = true;
                // add joint to set of joints at limit
                limited_joints_.insert(joint_name);
                ROS_WARN("[%s] Joint %s has change in velocity %f, which exceeds limit %f",
                         getNodeName().c_str(), joint_name.c_str(), vel_delta, JOINT_VELOCITY_DELTA_LIMIT_);
            }
        }
    }

    // check if monitoring torque
    if( MONITOR_TORQUE_ ) {
        // look through available torque information
        for( const std::pair<std::string, JointTorqueInfo>& joint_trq : joint_torques_ ) {
            // get joint info
            std::string joint_name = joint_trq.first;
            JointTorqueInfo trq_info = joint_trq.second;
            double trq_delta = abs(trq_info.curr_torque - trq_info.prev_torque);
            double trq_timestep = trq_info.curr_timestamp - trq_info.prev_timestamp;

            // print joint torque
            if( debug_ ) {
                ROS_INFO("[%s] DEBUG MODE: Joint %s has previous torque %f and current torque %f; change in torque %f over %f seconds",
                         getNodeName().c_str(), joint_name.c_str(), trq_info.prev_torque, trq_info.curr_torque, trq_delta, trq_timestep);
            }

            // check for timeout
            if( trq_timestep >= JOINT_TORQUE_TIMEOUT_ ) {
                // torque points are too far apart, do not check torque delta
                continue;
            }

            // check for change in torque limit
            if( trq_delta >= JOINT_TORQUE_DELTA_LIMIT_ ) {
                // update flag
                limit_found = true;
                // add joint to set of joints at limit
                limited_joints_.insert(joint_name);
                ROS_WARN("[%s] Joint %s has change in torque %f, which exceeds limit %f",
                         getNodeName().c_str(), joint_name.c_str(), trq_delta, JOINT_TORQUE_DELTA_LIMIT_);
            }
        }
    }

    return limit_found;
}

int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "ValkyrieJointStateDeltaMonitorNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create and initialize node
    JointStateDeltaMonitorNode joint_monitor;
    joint_monitor.initializeMonitor(nh);
    ROS_INFO("[%s] Node started!", joint_monitor.getNodeName().c_str());

    // get loop rate
    ros::Rate rate(joint_monitor.getLoopRate());

    // run node
    while( ros::ok() ) {
        // perform soft estop if necessary
        joint_monitor.performSoftEStop();

        // spin and sleep
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[%s] Node stopped, all done!", joint_monitor.getNodeName().c_str());

    return 0;
}
