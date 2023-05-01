/**
 * Joint State Monitor Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#include <nodes/joint_state_monitor_node.h>

// CONSTRUCTORS/DESTRUCTORS
JointStateMonitorNode::JointStateMonitorNode() {
    // initialize velocity and torque limits to some default
    JOINT_VELOCITY_LIMIT_ = 10.0;
    JOINT_TORQUE_LIMIT_   = 10.0;
    MONITOR_VELOCITY_ = true;
    MONITOR_TORQUE_ = true;
    // NOTE initialization does not matter, will immediately be reconfigured to defaults set in cfg/JointStateMonitorParams.cfg

    ROS_INFO("[%s] Constructed", getNodeName().c_str());
}

JointStateMonitorNode::~JointStateMonitorNode() {
    ROS_INFO("[%s] Destroyed", getNodeName().c_str());
}

// INITIALIZATION
void JointStateMonitorNode::initializeMonitor(const ros::NodeHandle& nh) {
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
bool JointStateMonitorNode::initializeConnections() {
    GenericMonitor::initializeConnections();

    // joint state subscriber
    joint_state_sub_ = nh_.subscribe(joint_state_topic_, 1, &JointStateMonitorNode::jointStateCallback, this);

    // safety reporter publisher
    safety_reporter_pub_ = nh_.advertise<val_safety_exception_reporter::SoftEStop>("/valkyrie_safety_reporter/soft_estop", 10);

    return true;
}

// DYNAMIC RECONFIGURE SERVER
void JointStateMonitorNode::initializeDynamicReconfigureServer() {
    // initialize callback type
    dynamic_reconfigure::Server<val_soft_estop_monitor::JointStateMonitorParamsConfig>::CallbackType f;

    // bind function to callback
    f = boost::bind(&JointStateMonitorNode::paramReconfigureCallback, this, _1, _2);

    // set callback for server
    reconfigure_server_.setCallback(f);

    return;
}

// CALLBACKS
void JointStateMonitorNode::paramReconfigureCallback(val_soft_estop_monitor::JointStateMonitorParamsConfig &config, uint32_t level) {
    // take params from reconfigure request and store them internally
    debug_ = config.debug;
    MONITOR_VELOCITY_ = config.monitor_velocity;
    JOINT_VELOCITY_LIMIT_ = config.joint_velocity_limit;
    MONITOR_TORQUE_ = config.monitor_torque;
    JOINT_TORQUE_LIMIT_ = config.joint_torque_limit;

    if( debug_ ) {
        ROS_INFO("[%s] ENTERED DEBUG MODE", getNodeName().c_str());
    }

    // velocity monitor
    if( !MONITOR_VELOCITY_ ) {
        ROS_INFO("[%s] NOT MONITORING VELOCITY", getNodeName().c_str());
    }
    else {
        ROS_INFO("[%s] Reconfigured velocity limit to %f",
                 getNodeName().c_str(), JOINT_VELOCITY_LIMIT_);
    }

    // torque monitor
    if( !MONITOR_TORQUE_ ) {
        ROS_INFO("[%s] NOT MONITORING TORQUE", getNodeName().c_str());
    }
    else {
        ROS_INFO("[%s] Reconfigured torque limit to %f",
                 getNodeName().c_str(), JOINT_TORQUE_LIMIT_);
    }

    // ROS_INFO("[%s] Reconfigured velocity limit to %f and torque limit to %f",
    //          getNodeName().c_str(), JOINT_VELOCITY_LIMIT_, JOINT_TORQUE_LIMIT_);

    return;
}

void JointStateMonitorNode::jointStateCallback(const sensor_msgs::JointState& msg) {
    // loop through joint state message
    for( int i = 0 ; i < msg.name.size() ; i++ ) {
        // update velocity and torque in respective map
        joint_velocities_[msg.name[i]] = msg.velocity[i];
        joint_torques_[msg.name[i]] = msg.effort[i];
    }

    return;
}

// GETTERS/SETTERS
std::string JointStateMonitorNode::getNodeName() {
    return std::string("Joint State Monitor Node");
}

// HELPERS
void JointStateMonitorNode::publishAllSoftEStopMessages() {
    // publish pause walking and stop trajectory messages
    publishPauseWalkingMessage();
    publishStopWalkingMessage();

    return;
}

void JointStateMonitorNode::publishSoftEStopReportMessage() {
    // only publish with the last round of soft e-stop messages
    if( ihmc_interface_pause_stop_msg_counter_ == 1 ) {
        // create soft e-stop report message
        val_safety_exception_reporter::SoftEStop soft_estop_report_msg;

        // add joint state to soft e-stop causes
        soft_estop_report_msg.soft_estop_causes.clear();
        soft_estop_report_msg.soft_estop_causes.push_back(soft_estop_report_msg.JOINT_STATE);

        // clear out joint state messages
        soft_estop_report_msg.joint_state_msg.clear();

        // look through set of joints that reached limits
        for( const std::string& joint_name : limited_joints_ ) {
            // get velocity and torque info
            double joint_vel = joint_velocities_[joint_name];
            double joint_trq = joint_torques_[joint_name];

            // create joint state soft e-stop message
            val_safety_exception_reporter::SoftEStopJointState j_msg;

            // set message fields
            j_msg.joint_name = joint_name;
            j_msg.joint_velocity = joint_vel;
            j_msg.velocity_threshold = JOINT_VELOCITY_LIMIT_;
            j_msg.joint_torque = joint_trq;
            j_msg.torque_threshold = JOINT_TORQUE_LIMIT_;

            // add to soft e-stop message
            soft_estop_report_msg.joint_state_msg.push_back(j_msg);
        }

        // publish message
        safety_reporter_pub_.publish(soft_estop_report_msg);

        // any stored info is out of date; clear everything
        limited_joints_.clear();
    }

    return;
}

// MONITOR FUNCTIONS
bool JointStateMonitorNode::checkMonitorCondition() {
    return checkVelocityTorqueLimits();
}

bool JointStateMonitorNode::checkVelocityTorqueLimits() {
    // verify some velocity/torque information has been received
    // both maps are always updated together, so if one is empty, the other will be as well
    if( joint_velocities_.empty() || joint_torques_.empty() ) {
        // no information received, no limits reached
        return false;
    }

    // initialize flag for limit
    bool limit_found = false;

    // check if monitoring velocity
    if( MONITOR_VELOCITY_) {
        // look through available velocity information
        for( const std::pair<std::string, double>& joint_vel : joint_velocities_ ) {
            // print joint velocity
            if( debug_ ) {
                ROS_INFO("[%s] DEBUG MODE: Joint %s has velocity %f",
                         getNodeName().c_str(), joint_vel.first.c_str(), joint_vel.second);
            }

            // check for velocity limit
            if( abs(joint_vel.second) >= JOINT_VELOCITY_LIMIT_ ) {
                // update flag
                limit_found = true;
                // add joint to set of joints at limit
                limited_joints_.insert(joint_vel.first);
                ROS_WARN("[%s] Joint %s has velocity %f, which exceeds limit %f",
                         getNodeName().c_str(), joint_vel.first.c_str(), joint_vel.second, JOINT_VELOCITY_LIMIT_);
            }
        }
    }

    // check if monitoring torque
    if( MONITOR_TORQUE_ ) {
        // look through available torque information
        for( const std::pair<std::string, double>& joint_trq : joint_torques_ ) {
            // print joint torque
            if( debug_ ) {
                ROS_INFO("[%s] DEBUG MODE: Joint %s has torque %f",
                         getNodeName().c_str(), joint_trq.first.c_str(), joint_trq.second);
            }

            // check for torque limit
            if( abs(joint_trq.second) >= JOINT_TORQUE_LIMIT_ ) {
                // update flag
                limit_found = true;
                // add joint to set of joints at limit
                limited_joints_.insert(joint_trq.first);
                ROS_WARN("[%s] Joint %s has torque %f, which exceeds limit %f",
                         getNodeName().c_str(), joint_trq.first.c_str(), joint_trq.second, JOINT_TORQUE_LIMIT_);
            }
        }
    }

    return limit_found;
}

int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "ValkyrieJointStateMonitorNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create and initialize node
    JointStateMonitorNode joint_monitor;
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
