/**
 * Joint Monitor Node
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#include <nodes/joint_monitor_node.h>

// CONSTRUCTORS/DESTRUCTORS
JointMonitorNode::JointMonitorNode(const ros::NodeHandle& nh) {
    nh_ = nh;

    loop_rate_ = 10.0; // Hz

    ihmc_interface_pause_stop_msg_counter_ = 0;

    // initialize velocity and torque limits to some default
    JOINT_VELOCITY_LIMIT_ = 10.0;
    JOINT_TORQUE_LIMIT_   = 10.0;
    // NOTE initialization does not matter, will immediately be reconfigured to defaults set in cfg/JointMonitorParams.cfg

    // set up parameters
    nh_.param("joint_state_topic", joint_state_topic_, std::string("/ihmc_ros/valkyrie/output/joint_states"));

    // initialize connections
    initializeConnections();

    // initialize dynamic reconfigure server
    initializeDynamicReconfigureServer();

    ROS_INFO("[Joint Monitor Node] Constructed");
}

JointMonitorNode::~JointMonitorNode() {
    ROS_INFO("[Joint Monitor Node] Destroyed");
}

// CONNECTIONS
bool JointMonitorNode::initializeConnections() {
    // joint state subscriber
    joint_state_sub_ = nh_.subscribe(joint_state_topic_, 1, &JointMonitorNode::jointStateCallback, this);

    // status publisher for soft estops
    ihmc_interface_status_pub_ = nh_.advertise<std_msgs::String>("/IHMCInterfaceNode/controllers/output/ihmc/controller_status", 100);

    return true;
}

// DYNAMIC RECONFIGURE SERVER
void JointMonitorNode::initializeDynamicReconfigureServer() {
    // initialize callback type
    dynamic_reconfigure::Server<val_soft_estop_monitor::JointMonitorParamsConfig>::CallbackType f;

    // bind function to callback
    f = boost::bind(&JointMonitorNode::paramReconfigureCallback, this, _1, _2);

    // set callback for server
    reconfigure_server_.setCallback(f);

    return;
}

// CALLBACKS
void JointMonitorNode::paramReconfigureCallback(val_soft_estop_monitor::JointMonitorParamsConfig &config, uint32_t level) {
    // take params from reconfigure request and store them internally
    JOINT_VELOCITY_LIMIT_ = config.joint_velocity_limit;
    JOINT_TORQUE_LIMIT_ = config.joint_torque_limit;

    ROS_INFO("[Joint Monitor Node] Reconfigured velocity limit to %f and torque limit to %f", JOINT_VELOCITY_LIMIT_, JOINT_TORQUE_LIMIT_);

    return;
}

void JointMonitorNode::jointStateCallback(const sensor_msgs::JointState& msg) {
    // loop through joint state message
    for( int i = 0 ; i < msg.name.size() ; i++ ) {
        // update velocity and torque in respective map
        joint_velocities_[msg.name[i]] = msg.velocity[i];
        joint_torques_[msg.name[i]] = msg.effort[i];
    }

    return;
}

// GETTERS/SETTERS
double JointMonitorNode::getLoopRate() {
    return loop_rate_;
}

// HELPERS
void JointMonitorNode::initializeMessageCounter() {
    ihmc_interface_pause_stop_msg_counter_ = 5;

    return;
}

void JointMonitorNode::decrementMessageCounter() {
    ihmc_interface_pause_stop_msg_counter_--;

    return;
}

void JointMonitorNode::publishPauseWalkingMessage() {
    // create string message
    std_msgs::String str_msg;
    str_msg.data = std::string("PAUSE-WALKING");

    ihmc_interface_status_pub_.publish(str_msg);

    ros::spinOnce(); // make sure messages go through

    return;
}

void JointMonitorNode::publishStopWalkingMessage() {
    // create string message
    std_msgs::String str_msg;
    str_msg.data = std::string("STOP-ALL-TRAJECTORY");

    ihmc_interface_status_pub_.publish(str_msg);

    ros::spinOnce(); // make sure messages go through

    return;
}

// MONITOR FUNCTIONS
bool JointMonitorNode::checkVelocityTorqueLimits() {
    // verify some velocity/torque information has been received
    // both maps are always updated together, so if one is empty, the other will be as well
    if( joint_velocities_.empty() || joint_torques_.empty() ) {
        // no information received, no limits reached
        return false;
    }

    // initialize flag for limit
    bool limit_found = false;

    // look through available velocity information
    for( const std::pair<std::string, double>& joint_vel : joint_velocities_ ) {
        // check for velocity limit
        if( abs(joint_vel.second) >= JOINT_VELOCITY_LIMIT_ ) {
            // update flag
            limit_found = true;
            ROS_WARN("[Joint Monitor Node] Joint %s has velocity %f, which exceeds limit %f",
                     joint_vel.first.c_str(), joint_vel.second, JOINT_VELOCITY_LIMIT_);
        }
    }

    // look through available torque information
    for( const std::pair<std::string, double>& joint_trq : joint_torques_ ) {
        // check for torque limit
        if( abs(joint_trq.second) >= JOINT_TORQUE_LIMIT_ ) {
            // update flag
            limit_found = true;
            ROS_WARN("[Joint Monitor Node] Joint %s has torque %f, which exceeds limit %f",
                     joint_trq.first.c_str(), joint_trq.second, JOINT_TORQUE_LIMIT_);
        }
    }

    return limit_found;
}

void JointMonitorNode::performSoftEStop() {
    // check for velocity and torque limits
    if( checkVelocityTorqueLimits() ) {
        // check if limit already detected
        if( ihmc_interface_pause_stop_msg_counter_ == 0) {
            // set message counter
            initializeMessageCounter();
            ROS_WARN("[Joint Monitor Node] Found velocity or torque limit! Performing soft e-stop!");
        }
        // otherwise, limit has already been detected and pause/stop messages are being sent
    }

    // check if messages need to be published for soft estop
    if( ihmc_interface_pause_stop_msg_counter_ > 0 ) {
        // publish pause walking and stop trajectory messages
        publishPauseWalkingMessage();
        publishStopWalkingMessage();
        // decrement counter
        decrementMessageCounter();
    }

    return;
}

int main(int argc, char **argv) {
    // initialize node
    ros::init(argc, argv, "ValkyrieJointMonitorNode");

    // initialize node handler
    ros::NodeHandle nh("~");

    // create node
    JointMonitorNode joint_monitor(nh);
    ROS_INFO("[Joint Monitor Node] Node started!");

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

    ROS_INFO("[Joint Monitor Node] Node stopped, all done!");

    return 0;
}
