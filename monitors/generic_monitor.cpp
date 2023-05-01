/**
 * Generic Monitor
 * Emily Sheetz, NSTGRO VTE 2023
 **/

#include <monitors/generic_monitor.h>

// CONSTRUCTORS/DESTRUCTORS
GenericMonitor::GenericMonitor() {
    loop_rate_ = 10.0; // Hz

    ihmc_interface_pause_stop_msg_counter_ = 0;

    debug_ = false;

    ROS_INFO("[Generic Monitor] Constructed");
}

GenericMonitor::~GenericMonitor() {
    ROS_INFO("[Generic Monitor] Destroyed");
}

// INITIALIZATION
void GenericMonitor::initializeMonitor(const ros::NodeHandle& nh) {
    nh_ = nh;

    initializeConnections();

    ROS_INFO("[Generic Monitor] Initialized");

    return;
}

// CONNECTIONS
bool GenericMonitor::initializeConnections() {
    // status publisher for soft estops
    ihmc_interface_status_pub_ = nh_.advertise<std_msgs::String>("/IHMCInterfaceNode/controllers/output/ihmc/controller_status", 100);

    return true;
}

// GETTERS/SETTERS
double GenericMonitor::getLoopRate() {
    return loop_rate_;
}

// HELPERS
double GenericMonitor::computePositionDistance(geometry_msgs::Point pos1, geometry_msgs::Point pos2) {
    // initialize Euclidean distance
    double dist = 0.0;

    // sum squared coordinate differences
    dist += std::pow((pos1.x - pos2.x), 2);
    dist += std::pow((pos1.y - pos2.y), 2);
    dist += std::pow((pos1.z - pos2.z), 2);

    // square root
    dist = sqrt(dist);

    return dist;
}

double GenericMonitor::computeRotationDistance(geometry_msgs::Quaternion quat1, geometry_msgs::Quaternion quat2) {
    /*
     * NOTE: rotation distance here is computed as the angular distance between given quaternions;
     *       put another way, the rotation distance is the angle part of the axis-angle difference quaternion
     * see resources:
     *      http://www.boris-belousov.net/2016/12/01/quat-dist/
     *      https://www.mathworks.com/help/fusion/ref/quaternion.dist.html
     *
     * to summarize, the angular distance between two quaternions p and q is:
     *      theta = 2arccos(abs(<p, q*>))
     * where:
     *      (.)* is the inverse/conjugate of a unit quaternion and
     *      <p,q> is the dot/inner product of two quaternions
     */

    // convert from geometry_msgs::Quaternion to Eigen quaternion
    Eigen::Quaternion<double> quat_1(quat1.w, quat1.x, quat1.y, quat1.z);
    Eigen::Quaternion<double> quat_2(quat2.w, quat2.x, quat2.y, quat2.z);

    // compute inverse of quaternion
    Eigen::Quaternion<double> quat_2inv = quat_2.inverse();

    // compute dot product of quaternions
    double dot = (quat_1.x() * quat_2inv.x()) +
                 (quat_1.y() * quat_2inv.y()) +
                 (quat_1.z() * quat_2inv.z()) +
                 (quat_1.w() * quat_2inv.w());

    // compute angular distance
    double dist = 2 * acos(abs(dot));

    return dist;
}

bool GenericMonitor::lookupEEWorldPose(std::string ee_name, geometry_msgs::Pose& ee_pose) {
    // initialize error message and world transform
    std::string world_frame = std::string("world");
    std::string err_msg;
    tf::StampedTransform ee_world_transform;

    try {
        // check if transform exists
        if( !tf_.waitForTransform(world_frame, ee_name, ros::Time(0), ros::Duration(1.0), // wait for transform of end-effector in world frame
                                  ros::Duration(0.01), &err_msg) ) { // default polling sleep duration
            ROS_ERROR("[%s] No transform of %s in %s frame", getNodeName().c_str(), ee_name.c_str(), world_frame.c_str());
            ROS_ERROR("[%s] Transform error: %s", getNodeName().c_str(), err_msg.c_str());
            return false;
        }
        else {
            // transform exists
            tf_.lookupTransform(world_frame, ee_name, ros::Time(0), ee_world_transform);
        }
    }
    catch( tf::TransformException ex ) {
        ROS_ERROR("[%s] Trouble getting transform of %s in %s frame", getNodeName().c_str(), ee_name.c_str(), world_frame.c_str());
        ROS_ERROR("[%s] Transform exception: %s", getNodeName().c_str(), ex.what());
        return false;
    }

    // get position and quaternion from transform
    tf::Vector3 transform_pos = ee_world_transform.getOrigin();
    tf::Quaternion transform_quat = ee_world_transform.getRotation();

    // set transformed pose
    ee_pose.position.x = transform_pos.getX();
    ee_pose.position.y = transform_pos.getY();
    ee_pose.position.z = transform_pos.getZ();
    ee_pose.orientation.x = transform_quat.x();
    ee_pose.orientation.y = transform_quat.y();
    ee_pose.orientation.z = transform_quat.z();
    ee_pose.orientation.w = transform_quat.w();

    return true;
}

void GenericMonitor::initializeMessageCounter() {
    ihmc_interface_pause_stop_msg_counter_ = 5;

    return;
}

void GenericMonitor::decrementMessageCounter() {
    ihmc_interface_pause_stop_msg_counter_--;

    return;
}

void GenericMonitor::publishPauseWalkingMessage() {
    // create string message
    std_msgs::String str_msg;
    str_msg.data = std::string("PAUSE-WALKING");

    ihmc_interface_status_pub_.publish(str_msg);

    ros::spinOnce(); // make sure messages go through

    return;
}

void GenericMonitor::publishStopWalkingMessage() {
    // create string message
    std_msgs::String str_msg;
    str_msg.data = std::string("STOP-ALL-TRAJECTORY");

    ihmc_interface_status_pub_.publish(str_msg);

    ros::spinOnce(); // make sure messages go through

    return;
}

// MONITOR FUNCTIONS
void GenericMonitor::performSoftEStop() {
    // check for velocity and torque limits
    if( checkMonitorCondition() ) {
        // check if limit already detected
        if( ihmc_interface_pause_stop_msg_counter_ == 0) {
            // set message counter
            initializeMessageCounter();
            ROS_WARN("[%s] Performing soft e-stop!", getNodeName().c_str());
        }
        // otherwise, limit has already been detected and pause/stop messages are being sent
    }

    // check if messages need to be published for soft estop
    if( ihmc_interface_pause_stop_msg_counter_ > 0 ) {
        // publish necessary messages to perform soft e-stop
        publishAllSoftEStopMessages();
        // publish message for safety reporter
        publishSoftEStopReportMessage();
        // decrement counter
        decrementMessageCounter();
    }

    return;
}
