# Valkyrie Soft E-Stop Monitor
Monitors for detecting conditions under which we would want to soft e-stop NASA's Valkyrie robot.

This package contains nodes that monitor conditions (such as joint velocities, joint torques, and distance between actual and desired end-effector poses) under which we may want to perform a soft e-stop on the robot.  Soft e-stops are performed by stopping execution of all non-walking trajectories and safely pausing walking by returning the robot to double support.  The package depends on `dynamic_reconfigure`, [IHMC's `controller_msgs`](https://github.com/ihmcrobotics/ihmc-open-robotics-software/tree/val-develop/ihmc-interfaces/src/main/messages/ros1/controller_msgs/msg), and [`val_vr_ros`](https://js-er-code.jsc.nasa.gov/vs/val_vr_ros).  This package also performs soft e-stops by sending messages to the [IHMC Message Interface](https://github.com/esheetz/IHMCMsgInterface).

To launch the monitors, run:
```
roslaunch ihmc_msg_interface ihmc_interface_node.launch
roslaunch val_soft_estop_monitor monitors.launch
```

The `monitors.launch` file launches all monitor nodes and `rqt_reconfigure` so parameters can be dynamically reconfigured.  Currently there are monitors for:
- Joint velocities and joint torques over some limit.  Can dynamically reconfigure the velocity limit and the torque limit.
- Distances between desired and actual end-effector poses over some distance threshold.  Can dynamically reconfigure the position and rotation limits for both the hands and head.
