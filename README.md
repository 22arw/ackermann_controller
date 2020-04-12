# Overview

This package is a movidified version of the original ackermann_controller package implemented on ROS Kinetic
The new Ackermann_Controller package has been tested on Melodic and modified to function correctly

Also included in this package are config files.
ackermann_config.yaml is the config file that is read at runtime and used to designate what joints are used
It would be nice in the future to migrate this config file elsewhere so it can be dynamically called at runtime

This package contains a [ros_control](http://wiki.ros.org/ros_control) controller for car-like vehicles.
This controller is based on the [diff_drive_controller](http://wiki.ros.org/diff_drive_controller) controller.

As to what this package does, AckermannController is a class that takes in a hardware interface and nodeHandles
It then parses the ackermann_config file and finds the following information
spinning_joints:    These are joints that provide propulsion
odometry_joints:    These are joints that provide position feedback over time to tell where the vehicle is currently at
steering_joints:    These are joints that steer the vehicle

In order to use this package, you need to instantiate an AckermannController using a robotHW object
You need to provide the controller with the necessary config file
You need to initialize the controller using init and this is where the RobotHW file is passed into the control
You need to start the controller using the starting function provided in the class