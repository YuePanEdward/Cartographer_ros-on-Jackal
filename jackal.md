# How to turn on those sensors on Jackal?

1. setup internet and enter Jackal remotely
       
       #connect Jackal's wifi
       ssh -X administrator@192.168.0.1
       #enter the password: clearpath

2. launch kinect2

       roslaunch kinect2_bridge kinect2_bridge.launch 
       
3. launch kvh_imu (original imu is launched automaticly while jackal power up)
           
       roslaunch kvh1750 imu.launch
       
4. launch velodyne lidar

       roslaunch velodyne_pointcloud VLP16_points.launch 
       
5. setup vicon's internet and then launch vicon (on your own computer)

       roslaunch vicon_bridge vicon.launch 


# Jackal system setting

Jackal's system setting: Once jackal is powered on, ros.launch is launched
the automatic launch tree is listed below

> ros.launch->accessories.launch

>          ->base.launch        ->description.launch(something about urdf, published by robot_state_publisher)

>                               ->control.launch(ekf_localization_node, about the sensor intergration)

>                               ->teleop.launch(connect with PS4 joystick by publishing geometry_msgs/Twist, specialfized in the robot_localization.yaml file)

According to http://docs.ros.org/melodic/api/robot_localization/html/index.html#

All the state estimation nodes in robot_localization share common features, namely:

   ### Fusion of an arbitrary number of sensors.
   The nodes do not restrict the number of input sources. If, for example, your robot has multiple IMUs or multiple sources of odometry information, the state estimation nodes within robot_localization can support all of them.
   
   ### Support for multiple ROS message types.
   All state estimation nodes in robot_localization can take in nav_msgs/Odometry, sensor_msgs/Imu, geometry_msgs/PoseWithCovarianceStamped, or geometry_msgs/TwistWithCovarianceStamped messages.
   
   ### Per-sensor input customization.
   If a given sensor message contains data that you don’t want to include in your state estimate, the state estimation nodes in robot_localization allow you to exclude that data on a per-sensor basis.
   
   ### Continuous estimation.
   Each state estimation node in robot_localization begins estimating the vehicle’s state as soon as it receives a single measurement. If there is a holiday in the sensor data (i.e., a long period in which no data is received), the filter will continue to estimate the robot’s state via an internal motion model.

   All state estimation nodes track the 15-dimensional state of the vehicle: (X,Y,Z,roll,pitch,yaw,X˙,Y˙,Z˙,roll˙,pitch˙,yaw˙,X¨,Y¨,Z¨)

