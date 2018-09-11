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
       
5. launch vicon (on your own computer)

       roslaunch vicon_bridge vicon.launch 

you also need to setup internet 

