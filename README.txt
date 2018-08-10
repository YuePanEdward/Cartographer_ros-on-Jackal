# Carographer_ros-on-Jackal
# Documnet for parameter tuning

------------------------------------------------------------------------------------------------------
Prepare:
Robot: Jackal (Clearpath) 
Lidar: Velodyne VLP16
IMU:   KVH1750
Computer: Desktop/Laptop i5+ Ubuntu 14.04 with ROS Indigo
------------------------------------------------------------------------------------------------------
Install Cartographer











-------------------------------------------------------------------------------------------------------
Tuning parameters
For 2D Cartographer

1.Frames
    map_frame = "map",   
    --this one is fixed
    
    tracking_frame = "kvh_link", 
    -- when using imu,you must use the imu's link. It's kvh_link here.
    -- for 2d cartographer, imu is not neccessary. But from my perspective, you need to use kvh1750 imu or the map would be a mess.
    -- for 3d cartographer, imu is neccessary.
    
    published_frame = "base_link", 
    -- On jackal, odom should be ok. I don't know why. 
    -- But for rosbag, use base_link as other frames are based on baselink
     
    odom_frame = "odom",
    provide_odom_frame = false, 
    use_odometry = true,
    -- If you want to use the odometry provided by the robot, then provide_odom_frame should be false.
    -- odom_frame should be odom, in jackal, as the rostopic of ekf result is called /odometery/filtered, you need to remap it to odom in the launch file
    
    publish_frame_projected_to_2d = false, 
    use_nav_sat = false,
    use_landmarks = false,
    -- the second choice has some problem, which leads to the map frame seems like base_link
  
    num_laser_scans = 0,
    num_multi_echo_laser_scans = 0,
    num_subdivisions_per_laser_scan = 1, 
    -- try changing this one and see what will happen, seems like nothing happened
    -- Originally, it's set to be 10.
   
    num_point_clouds = 1,
    lookup_transform_timeout_sec = 0.2,  --0.2
    submap_publish_period_sec = 0.3,     --0.3 
    pose_publish_period_sec = 5e-3,      --5e-3
    trajectory_publish_period_sec = 30e-3, --20e-3
    rangefinder_sampling_ratio = 1.,
    odometry_sampling_ratio = 0.1, --0.5 or you will meet some check failed:data.time>std::prev error but I don't know why.
    fixed_frame_pose_sampling_ratio = 1., --0.5
    imu_sampling_ratio = 1.,
    landmarks_sampling_ratio = 1.,




For 3D Cartographer

