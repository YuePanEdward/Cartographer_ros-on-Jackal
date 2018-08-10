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

There are three packages to be install : ceres slover (a graph optimizer provided by google) cartographer and cartographer_ros

1. Install all the dependency
sudo apt-get install -y google-mock libboost-all-dev  libeigen3-dev libgflags-dev libgoogle-glog-dev liblua5.2-dev libprotobuf-dev  libsuitesparse-dev libwebp-dev ninja-build protobuf-compiler python-sphinx  ros-indigo-tf2-eigen libatlas-base-dev libsuitesparse-dev liblapack-dev

   Method and codes provided by hitcm (a phd student in HIT)
2. Install ceres slover 1.11
 git clone https://github.com/hitcm/ceres-solver-1.11.0.git
 cd ceres-solver-1.11.0/build
 cmake ..
 make –j
 sudo make install
 
3.Install cartographer
 git clone https://github.com/hitcm/cartographer.git
 cd cartographer/build
 cmake .. -G Ninja
 ninja
 ninja test
 sudo ninja install
 
4.Install cartographer_ros 
 git clone https://github.com/hitcm/cartographer_ros.git 
 cd ~/catkin_ws
 catkin_make

 Another method provided by Google officially
  Refer to this website: cartographer   http://google-cartographer.readthedocs.io/en/latest/index.html
                         cartographer_ros   https://google-cartographer-ros.readthedocs.io/en/latest/

Building &Installation of it 

# Install wstool and rosdep.
sudo apt-get update
sudo apt-get install -y python-wstool python-rosdep ninja-build

# Create a new workspace in 'catkin_ws'.
mkdir catkin_ws
cd catkin_ws
wstool init src

# Merge the cartographer_ros.rosinstall file and fetch code for dependencies.
wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src

# Install proto3.
src/cartographer/scripts/install_proto3.sh

# Install deb dependencies.
# The command 'sudo rosdep init' will print an error if you have already
# executed it since installing ROS. This error can be ignored.
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Build and install.
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash


-------------------------------------------------------------------------------------------------------
Tuning parameters
Some of this part should refer to offical readme file provided by Google:  https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html

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
    
    -- Option 1 (Recommended) If you want to use the odometry provided by the robot, then provide_odom_frame should be false and use_odometry = true,
    -- odom_frame should be odom, in jackal, as the rostopic of ekf result is called /odometery/filtered, you need to remap it to odom in the launch file
    odom_frame = "odom",
    provide_odom_frame = false, 
    use_odometry = true,
    
    -- Option 2 (Not Recommended) Use the odom provided by cartographer (the laser odometry), then provide_odom_frame should be true and use_odometry= false,
    -- the second choice has some problem, which leads to the map frame seems like base_link. However, I don't know why.
    -- odom_frame = "odom", (Not required)
    provide_odom_frame = true, 
    use_odometry = false,
    
    -- These settings can be keep as the original one
    publish_frame_projected_to_2d = false, 
    use_nav_sat = false,
    use_landmarks = false,
    
2.Sensor setting
    
    num_laser_scans = 0, -- If you'd like to use laser (projected point cloud to a 2d plane), then this one should be 1. 
    
    num_multi_echo_laser_scans = 0, -- We do not use multi-echo laser
    
    num_subdivisions_per_laser_scan = 1, 
    -- try changing this one and see what will happen, seems like nothing happened
    -- Originally, it's set to be 10.
   
    num_point_clouds = 1, -- If you use point cloud, then it should be 1. Or it should be 0.
    
    lookup_transform_timeout_sec = 0.2,  --Originally 0.2. A waiting time threshold for tf transform error
    submap_publish_period_sec = 0.3,     --Originally 0.3
    pose_publish_period_sec = 5e-3,      --Originally 5e-3
    trajectory_publish_period_sec = 30e-3, --Originally 20e-3
    rangefinder_sampling_ratio = 1.,
    odometry_sampling_ratio = 0.1, --Originally 0.5.
    --In my opinion, You need to set it to be 0.1 or you will meet some check failed:data.time>std::prev error but I don't know why.
    fixed_frame_pose_sampling_ratio = 1., --0.5
    imu_sampling_ratio = 1.,
    landmarks_sampling_ratio = 1.,


3. SLAM paramter tuning

Two lua files are included, 
include "map_builder.lua"
include "trajectory_builder.lua"
They act as the default setting of parameters, if you want to change some of them, you can specify them downward.

  MAP_BUILDER.use_trajectory_builder_2d = true  -- for 2d cartographer
  MAP_BUILDER.num_background_threads = 6 -- Increase up to number of cores of your computer

  TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 --Originally 10  
  --Set a small number.
  --Very Important to solve the problem of map messing up when rotating
  
  TRAJECTORY_BUILDER.pure_localization=false -- If you've already build the map, you can use this pure_localiztion mode, which may give you better result of pose.
  
  TRAJECTORY_BUILDER_2D.use_imu_data = true --important
  -- If this one is true ,then you need to set the track frame as imu (it's kvh_link in our case). Besides, ium is not neccessary for 2d slam but neccessary for 3d cases
   
  ------------------SCAN MATCHER (FRONT END----LOCAL MAP----Lidar Odometry)
 
 --There are two kind of scan matcher solution.
 One is CSM (Correlated Scan Matching 2D: First, do the gridding. Then a gaussian score field can be calculated. Finally, a transformation resulting in the highest score is regarded as the optimal pose transformation in this case) .
 Another one is the optimizer Ceres given by Google. Ceres is set as default
 
 -------CSM parameter
  --TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false --true
  --TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
  --TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
  --TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10.
  
  ------Ceres parameter
  TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10 ---10 originally
  TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight =40     ---40 originally 
  -- Two important paramters
  -- Determine how the map would be robust to translation and rotation.
  -- The larger these parameters are, the more likely cartographer would avoid new submaps being added at translation/rotation
  
  --POSE GRAPH problem, refer to Cartographer tuning file 
  POSE_GRAPH.optimization_problem.huber_scale = 50 --10 originally
  POSE_GRAPH.optimization_problem.odometry_rotation_weight= 0 
  -- Set it to be 0 for trusting only the laser odometery during rotation. This is due to the poor performance of wheeled odom and imu for rotation.
  
  
  -----------------TUNE THESE PARAMETERS FOR LOW LATENCY-------------------------------
  
  ------------Global SLAM (Graph optimization by Ceres, BACK END)------------
  POSE_GRAPH.optimize_every_n_nodes = 320 -- Decrease --original 1 | 90 (it should not be too large or there will be a lot of problem)
  POSE_GRAPH.global_sampling_ratio = 0.0001 -- Decrease 0.00001 originally (X)
  POSE_GRAPH.constraint_builder.sampling_ratio = 0.03 -- Decrease 0.0001 (X)
  POSE_GRAPH.constraint_builder.min_score = 0.62 -- Increase 0.8 
  --POSE_GRAPH.global_constraint_search_after_n_seconds = 20 -- Increase ,orginally 20 (X)
  POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
  POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66
  --TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 5 -- Decrease
  
  ---------Global/Local SLAM(FIND LOOP CLOSURE)---------
  --TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100 -- Decrease
  --TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 10. -- Decrease
  --TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 1.0 -- Increase
  --TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 40 -- Decrease  originally 50
  --TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 10. -- Decrease
  --TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 2.0 -- Increase 1.8
  --TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05 -- Increase
  --TRAJECTORY_BUILDER_2D.submaps.resolution = 0.05 -- Increase
  TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80 -- Decrease   (important,control submap number)
  --TRAJECTORY_BUILDER_2D.max_range = 15. -- Decrease
  
  -------------------------------------------------------------------------------------
  
  return options
  -----you need to visualize it through rviz on a ubuntu14.04 indigo system or the typename of trajectory or other arrays would mess up



   For 3D Cartographer
