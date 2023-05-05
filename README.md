# orb_slam2-CARLA
ROS2 node that runs ORB SLAM2 in CARLA Simulator using ROS

### Requirements

 - [ROS2 Foxy](https://github.com/ros2/ros2/wiki/Installation)
 - [Pangolin](https://github.com/stevenlovegrove/Pangolin)
 - [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2)
 - [OpenCV3](https://docs.opencv.org/3.0-beta/doc/tutorials/introduction/linux_install/linux_install.html)
 - [message_filters](https://github.com/ros2/message_filters)
 
 ### Usage
 Add to the LD_LIBRARY_PATH the location of Pangolin
 
    $ LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/Pangolin/build/
    $ export LD_LIBRARY_PATH
    
 After telling CMake where to find the ORB_SLAM2 directory, build the package in a ROS2 workspace with
 
    $ colcon build
   
 After launching the CARLA server with
 
    $ ./CarlaUE4.sh
    
 Launch the environment created by the CARLA-ROS-Bridge with
 
    $ ros2 launch orb_slam2-CARLA slam_launch.py

 Run ORB_SLAM in stereo mode:
 
    $ ros2 run orb_node stereo-node PATH_TO_VOCABULARY PATH_TO_YAML_CONFIG_FILE BOOL_RECTIFY
