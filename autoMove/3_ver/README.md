### Optional, remove directory ~/catkin_ws/src/knu_ros_lecture and rebuild
##### :.../catkin_ws/src$ rm -rf ./knu_ros_lecture
##### :.../catkin_ws/build$ make

### 1st, make new package
##### :.../catkin_ws/src$ catkin_create_pkg robot_manager sensor_msgs cv_bridge roscpp std_msgs image_transport

### 2nd, then get this ver3's files in this package
