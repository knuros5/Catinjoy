### first, remove directory ~/catkin_ws/src/knu_ros_lecture and rebuild
##### :.../catkin_ws/src$ rm -rf ./knu_ros_lecture
##### :.../catkin_ws/build$ make
### second, make new package
##### :.../catkin_ws/src$ catkin_create_pkg knu_ros_lecture sensor_msgs cv_bridge roscpp std_msgs image_transport
### third, then get this ver2's files in this package
