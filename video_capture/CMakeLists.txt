cmake_minimum_required(VERSION 2.8.3)
project(video_capture)
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
)

catkin_python_setup()
catkin_package()
