cmake_minimum_required(VERSION 2.8.3)
project(audio_capture2)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  audio_capture
)

catkin_package()
