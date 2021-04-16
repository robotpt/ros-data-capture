cmake_minimum_required(VERSION 2.8.3)
project(mem_use_watcher)

find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_python_setup()
catkin_package()

install(DIRECTORY scripts src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
