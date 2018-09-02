cmake_minimum_required(VERSION 2.8.3)
project(hlpr_color_object_labeler)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  hlpr_perception_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS geometry_msgs hlpr_perception_msgs roscpp rospy std_msgs visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

