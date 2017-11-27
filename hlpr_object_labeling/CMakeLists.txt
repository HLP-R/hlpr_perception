cmake_minimum_required(VERSION 2.8.3)
project(hlpr_object_labeling)

find_package(catkin REQUIRED COMPONENTS message_generation rospy std_msgs geometry_msgs sensor_msgs hlpr_perception_msgs)

#catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs hlpr_perception_msgs
)


