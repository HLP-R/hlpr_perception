cmake_minimum_required(VERSION 2.8.3)
project(hlpr_object_labeling)

find_package(catkin REQUIRED COMPONENTS message_generation rospy std_msgs geometry_msgs sensor_msgs hlpr_feature_extraction)

#catkin_python_setup()

add_message_files(
  FILES
  LabeledObjects.msg
)

## Generate added messages and services with any dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  hlpr_feature_extraction
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs hlpr_feature_extraction
)


