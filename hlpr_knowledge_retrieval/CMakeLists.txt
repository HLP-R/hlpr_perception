cmake_minimum_required(VERSION 2.8.3)
project(hlpr_knowledge_retrieval)

find_package(catkin REQUIRED COMPONENTS message_generation rospy std_msgs geometry_msgs sensor_msgs feature_extraction)

add_message_files(
  FILES  # e.g. Floats.msg HeaderString.msg
  #SegmentedClusters.msg
  ObjectKnowledge.msg
)

## Generate added messages and services with any dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs feature_extraction
)

