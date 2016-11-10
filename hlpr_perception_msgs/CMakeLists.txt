cmake_minimum_required(VERSION 2.8.3)
project(hlpr_perception_msgs)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  message_generation
)

find_package(PCL 1.8 REQUIRED COMPONENTS)

add_message_files(
  FILES
  # Segmentation msgs
  NonPlanarSegClusters.msg
  SegClusters.msg
  SegPlanes.msg
  ClusterIndex.msg
  # Feature extraction msgs
  BasicFeatures.msg
  ShapeHist.msg
  ColorHist.msg
  OrientedBoundingBox.msg
  OtherFeatures.msg
  ViewpointHist.msg
  ObjectFeatures.msg
  PlaneFeatures.msg
  ExtractedFeaturesArray.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
)

catkin_package(
 # CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp std_msgs geometry_msgs
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)

