cmake_minimum_required(VERSION 2.8.3)
project(hlpr_single_plane_segmentation)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed")

set(OPENNI_INCLUDE_DIRS /usr/include/ni)
set(ROS_DISTRO indigo)
set(ROS_LIBRARIES /opt/ros/${ROS_DISTRO}/lib/libroscpp.so 
        /opt/ros/${ROS_DISTRO}/lib/libroscpp_serialization.so 
        /opt/ros/${ROS_DISTRO}/lib/librosconsole.so     
        /opt/ros/${ROS_DISTRO}/lib/libroslib.so
    /opt/ros/${ROS_DISTRO}/lib/librostime.so)

#find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation roscpp std_msgs geometry_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation roscpp std_msgs geometry_msgs sensor_msgs hlpr_perception_msgs)
find_package(PCL 1.8 REQUIRED COMPONENTS)
find_package(freenect2 REQUIRED)
find_package(OpenCV REQUIRED)

#add_message_files(
#  FILES
#  SegmentedClusters.msg
#  ClusterIndex.msg
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

#generate_dynamic_reconfigure_options(
#  cfg/segmentation.cfg
#)

catkin_package(
 # CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp std_msgs geometry_msgs
 # CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs sensor_msgs
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${FREENECT2_INCLUDE_DIRS})
include_directories(${freenect2_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(sgseg SHARED src/pc_segmentation.cpp src/utils.cpp)
target_link_libraries(sgseg ${PCL_LIBRARIES} ${ROS_LIBRARIES})

add_executable(sg_seg src/segmentation.cpp)
target_link_libraries(sg_seg ${PCL_LIBRARIES} sgseg ${freenect2_LIBRARY} ${ROS_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(sg_seg hlpr_perception_msgs_generate_messages_cpp)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS sg_seg
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
