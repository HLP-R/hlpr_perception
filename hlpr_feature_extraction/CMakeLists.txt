cmake_minimum_required(VERSION 2.8.3)
project(hlpr_feature_extraction)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed -pthread")

set(OPENNI_INCLUDE_DIRS /usr/include/ni)

#find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation roscpp std_msgs geometry_msgs)
find_package(OpenCL QUIET)
IF(OpenCL_FOUND)
	SET(LIBRARIES
		${OpenCL_LIBRARIES}
		${LIBRARIES}
	)
ENDIF(OpenCL_FOUND)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  tf2_ros
  hlpr_segmentation
  pcl_ros
)
find_package(freenect2 REQUIRED)
find_package(OpenCV 3 REQUIRED)

add_message_files(
  FILES
  PcFeatures.msg
  PcFeatureArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  hlpr_segmentation
)

#generate_dynamic_reconfigure_options(
#  cfg/segmentation.cfg
#)

catkin_package(
 # CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp std_msgs geometry_msgs
  CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs hlpr_segmentation
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${FREENECT2_INCLUDE_DIRS})
include_directories(${freenect2_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(ftex SHARED src/cluster_processing.cpp src/rotcalipers.cpp src/utils.cpp src/utils_pcl_ros.cpp)
target_link_libraries(ftex ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(ftex hlpr_feature_extraction_generate_messages_cpp)

add_executable(ft_ex src/feature_extraction.cpp)
#add_executable(listener src/listener.cpp src/nodes/listener_node.cpp)
#target_link_libraries(pc_seg ${catkin_LIBRARIES} ${freenect2_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(ft_ex ${PCL_LIBRARIES} ftex ${freenect2_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(ft_ex hlpr_feature_extraction_generate_messages_cpp)

#add_dependencies(pc_seg segmentation_gencfg segmentation_generate_messages_cpp)


#install(DIRECTORY include/${PROJECT_NAME}/
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ft_ex
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
