#pragma once

#ifndef UTILS_PCL_ROS_HPP_
#define UTILS_PCL_ROS_HPP_

#include<utils.hpp>
#include<vector>
#include<objectFeatures.hpp>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <hlpr_perception_msgs/ViewpointHist.h>
#include <hlpr_perception_msgs/OtherFeatures.h>
#include <hlpr_perception_msgs/BasicFeatures.h>
#include <hlpr_perception_msgs/OrientedBoundingBox.h>
#include <hlpr_perception_msgs/ShapeHist.h>
#include <hlpr_perception_msgs/ColorHist.h>
#include <hlpr_perception_msgs/ObjectFeatures.h>
#include <hlpr_perception_msgs/PlaneFeatures.h>

/*
 * Ties are resolved by the cluster size
 */
int findFeatVecByHue(std::vector<pc_cluster_features> &feats, double main_hue);

int getClusterByColor(std::vector<pc_cluster_features> &feats, int feature_number, bool is_max = true);

void getClusterByColorThresh(std::vector<pc_cluster_features> &feats, int feature_number, std::vector<int > &indices, unsigned char low = 0, unsigned char high = 255);

// Some useful functions
void
Box3D_2_MC(Box3D &in_box, pcl::ModelCoefficients &out_cube);

void
minAreaRect(pcl::PointCloud<PointT>::Ptr &input_cloud, pcl::ModelCoefficients &out_cube);

void
Cube_2_Arrows(pcl::ModelCoefficients &cube, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int index);

//Box3D
//boundingBoxWithCoeff(pcl::PointCloud<PointT> &cluster, pcl::ModelCoefficients::Ptr coefficients);
//boundingBoxWithCoeff(pcl::PointCloud<PointT> &cluster, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<PointT>::Ptr &cloud_transformed);

void 
fillBasicFeaturesMsg (hlpr_perception_msgs::BasicFeatures &basicInfo, const pc_cluster_features &inObjFeatures);

void 
fillOrientedBoundingBoxMsg (hlpr_perception_msgs::OrientedBoundingBox &obb, const pc_cluster_features &inObjFeatures);

void 
fillColorHistMsg(hlpr_perception_msgs::ColorHist &hs, const pc_cluster_features &inObjFeatures);

void 
fillShapeHistMsg(hlpr_perception_msgs::ShapeHist &sh, const pc_cluster_features &inObjFeatures);

void
fillViewpointHistMsg(hlpr_perception_msgs::ViewpointHist &vph, const pc_cluster_features &inObjFeatures);

void
fillRosMessageForPlanes (hlpr_perception_msgs::PlaneFeatures &planeRosMsg, const pc_cluster_features &inObjFeatures);

void
fillRosMessageForObjects (hlpr_perception_msgs::ObjectFeatures &objRosMsg, 
                            const pc_cluster_features &inObjFeatures);

// Top-level message for Objects
void
fillObjectFeaturesMsg (hlpr_perception_msgs::ObjectFeatures &objRosMsg, 
                       hlpr_perception_msgs::BasicFeatures &basicInfo,
                       hlpr_perception_msgs::OrientedBoundingBox &obb,
                       hlpr_perception_msgs::ColorHist &hs,
                       hlpr_perception_msgs::ShapeHist &sh,
                       hlpr_perception_msgs::ViewpointHist &vph,
                       hlpr_perception_msgs::OtherFeatures &other,
                        const pc_cluster_features &inObjFeatures);

// Top-level message for planes
void
fillPlaneFeaturesMsg (hlpr_perception_msgs::PlaneFeatures &planeRosMsg, 
                      hlpr_perception_msgs::BasicFeatures &basicInfo,
                      hlpr_perception_msgs::OrientedBoundingBox &obb,
                      const pc_cluster_features &inObjFeatures);

void
objectPoseTF(geometry_msgs::Transform geom_transform);

void
getObjectMarker(visualization_msgs::Marker &marker, hlpr_perception_msgs::ObjectFeatures &feats);


template<class pcType>
bool
loadPCD(char *name, pcType cloud_ptr)
{
  if (isPath(name) != PT_FILE)
  {
      std::cout << "Filename " << name << " is not a valid file."
          << std::endl;
      return false;
  }
  bool result = (pcl::io::loadPCDFile(name, *cloud_ptr) == 0);
  if (result)
    std::cout << "Loaded file " << name << " with " << cloud_ptr->size()
        << " points.";
  std::cout << std::endl;
  return result;
}

#endif /* UTILS_PCL_ROS_HPP_ */
