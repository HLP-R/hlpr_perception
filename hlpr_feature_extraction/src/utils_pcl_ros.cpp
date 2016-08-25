/*
 * utils_pcl_ros.cpp
 *
 *  Created on: Sep 9, 2014
 *      Author: baris
 */

#include<utils_pcl_ros.hpp>


void
Cube_2_Arrows(pcl::ModelCoefficients &cube, boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, int index)
{
  pcl::PointXYZ base;
  base.x = cube.values[0];
  base.y = cube.values[1];
  base.z = cube.values[2];

  pcl::PointXYZ p2a;
  Eigen::Quaternionf quat(cube.values[6],cube.values[3],cube.values[4],cube.values[5]);
  Eigen::Matrix3f mat(quat.matrix());

  p2a.x = base.x + mat(0,0)*0.1;
  p2a.y = base.y + mat(1,0)*0.1;
  p2a.z = base.z + mat(2,0)*0.1;


  pcl::PointXYZ p2b;
  p2b.x = base.x + mat(0,1)*0.1;
  p2b.y = base.y + mat(1,1)*0.1;
  p2b.z = base.z + mat(2,1)*0.1;

  pcl::PointXYZ p2c;
  p2c.x = base.x + mat(0,2)*0.1;
  p2c.y = base.y + mat(1,2)*0.1;
  p2c.z = base.z + mat(2,2)*0.1;

  char name[1024];
  sprintf(name, "line %d-a", index);
  viewer->addLine(p2a, base, 255.0, 0.0, 0.0, name);
  sprintf(name, "line %d-b", index);
  viewer->addLine(p2b, base, 0.0, 255.0, 0.0, name);
  sprintf(name, "line %d-c", index);
  viewer->addLine(p2c, base, 0.0, 0.0, 255.0, name);
}

void
fillRosMessage (hlpr_feature_extraction::PcFeatures &outRosMsg, const pc_cluster_features &inObjFeatures)
{
  outRosMsg.header.stamp = ros::Time::now();

  outRosMsg.transform.translation.x = inObjFeatures.aligned_bounding_box.center.x;
  outRosMsg.transform.translation.y = inObjFeatures.aligned_bounding_box.center.y;
  outRosMsg.transform.translation.z = inObjFeatures.aligned_bounding_box.center.z;
  //make sure the quat order is correct!
  outRosMsg.transform.rotation.x = inObjFeatures.aligned_bounding_box.rot_quat[0];
  outRosMsg.transform.rotation.y = inObjFeatures.aligned_bounding_box.rot_quat[1];
  outRosMsg.transform.rotation.z = inObjFeatures.aligned_bounding_box.rot_quat[2];
  outRosMsg.transform.rotation.w = inObjFeatures.aligned_bounding_box.rot_quat[3];

  outRosMsg.points_centroid.x = inObjFeatures.centroid[0];
  outRosMsg.points_centroid.y = inObjFeatures.centroid[1];
  outRosMsg.points_centroid.z = inObjFeatures.centroid[2];

  outRosMsg.points_min.x = inObjFeatures.min[0];
  outRosMsg.points_min.y = inObjFeatures.min[1];
  outRosMsg.points_min.z = inObjFeatures.min[2];

  outRosMsg.points_max.x = inObjFeatures.max[0];
  outRosMsg.points_max.y = inObjFeatures.max[1];
  outRosMsg.points_max.z = inObjFeatures.max[2];

  outRosMsg.num_points = (float) inObjFeatures.volume2;

  outRosMsg.rgba_color.r = inObjFeatures.color[0]/255.;
  outRosMsg.rgba_color.g = inObjFeatures.color[1]/255.;
  outRosMsg.rgba_color.b = inObjFeatures.color[2]/255.;
  outRosMsg.rgba_color.a = 1.0;
  outRosMsg.hue = inObjFeatures.hue;

  outRosMsg.bb_center.x = inObjFeatures.aligned_bounding_box.center.x;
  outRosMsg.bb_center.y = inObjFeatures.aligned_bounding_box.center.y;
  outRosMsg.bb_center.z = inObjFeatures.aligned_bounding_box.center.z;

  outRosMsg.bb_angle  = inObjFeatures.aligned_bounding_box.angle;
  outRosMsg.bb_dims.x = inObjFeatures.aligned_bounding_box.size.xSize;
  outRosMsg.bb_dims.y = inObjFeatures.aligned_bounding_box.size.ySize;
  outRosMsg.bb_dims.z = inObjFeatures.aligned_bounding_box.size.zSize;

  outRosMsg.other_features_size = 308;
  for(int i = 0;i<outRosMsg.other_features_size;i++)
    outRosMsg.data.push_back(inObjFeatures.vfhs.points[0].histogram[i]);

}

void objectPoseTF(geometry_msgs::Transform geom_transform)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.transform = geom_transform;
  transformStamped.header.stamp = ros::Time::now();
  //or is it kinect_ir_optical_frame
  transformStamped.header.frame_id = "kinect_rgb_optical_frame";
  transformStamped.child_frame_id = "main_object";
  br.sendTransform(transformStamped);
}

template<class pcType>
bool
loadPCD(char *name, pcType cloud_ptr);

