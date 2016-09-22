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
fillBasicFeaturesMsg (nonplanar_feature_extraction::BasicFeatures &basicInfo, const pc_cluster_features &inObjFeatures, bool setNumPoints)
{
  basicInfo.points_centroid.x = inObjFeatures.centroid[0];
  basicInfo.points_centroid.y = inObjFeatures.centroid[1];
  basicInfo.points_centroid.z = inObjFeatures.centroid[2];

  basicInfo.points_min.x = inObjFeatures.min[0];
  basicInfo.points_min.y = inObjFeatures.min[1];
  basicInfo.points_min.z = inObjFeatures.min[2];

  basicInfo.points_max.x = inObjFeatures.max[0];
  basicInfo.points_max.y = inObjFeatures.max[1];
  basicInfo.points_max.z = inObjFeatures.max[2];

  if(setNumPoints == true)               // For objects, num_points is true
  {        
      basicInfo.setNumPoints = true;
      basicInfo.num_points = (float) inObjFeatures.volume2; 
  }
  else
  {
     basicInfo.setNumPoints = false;
     basicInfo.num_points = 0;
  }

  basicInfo.rgba_color.r = inObjFeatures.color[0]/255.;
  basicInfo.rgba_color.g = inObjFeatures.color[1]/255.;
  basicInfo.rgba_color.b = inObjFeatures.color[2]/255.;
  basicInfo.rgba_color.a = 1.0;
  basicInfo.hue = inObjFeatures.hue;
} 

void 
fillOrientedBoundingBoxMsg (nonplanar_feature_extraction::OrientedBoundingBox &obb, const pc_cluster_features &inObjFeatures)
{
  obb.bb_center.x = inObjFeatures.oriented_bounding_box.center.x;
  obb.bb_center.y = inObjFeatures.oriented_bounding_box.center.y;
  obb.bb_center.z = inObjFeatures.oriented_bounding_box.center.z;

  obb.bb_dims.x = inObjFeatures.oriented_bounding_box.size.xSize;
  obb.bb_dims.y = inObjFeatures.oriented_bounding_box.size.ySize;
  obb.bb_dims.z = inObjFeatures.oriented_bounding_box.size.zSize;

  obb.bb_rot_quat.x = inObjFeatures.oriented_bounding_box.rot_quat[0];
  obb.bb_rot_quat.y = inObjFeatures.oriented_bounding_box.rot_quat[1];
  obb.bb_rot_quat.z = inObjFeatures.oriented_bounding_box.rot_quat[2];
  obb.bb_rot_quat.w = inObjFeatures.oriented_bounding_box.rot_quat[3];
}

void
fillColorHistMsg(nonplanar_feature_extraction::ColorHist &hs, const pc_cluster_features &inObjFeatures)
{
  hs.hs_features_size = 256;
  for(int i = 0;i<hs.hs_features_size;i++)
    hs.hs_feature_vector.push_back(inObjFeatures.histogram_hs[i]);
}

void
fillShapeHistMsg(nonplanar_feature_extraction::ShapeHist &sh, const pc_cluster_features &inObjFeatures)
{
  sh.cvfh_features_size = 308;
  for(int i = 0;i<sh.cvfh_features_size;i++)
    sh.cvfh_feature_vector.push_back(inObjFeatures.cvfhs.points[0].histogram[i]);
    
  sh.fpfh_features_size = 308;
  for(int i = 0;i<sh.fpfh_features_size;i++)
    sh.fpfh_feature_vector.push_back(inObjFeatures.fpfhs.points[0].histogram[i]);
}

void 
fillViewpointHistMsg(nonplanar_feature_extraction::ViewpointHist &vph, const pc_cluster_features &inObjFeatures)
{
  vph.vfh_features_size = 308;
  for(int i = 0;i<vph.vfh_features_size;i++)
    vph.vfh_feature_vector.push_back(inObjFeatures.vfhs.points[0].histogram[i]);
}

void 
fillOtherFeaturesMsg(nonplanar_feature_extraction::OtherFeatures &other, const pc_cluster_features &inObjFeatures)
{
  other.other_features_size = inObjFeatures.otherFeatures.size();
  for(int i = 0;i<other.other_features_size;i++)
    other.data.push_back(inObjFeatures.otherFeatures[i]);
}

void 
fillObjectFeaturesMsg (nonplanar_feature_extraction::ObjectFeatures &objRosMsg, 
                   nonplanar_feature_extraction::BasicFeatures &basicInfo,
                   nonplanar_feature_extraction::OrientedBoundingBox &obb,
                   nonplanar_feature_extraction::ColorHist &hs,
                   nonplanar_feature_extraction::ShapeHist &sh,
                   nonplanar_feature_extraction::ViewpointHist &vph,
                   nonplanar_feature_extraction::OtherFeatures &other,
                   const pc_cluster_features &inObjFeatures)
{
    objRosMsg.header.stamp = ros::Time::now();

    objRosMsg.transform.translation.x = obb.bb_center.x;
    objRosMsg.transform.translation.y = obb.bb_center.y;
    objRosMsg.transform.translation.z = obb.bb_center.z;

    objRosMsg.transform.rotation.x = obb.bb_rot_quat.x;
    objRosMsg.transform.rotation.y = obb.bb_rot_quat.y;
    objRosMsg.transform.rotation.z = obb.bb_rot_quat.z;
    objRosMsg.transform.rotation.w = obb.bb_rot_quat.w;

    objRosMsg.basicInfo = basicInfo;
    objRosMsg.obb = obb;
   
    objRosMsg.setColorHist = inObjFeatures.setColorHist;
    objRosMsg.hs_hist = hs;

    objRosMsg.setShapeHist = inObjFeatures.setShapeHist;
    objRosMsg.shape_hist = sh;

    objRosMsg.setViewpointHist = inObjFeatures.setViewpointHist;
    objRosMsg.view_hist = vph;

    objRosMsg.setOtherFeatures = inObjFeatures.setOtherFeatures;
    objRosMsg.other = other; 
}

void
fillPlaneFeaturesMsg(nonplanar_feature_extraction::PlaneFeatures &planeRosMsg, 
                     nonplanar_feature_extraction::BasicFeatures &basicInfo,
                     nonplanar_feature_extraction::OrientedBoundingBox &obb,
                     const pc_cluster_features &inObjFeatures)
{
  planeRosMsg.header.stamp = ros::Time::now();

  planeRosMsg.basicInfo = basicInfo;
  planeRosMsg.obb = obb;
}

// Takes in all the features and then fills up the messages accordingly
void
fillRosMessageForObjects (nonplanar_feature_extraction::ObjectFeatures &objRosMsg, const pc_cluster_features &inObjFeatures)
{
  // Fill up the BasicFeatures.msg
  nonplanar_feature_extraction::BasicFeatures basicInfo;
  fillBasicFeaturesMsg(basicInfo, inObjFeatures, true);

  // Fill up the OrientedBoundingBox.msg
  nonplanar_feature_extraction::OrientedBoundingBox obb;
  fillOrientedBoundingBoxMsg(obb, inObjFeatures);
  
  // Fill up the ColorHist.msg
  nonplanar_feature_extraction::ColorHist hs;
  fillColorHistMsg(hs, inObjFeatures);

  // Fill up the ShapeHist.msg
  nonplanar_feature_extraction::ShapeHist sh;
  fillShapeHistMsg(sh, inObjFeatures);

  // Fill up the ViewpointHist.msg
  nonplanar_feature_extraction::ViewpointHist vph;
  fillViewpointHistMsg(vph, inObjFeatures);

  // Fill up OtherFeatures.msg
  nonplanar_feature_extraction::OtherFeatures other;
  fillOtherFeaturesMsg(other, inObjFeatures);

  // Fill out the top-level main msg for objects
  fillObjectFeaturesMsg(objRosMsg, basicInfo, obb, hs, sh, vph, other, inObjFeatures);
}

void
fillRosMessageForPlanes (nonplanar_feature_extraction::PlaneFeatures &planeRosMsg, const pc_cluster_features &inObjFeatures)
{
  // Fill up the BasicFeatures.msg
  nonplanar_feature_extraction::BasicFeatures basicInfo;
  fillBasicFeaturesMsg(basicInfo, inObjFeatures, false);           // False flag sent as the volume for planes is not right
 
  // Fill up the OrientedBoundingBox.msg
  nonplanar_feature_extraction::OrientedBoundingBox obb;
  fillOrientedBoundingBoxMsg(obb, inObjFeatures);

  // Fill out the top-level main msg for planes
  fillPlaneFeaturesMsg(planeRosMsg, basicInfo, obb, inObjFeatures);
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

