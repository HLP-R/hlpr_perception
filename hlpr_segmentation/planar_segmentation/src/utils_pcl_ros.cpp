/*
 * utils_pcl_ros.cpp
 *
 *  Created on: Sep 9, 2014
 *      Author: baris
 */

#include<utils_pcl_ros.hpp>

int findFeatVecByHue(std::vector<pc_cluster_features> &feats, double main_hue)
{
  std::vector<double> hue_diffs (feats.size());
  double min_diff = 1000;
  int color_index = -1;
  for(int i = 0; i < feats.size(); i++)
  {
    hue_diffs[i] = hueDiff(feats[i].hue, main_hue);//abs(feats[i].hue - main_hue);
    if(hue_diffs[i] < min_diff)
    {
      min_diff = hue_diffs[i];
      color_index = i;
    }
    else if(hue_diffs[i] == min_diff)
    {
      if(feats[i].volume2 > feats[color_index].volume2)
        color_index = i;
    }
  }
  return color_index;
}

int getClusterByColor(std::vector<pc_cluster_features> &feats, int feature_number, bool is_max)
{
  unsigned char max_color = 0;
  int cluster_num = -1;
  for(int i = 0; i < feats.size(); i++)
  {
    if(feats[i].color[feature_number] > max_color)
    {
      max_color = feats[i].color[feature_number];
      cluster_num = i;
    }
  }
  return cluster_num;
}

void getClusterByColorThresh(std::vector<pc_cluster_features> &feats, int feature_number, std::vector<int > &indices, unsigned char low, unsigned char high)
{
  for(int i = 0; i < feats.size(); i++)
  {
    if(feats[i].color[feature_number] > low && feats[i].color[feature_number] < high)
    {
        indices.push_back(i);
    }
  }
}

Box3D boundingBoxWithCoeff(pcl::PointCloud<PointT> &cluster, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<PointT>::Ptr &cloud_transformed)
{
	Box3D aligned_bounding_box;
	Eigen::Vector4f min;
	Eigen::Vector4f max;

	pcl::PointCloud<PointT>::Ptr projected_cluster = cluster.makeShared();
	//pcl::PointCloud<PointT>::Ptr cloud_transformed (new pcl::PointCloud<PointT>);

	Eigen::Vector3f z_axis(0,0,1);
	Eigen::Vector3f plane_normal(coefficients->values[0],coefficients->values[1],coefficients->values[2]);

	plane_normal.normalize();

	Eigen::Vector3f c = z_axis.cross(plane_normal);//z_axis.cross(plane_normal);
	c.normalize();

	//std::cout << c << std::endl;
	//std::cout << plane_normal << std::endl << std::endl;

	double cost  = plane_normal.dot(z_axis);
	double half_theta = acos(cost)/2;
	double sin_ht = sin(half_theta);

	Eigen::Quaternion<float> q(cos(half_theta), c[0]*sin_ht, c[1]*sin_ht, c[2]*sin_ht);
	Eigen::Matrix3f R = q.toRotationMatrix();
	Eigen::Affine3f T = Eigen::Affine3f::Identity();
	T.rotate(R);

	//std::cout << q.w() << " " << q.x() << " " <<  q.y() << " " << q.z() << " " << std::endl;

	pcl::transformPointCloud (*projected_cluster, *cloud_transformed, T);

	pcl::getMinMax3D(*cloud_transformed, min, max);

	std::vector<float> tmp;

	for(size_t j = 1; j < cloud_transformed->points.size(); j++)
	{
		tmp.push_back(cloud_transformed->points[j].z);
		cloud_transformed->points[j].z = cloud_transformed->points[0].z;
	}

	aligned_bounding_box = minAreaRect(cloud_transformed);

	for(size_t j = 1; j < cloud_transformed->points.size(); j++)
	{
		cloud_transformed->points[j].z = tmp[j];
	}

	Eigen::Quaternion<float> q2(cos(aligned_bounding_box.angle/2), 0, 0, sin(aligned_bounding_box.angle/2));
	Eigen::Quaternion<float> q3 = q2*q;


	aligned_bounding_box.center.z = min[2]+(max[2]-min[2])/2; //this should be maxheight/2+planeHeight
	aligned_bounding_box.size.zSize = fabs((float)(max[2]-min[2])); //this should be max height!
	aligned_bounding_box.fillQuatGivenAxisAngle();
	/*Eigen::Vector3f X(aligned_bounding_box.center.x,aligned_bounding_box.center.y,aligned_bounding_box.center.z);
	Eigen::Vector3f Y(0,0,0);

	Y = T.inverse()*X;

	aligned_bounding_box.center.x = Y.x();
	aligned_bounding_box.center.y = Y.y();
	aligned_bounding_box.center.z = Y.z();

	aligned_bounding_box.rot_quat[0] = q3.x();
	aligned_bounding_box.rot_quat[1] = q3.y();
	aligned_bounding_box.rot_quat[2] = q3.z();
	aligned_bounding_box.rot_quat[3] = q3.w();*/

	return aligned_bounding_box;
}

// Implementations
void
Box3D_2_MC(Box3D &in_box, pcl::ModelCoefficients &out_cube)
{
  out_cube.values.resize(10);
  //position of center
  out_cube.values[0] = in_box.center.x;
  out_cube.values[1] = in_box.center.y;
  out_cube.values[2] = in_box.center.z;

  //rotation in quaternions, assuming the rotation is in z only and the quat representation is [qx,qy,qz,qw]
  out_cube.values[3] = in_box.rot_quat[0];
  out_cube.values[4] = in_box.rot_quat[1];
  out_cube.values[5] = in_box.rot_quat[2];
  out_cube.values[6] = in_box.rot_quat[3];

  //std::cout << out_cube.values[3] << " " << out_cube.values[4] << " " << out_cube.values[5] << " " << out_cube.values[6] << " " << std::endl;

  //side lengths
  out_cube.values[7] = in_box.size.xSize;
  out_cube.values[8] = in_box.size.ySize;
  out_cube.values[9] = in_box.size.zSize;
}

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
minAreaRect(pcl::PointCloud<PointT>::Ptr &input_cloud, pcl::ModelCoefficients &out_cube)
{
  Eigen::Vector4f min;
  Eigen::Vector4f max;
  pcl::getMinMax3D(*input_cloud, min, max);
  Box3D box = minAreaRect(input_cloud);
  box.center.z = min[2]+(max[2]-min[2])/2; //this should be maxheight/2+planeHeight
  box.size.zSize = fabs((float)(max[2]-min[2])); //this should be max height!
  Box3D_2_MC(box, out_cube);
}

void
fillRosMessage (pc_segmentation::PcFeatures &outRosMsg, const pc_cluster_features &inObjFeatures)
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

void getObjectMarker(visualization_msgs::Marker &marker, pc_segmentation::PcFeatures &feats)
{
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/marker_frame";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = feats.transform.translation.x;
  marker.pose.position.y = feats.transform.translation.y;
  marker.pose.position.z = feats.transform.translation.z;
  marker.pose.orientation.x = feats.transform.rotation.x;
  marker.pose.orientation.y = feats.transform.rotation.y;
  marker.pose.orientation.z = feats.transform.rotation.z;
  marker.pose.orientation.w = feats.transform.rotation.w;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = feats.bb_dims.x;
  marker.scale.y = feats.bb_dims.y;
  marker.scale.z = feats.bb_dims.z;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = feats.rgba_color.r;
  marker.color.g = feats.rgba_color.g;
  marker.color.b = feats.rgba_color.b;
  marker.color.a = feats.rgba_color.a;

  marker.lifetime = ros::Duration();

  //if using a mesh!!
  //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
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

