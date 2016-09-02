/*
 * objectFeatures.hpp
 *
 *  Created on: Jul 31, 2013
 *      Author: siml
 */
#ifndef OBJECTFEATURES_HPP_
#define OBJECTFEATURES_HPP_

#include<common.hpp>

#include<misc_structures.hpp>
#include<utils.hpp>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include"rotcalipers.h"
#include<pcl/PointIndices.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include<pcl/common/centroid.h>
#include<pcl/common/transforms.h>
#include<pcl/ModelCoefficients.h>
#include<pcl/filters/project_inliers.h>

//use eigen data structures?


class pc_cluster_features
{
private:
  bool isFilled;

  //(r,g,b),(x,y,z),(angle),(volume2,volume,area),(aspect ratio,av_ratio, compactness),(vfh)
  void defaultConst() { isFilled = false; numFeatures =  26+308;

}

  void boundingBoxWithZ(pcl::PointCloud<PointT> &cluster)
  {
    pcl::PointCloud<PointT>::Ptr projected_cluster = cluster.makeShared();
    for(size_t j = 0; j < projected_cluster->points.size(); j++)
      projected_cluster->points[j].z = 1;

     volume2 = cluster.size();

     pcl::getMinMax3D(cluster, min, max);
     aligned_bounding_box = minAreaRect(projected_cluster);
     aligned_bounding_box.center.z = min[2]+(max[2]-min[2])/2; //this should be maxheight/2+planeHeight
     aligned_bounding_box.size.zSize = fabs((float)(max[2]-min[2])); //this should be max height!

     aligned_bounding_box.fillQuatGivenAxisAngle();
  }

  void boundingBoxWithCoeff(pcl::PointCloud<PointT> &cluster, pcl::ModelCoefficients::Ptr coefficients)
  {
    pcl::PointCloud<PointT>::Ptr projected_cluster = cluster.makeShared();

    pcl::PointCloud<PointT>::Ptr cloud_transformed (new pcl::PointCloud<PointT>);

    /*pcl::PointCloud<PointT>::Ptr cloud_projected (new pcl::PointCloud<PointT>);
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cluster.makeShared());
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);*/

    Eigen::Vector3f z_axis(0,0,1);
    Eigen::Vector3f plane_normal(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
//    Eigen::Vector3f plane_normal(0.0270649, 0.849503, 0.526889);

    plane_normal.normalize();

    Eigen::Vector3f c = plane_normal.cross(z_axis);//z_axis.cross(plane_normal);
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

    volume2 = cluster.size();
    pcl::getMinMax3D(*cloud_transformed, min, max);

    for(size_t j = 0; j < cloud_transformed->points.size(); j++)
    	cloud_transformed->points[j].z = 1;

    aligned_bounding_box = minAreaRect(cloud_transformed);


    Eigen::Quaternion<float> q2(cos(-aligned_bounding_box.angle/2), 0, 0, sin(-aligned_bounding_box.angle/2));
    Eigen::Quaternion<float> q3 = q2*q;
    q3 = q3.inverse();

    //pcl::getMinMax3D(cluster, min, max);
    //aligned_bounding_box = minAreaRect(projected_cluster);

    /*for(int i = 0; i< 3 ; i++)
      aligned_bounding_box.rot_axis[i] = c[i];
    aligned_bounding_box.fillQuatGivenAxisAngle();*/

    aligned_bounding_box.center.z = min[2]+(max[2]-min[2])/2; //this should be maxheight/2+planeHeight
    aligned_bounding_box.size.zSize = fabs((float)(max[2]-min[2])); //this should be max height!

    Eigen::Vector3f X(aligned_bounding_box.center.x,aligned_bounding_box.center.y,aligned_bounding_box.center.z);
    Eigen::Vector3f Y(0,0,0);

    Y = T.inverse()*X;

    aligned_bounding_box.center.x = Y.x();
    aligned_bounding_box.center.y = Y.y();
    aligned_bounding_box.center.z = Y.z();

    /*for(int i = 0; i < 3; i++)
        aligned_bounding_box.rot_axis[i] = c[i];//coefficients->values[i];

    aligned_bounding_box.angle = //half_theta*2;

    aligned_bounding_box.fillQuatGivenAxisAngle();*/

    aligned_bounding_box.rot_quat[0] = q3.x();
	aligned_bounding_box.rot_quat[1] = q3.y();
	aligned_bounding_box.rot_quat[2] = q3.z();
    aligned_bounding_box.rot_quat[3] = q3.w();


  }

  void clusterConst(pcl::PointCloud<PointT> &cluster)
  {
    cloud = cluster.makeShared();
     /*if(aligned_bounding_box.angle < -0.2618) //this could end up being totally random ahahah (~ -15.0 degrees)
       aligned_bounding_box.angle = aligned_bounding_box.angle + M_PI;*/

     /*boxIt.fillQuatGivenAxisAngle();
     Eigen::Quaternionf tmpQuat(boxIt.rot_quat[3],boxIt.rot_quat[],boxIt.rot_quat[1],boxIt.rot_quat[0]);
     /*tmpQuat = plane_transformation.rotation().inverse()*tmpQuat;
     boxIt.rot_quat[0] = tmpQuat.x(); boxIt.rot_quat[1] = tmpQuat.y(); boxIt.rot_quat[2] = tmpQuat.z(); boxIt.rot_quat[3] = tmpQuat.w();*/

    volume2 = cluster.size();

     float r = 0.0f;
     float g = 0.0f;
     float b = 0.0f;
     float n = static_cast<float> (cluster.size());
     for (size_t j = 0; j < n; j++)
     {
      r += (float) (cluster[j].r);
      g += (float) (cluster[j].g);
      b += (float) (cluster[j].b);
     }
     r /= n; g /= n; b /= n;

     hue = rgb2hue(r,g,b);


     //std::cout << rgb2hue(r,g,b) << endl;

     int ri = r; int gi = (int)g; int bi = (int)b;

     color[0] = (unsigned char) ri;
     color[1] = (unsigned char) gi;
     color[2] = (unsigned char) bi;

     fill();
  }

public:
  int numFeatures;

  pc_cluster_features(){defaultConst();}
  pc_cluster_features(pcl::PointCloud<PointT> &cluster)
  {
    defaultConst();

    pcl::compute3DCentroid(cluster, centroid);

    boundingBoxWithZ(cluster);

    //BARIS: PROJECT HERE
//        pcl::transformPointCloud (cluster, *projected_cluster, plane_transformation);
//        pcl::transformPointCloud (cluster, cluster, plane_transformation);
//        pcl::PointCloud<PointT>::Ptr projected_cluster = cluster.makeShared(); //(new pcl::PointCloud<PointT>)
      //BARIS: USE THIS PROJECTION TO YOUR ADVANTAGE WHEN CALCULATING HEIGHT!
      /*proj.setInputCloud (cluster.makeShared());
      for(int t = 0; t<4; t++) pl_coefficients->values[t] = closest_plane_model[t];
      proj.setModelCoefficients (pl_coefficients);
      proj.filter (*projected_cluster);*/
//        pcl::transformPointCloud (*projected_cluster, *projected_cluster, plane_transformation);
//        for(size_t j = 0; j < projected_cluster->points.size(); j++)
//          projected_cluster->points[j].z = 1;

    clusterConst(cluster);

  }
  pc_cluster_features(pcl::PointCloud<PointT> &cluster, Eigen::Vector4f &used_plane_model)
   {
     defaultConst();

     pcl::compute3DCentroid(cluster, centroid);

     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());;
     coefficients->values.resize (4);
     //std::cout << "here" << std::endl;
     int zero_count = 0;
     for(int i = 0; i < 4; i++)
     {
       if(used_plane_model[i] == 0)
         zero_count++;
       coefficients->values[i] = used_plane_model[i];
     }

     bool use_projection = true;
     if(zero_count > 3)
       use_projection = false;

     if(use_projection)
       boundingBoxWithCoeff(cluster, coefficients);
     else
       boundingBoxWithZ(cluster);

     clusterConst(cluster);

   }

  ~pc_cluster_features(){}

  Box3D aligned_bounding_box;
  //Box3D aligned_bounding_box2;

  //FEATURES

  //float volume;
  //float area;
  int volume2; //num_data points

  //Eigen::Vector3f color;
  ColorVec color;
  float hue;

  Eigen::Vector4f centroid;
  Eigen::Quaternionf rotation_quat; // kind of hard to get full orientation, need to fit a box which is not trivial! n^3, needs 3d convez hull and hard to implement, approximations exist

  Eigen::Vector4f min;
  Eigen::Vector4f max;

  //assuming on table, coming from the bounding box
  float bb_aspect_ratio;
  float bb_orientation; //wrt to the table, use a complex number?

  float compactness; // volume/bb_volume need a better name?
  float av_ratio;   //  area/volume

  float bb_volume;
  float bb_area;

  float objectBuffer[5000];

  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::PointCloud<PointT>::Ptr parent_cloud;
  pcl::PointIndices indices; //indices in the parent point cloud
  pcl::PointCloud<pcl::Normal> normals;
  pcl::PointCloud<pcl::VFHSignature308> vfhs;// (new pcl::PointCloud<pcl::VFHSignature308> ());

  //Image based features??? blob features, color histogram? using opencv

  void fill()
  {
    aligned_bounding_box.calculateProperties();
    //aligned_bounding_box.fillQuatGivenAxisAngle();

    bb_orientation = aligned_bounding_box.angle;
    bb_volume = aligned_bounding_box.volume;
    bb_area = aligned_bounding_box.area;
    bb_aspect_ratio = aligned_bounding_box.aspect_ratio;
    rotation_quat = Eigen::Quaternionf(aligned_bounding_box.rot_quat);

    compactness = (float)bb_volume/volume2;
    //where is the real area/real volume?
    av_ratio = bb_area/bb_volume;

    isFilled = true;
  }

  void fillFeatureContainer (float out_features[], int start_index = 0)
  {
    int feature_counter = start_index;

    out_features[feature_counter++] = numFeatures;

    out_features[feature_counter++] = aligned_bounding_box.center.x;
    out_features[feature_counter++] = aligned_bounding_box.center.y;
    out_features[feature_counter++] = aligned_bounding_box.center.z;
    out_features[feature_counter++] = aligned_bounding_box.angle;

    for(int i = 0;i<3;feature_counter++,i++)
      out_features[feature_counter] = centroid[i];

    for(int i = 0;i<3;feature_counter++,i++)
      out_features[feature_counter] = min[i];

    for(int i = 0;i<3;feature_counter++,i++)
      out_features[feature_counter] = max[i];

    for(int i = 0;i<3;feature_counter++,i++)
      out_features[feature_counter] = color[i];

    out_features[feature_counter++] = hue;

    out_features[feature_counter++] = volume2;

    out_features[feature_counter++] = aligned_bounding_box.size.xSize;
    out_features[feature_counter++] = aligned_bounding_box.size.ySize;
    out_features[feature_counter++] = aligned_bounding_box.size.zSize;

    out_features[feature_counter++] = bb_volume;
    out_features[feature_counter++] = bb_area;
    out_features[feature_counter++] = bb_aspect_ratio;
    out_features[feature_counter++] = av_ratio;
    out_features[feature_counter++] = compactness;

    for(int i = 0;i<308;feature_counter++,i++)
      out_features[feature_counter] = vfhs.points[0].histogram[i];

    if((feature_counter-start_index-1) != numFeatures)
      std::cout << " fc: " << feature_counter-start_index-1 << ", nf: " << numFeatures << std::endl;
  }

  void write2file(const char *fileName = "tmp.txt", bool isAppend = false, bool isCol = false)
  {
	/*if(fileName == NULL)
	{
		fileName =  malloc(8 * sizeof(char));
		sprintf(fileName,"%s","tmp.txt");
	}*/
    if(!isFilled)
    {
      std::cerr << "Trying to write the features to a file without initializing first." << std::endl;
      return;
    }

    std::ofstream myfile;
    if(!isAppend)
      myfile.open( fileName );
    else
      myfile.open( fileName, std::ios::out | std::ios::app );

    //(x,y,z),(angle),(r,g,b),(volume2, volume, area),(aspect ratio,av_ratio, compactness),(vfh)
    char *delim = (char*)malloc(sizeof(char));
    if(~isCol)
    {
        sprintf(delim,"%c", ',');//delim = ",";
    }
    else
    {
    	sprintf(delim,"%c", '\n');//delim= "\n";
    }

    //a bit inefficient since we are copying the vfhs points  but it is more consistent this way.
    fillFeatureContainer(objectBuffer,0);
    array2file(objectBuffer,numFeatures+1,myfile,delim);

    /*myfile << centroid[0] << delim << centroid[1] << delim << centroid[2] << delim << aligned_bounding_box.angle << delim; //4
    myfile << color[0] << delim << color[1] << delim << color[2] << delim; //7
    myfile << volume2 << delim << bb_volume << delim << bb_area << delim; //10
    myfile << aligned_bounding_box.size.xSize << delim << aligned_bounding_box.size.ySize <<  delim << aligned_bounding_box.size.zSize <<  delim; //13
    myfile << bb_aspect_ratio << delim << av_ratio << delim << compactness << delim; //16
    array2file(vfhs.points[0].histogram, 308, myfile, delim);

    myfile.close();*/
  }
};


/*float t = 0;
float h = (coefficients->values[0]*coefficients->values[0] +
		   coefficients->values[1]*coefficients->values[1] +
		   coefficients->values[2]*coefficients->values[2]); // h = 1!;

for(size_t j = 0; j < projected_cluster->points.size(); j++)
{
	t = (coefficients->values[0]*projected_cluster->points[j].x
	  +  coefficients->values[1]*projected_cluster->points[j].y
	  +  coefficients->values[2]*projected_cluster->points[j].z
	  +  coefficients->values[3]);

	projected_cluster->points[j].x = projected_cluster->points[j].x - coefficients->values[0]*t;
	projected_cluster->points[j].y = projected_cluster->points[j].y - coefficients->values[1]*t;
	projected_cluster->points[j].z = projected_cluster->points[j].z - coefficients->values[2]*t;
}*/

#endif /* OBJECTFEATURES_HPP_ */

