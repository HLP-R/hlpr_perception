/*
 * objectFeatures.hpp
 *
 *  Created on: Jul 31, 2013
 *      Author: siml
 */
#ifndef OBJECTFEATURES_HPP_
#define OBJECTFEATURES_HPP_

#include<common.hpp>
#include <vector>
#include <cmath>

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
#include <pcl/features/moment_of_inertia_estimation.h>

//use eigen data structures?

class pc_cluster_features
{
private:
  bool isFilled;

  //(r,g,b),(x,y,z),(angle),(volume2,volume,area),(aspect ratio,av_ratio, compactness),(vfh)
  void defaultConst() { isFilled = false; numFeatures =  26+308;

}

  // Added by Priyanka - for non-planar segmentation
  void orientedBoundingBox(pcl::PointCloud<PointT> &cluster)
  {
      pcl::PointCloud<PointT>::Ptr projected_cluster = cluster.makeShared();
      volume2 = cluster.size();
      pcl::getMinMax3D(cluster, min, max);
      
      std::vector <float> moment_of_inertia;
      std::vector <float> eccentricity;
      pcl::PointXYZ min_point_OBB;
      pcl::PointXYZ max_point_OBB;
      pcl::PointXYZ position_OBB;
      Eigen::Matrix3f rotational_matrix_OBB;
      //Eigen::Vector3f major_vector, middle_vector, minor_vector;

      pcl::PointCloud<PointT>::Ptr cluster_xyzrgb (projected_cluster);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*cluster_xyzrgb, *cloud);
      pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
      feature_extractor.setInputCloud(cloud);
      feature_extractor.compute();
  
      //feature_extractor.getMomentOfInertia (moment_of_inertia);
      //feature_extractor.getEccentricity (eccentricity);
      feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
      //feature_extractor.getEigenValues(major_value, middle_value, minor_value);
      //feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
      //feature_extractor.getMassCenter(mass_center);

      // Fill up the Box3D structure
      Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
      oriented_bounding_box.center.x = position(0);
      oriented_bounding_box.center.y = position(1);
      oriented_bounding_box.center.z = position(2);

      Eigen::Quaternion<float> quat (rotational_matrix_OBB);
      oriented_bounding_box.rot_quat[0] = quat.x();
	    oriented_bounding_box.rot_quat[1] = quat.y();
	    oriented_bounding_box.rot_quat[2] = quat.z();
      oriented_bounding_box.rot_quat[3] = quat.w();

      oriented_bounding_box.size.xSize = max_point_OBB.x - min_point_OBB.x;
      std::cout<<"xSize : "<<oriented_bounding_box.size.xSize<<std::endl;
      oriented_bounding_box.size.ySize = max_point_OBB.y - min_point_OBB.y;
      std::cout<<"ySize : "<<oriented_bounding_box.size.ySize<<std::endl;
      oriented_bounding_box.size.zSize = max_point_OBB.z - min_point_OBB.z;
      std::cout<<"zSize : "<<oriented_bounding_box.size.zSize<<std::endl;
   } 

  void boundingBoxWithCoeff(pcl::PointCloud<PointT> &cluster, pcl::ModelCoefficients::Ptr coefficients)
  {
    pcl::PointCloud<PointT>::Ptr projected_cluster = cluster.makeShared();
    pcl::PointCloud<PointT>::Ptr cloud_transformed (new pcl::PointCloud<PointT>);

    Eigen::Vector3f z_axis(0,0,1);
    Eigen::Vector3f plane_normal(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
    plane_normal.normalize();
    Eigen::Vector3f c = plane_normal.cross(z_axis);//z_axis.cross(plane_normal);
        c.normalize();

    double cost  = plane_normal.dot(z_axis);
    double half_theta = acos(cost)/2;
    double sin_ht = sin(half_theta);

    Eigen::Quaternion<float> q(cos(half_theta), c[0]*sin_ht, c[1]*sin_ht, c[2]*sin_ht);
    Eigen::Matrix3f R = q.toRotationMatrix();
    Eigen::Affine3f T = Eigen::Affine3f::Identity();
    T.rotate(R);

    pcl::transformPointCloud (*projected_cluster, *cloud_transformed, T);

    volume2 = cluster.size();
    pcl::getMinMax3D(*cloud_transformed, min, max);

    for(size_t j = 0; j < cloud_transformed->points.size(); j++)
        cloud_transformed->points[j].z = 1;

    oriented_bounding_box = minAreaRect(cloud_transformed);
    Eigen::Quaternion<float> q2(cos(-oriented_bounding_box.angle/2), 0, 0, sin(-oriented_bounding_box.angle/2));
    Eigen::Quaternion<float> q3 = q2*q;
    q3 = q3.inverse();

    oriented_bounding_box.center.z = min[2]+(max[2]-min[2])/2; //this should be maxheight/2+planeHeight
    oriented_bounding_box.size.zSize = fabs((float)(max[2]-min[2])); //this should be max height!

    Eigen::Vector3f X(oriented_bounding_box.center.x,oriented_bounding_box.center.y,oriented_bounding_box.center.z);
    Eigen::Vector3f Y(0,0,0);

    Y = T.inverse()*X;

    oriented_bounding_box.center.x = Y.x();
    oriented_bounding_box.center.y = Y.y();
    oriented_bounding_box.center.z = Y.z();
    oriented_bounding_box.rot_quat[0] = q3.x();
        oriented_bounding_box.rot_quat[1] = q3.y();
        oriented_bounding_box.rot_quat[2] = q3.z();
    oriented_bounding_box.rot_quat[3] = q3.w();
  }

  // Added by Priyanka - for planes
  void planeConst(pcl::PointCloud<PointT> &contour)
  {
     plane_cloud = contour.makeShared();

     float r = 0.0f;
     float g = 0.0f;
     float b = 0.0f;
     float n = static_cast<float> (contour.size());
     for (size_t j = 0; j < n; j++)
     {
      r += (float) (contour[j].r);
      g += (float) (contour[j].g);
      b += (float) (contour[j].b);
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

    orientedBoundingBox(cluster); // Changed by Priyanka for non-planar segmentation

    //boundingBoxWithZ(cluster);

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

     //if(use_projection)
       boundingBoxWithCoeff(cluster, coefficients);
     //else
     //  boundingBoxWithZ(cluster);

     clusterConst(cluster);

   }

  pc_cluster_features(pcl::PointCloud<PointT>::Ptr &contour){
    defaultConst();

    pcl::compute3DCentroid(*contour, centroid);

    orientedBoundingBox(*contour);

    planeConst(*contour);

    orientedBoundingBox(*contour);
  }

  ~pc_cluster_features(){}

  //Box3D aligned_bounding_box;     
  Box3D oriented_bounding_box;
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
  //float bb_orientation; //wrt to the table, use a complex number?

  float compactness; // volume/bb_volume need a better name?
  float av_ratio;   //  area/volume

  float bb_volume;
  float bb_area;

  float objectBuffer[5000];

  pcl::PointCloud<PointT>::Ptr cloud;
  pcl::PointCloud<PointT>::Ptr plane_cloud;
  pcl::PointCloud<PointT>::Ptr parent_cloud;
  pcl::PointIndices indices; //indices in the parent point cloud
  pcl::PointCloud<pcl::Normal> normals;
  
  bool setViewpointHist = true;                // boolean to check if viewpoint Histogram is being computed and set (Default - true)
  pcl::PointCloud<pcl::VFHSignature308> vfhs;// (new pcl::PointCloud<pcl::VFHSignature308> ());
  
  bool setShapeHist = true;                   // boolean to check if shape histogram is being computed and set (Default - true)
  pcl::PointCloud<pcl::VFHSignature308> cvfhs; // (new pcl::PointCloud<pcl::VFHSignature308> ());
  pcl::PointCloud<pcl::FPFHSignature33> fpfhs; //(new pcl::PointCloud<pcl::FPFHSignature33> ());
  
  // Normalized cvfh feature vector?
  //std::vector<double> histogram_cvfh_vector (308);
  
  // Normalized fpfh feature vector?
  //std::vector<double> histogram_fpfh_vector (308); 
  
  // Normalized hue-saturation histogram
  bool setColorHist = true;              // boolean to check if color Histogram is being computed and set (Default - true)
  std::vector<double> histogram_hs;

  // Other features (default is set to false)
  bool setOtherFeatures = false;
  std::vector<double> otherFeatures;               // Vector to store the other features
  
  //Image based features??? blob features, color histogram? using opencv

  void fill()
  {
    oriented_bounding_box.calculateProperties();
    //aligned_bounding_box.fillQuatGivenAxisAngle();

    //bb_orientation = aligned_bounding_box.angle;
    bb_volume = oriented_bounding_box.volume;
    bb_area = oriented_bounding_box.area;
    bb_aspect_ratio = oriented_bounding_box.aspect_ratio;
    rotation_quat = Eigen::Quaternionf(oriented_bounding_box.rot_quat);
  
    //NOTE: compactness is not valid for planes
    compactness = (float)bb_volume/volume2;
    //where is the real area/real volume?
    av_ratio = bb_area/bb_volume;

    isFilled = true;
  }

  void fillFeatureContainer (float out_features[], int start_index = 0)
  {
    int feature_counter = start_index;

    out_features[feature_counter++] = numFeatures;

    out_features[feature_counter++] = oriented_bounding_box.center.x;
    out_features[feature_counter++] = oriented_bounding_box.center.y;
    out_features[feature_counter++] = oriented_bounding_box.center.z;
    //out_features[feature_counter++] = oriented_bounding_box.angle;

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

    out_features[feature_counter++] = oriented_bounding_box.size.xSize;
    out_features[feature_counter++] = oriented_bounding_box.size.ySize;
    out_features[feature_counter++] = oriented_bounding_box.size.zSize;

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

typedef pcl::PointCloud<PointT> PointCloudT;
typedef unsigned int uint;

// Histogram of hue and saturation
class HSHistogram{      
	private:
		int dim;
	public:
		HSHistogram(int dim):dim(dim){ }         // empty constructor -> just sets the dimension of the histogram
		
	void computeHistogram(PointCloudT &cloud, std::vector<std::vector<uint> > &hist2, std::vector<double> &hist2_double_vector){
		int cloud_size = cloud.points.size();
		
		for(int i = 0; i < cloud_size; i++){
            int r = (int)(double)(cloud.points[i].r);
            int g = (int)(double)(cloud.points[i].g);
            int b = (int)(double)(cloud.points[i].b);
            
            // convert the rgb value to HS value
            std::pair<double, double> HS = rgb2hue(r,g,b);
            //std::cout<<"Value of hue: "<<std::get<0>(HS)<< "    sat: "<<std::get<1>(HS)<<std::endl;
            
			// Get the HS bins
			double round = (double)360 / dim;              // As the range of Hue is from 0-359
			int h = (int) (std::get<0>(HS) / round);       // Get a bin for hue from 0-15
			int s = (int) (std::get<1>(HS) * (dim-1));     // Get a bin for saturation from 0-15
			
			//std::cout<<"Value of h: "<<h<< "    s: "<<s<<std::endl;
            hist2[h][s] = hist2[h][s]+1;
            
            //std::cout<<"\tValue after updating: "<<hist2[h][s] << " ";
            //std::cout<< h << " " << s << std::endl;
		}
		
	   // Convert to a double vector
        for (int i = 0; i < dim; i++) {
            for (int j = 0; j < dim; j++) {
				hist2_double_vector[i * dim + j] = hist2[i][j];
            }
        }
        
        for (int i = 0; i < hist2_double_vector.size(); i++) {
            hist2_double_vector[i] /= (double) cloud_size;
        }
	}
		
	std::pair<double, double> rgb2hue(int r, int g, int b){
	  const unsigned char max = std::max (r, std::max (g, b));
	  const unsigned char min = std::min (r, std::min (g, b));

	  float hue;
	  float sat;

	  const float diff = static_cast <float> (max - min);

	  if (max == 0) // division by zero
	  {
		sat = 0;
		//hue undefined! set to your favorite value (any value with zero saturation will produce black)
		hue = 0;
	  }
	  else
	  {
		sat = diff/max;

		if (min == max) // diff == 0 -> division by zero
		{
		 sat = 0;
		 //hue undefined! set to your favorite value (black)
		 hue = 0;
		}
		else
		{
		  if      (max == r) hue = 60.f * (      static_cast <float> (g - b) / diff);
		  else if (max == g) hue = 60.f * (2.f + static_cast <float> (b - r) / diff);
		  else    hue = 60.f * (4.f + static_cast <float> (r - g) / diff); // max == b

		  if (hue < 0.f) hue += 360.f;
		}
	  }

	  //if (sat < ridiculous_global_variables::saturation_threshold && ridiculous_global_variables::ignore_low_sat)
		//hue = ridiculous_global_variables::saturation_mapped_value; //hackzz oh the hackz
		
		// Return both hue and saturation as a pair
		std::pair <double, double> returnHS = std::make_pair(hue,sat);

	  return returnHS;
	}
};

// Helper class to help with colour histograms
/*class ColorHistogram {
  private:
    //std::vector<std::vector<std::vector<uint> > > hist3;
    int dim;
  public:
    /*ColorHistogram(int dim):dim(dim) {
        cloud_size = 0;
        hist3.resize(dim);
        for (int i = 0; i < dim; i++) {
            hist3[i].resize(dim);
            for (int j = 0; j < dim; j++) {
                hist3[i][j].resize(dim);
                std::fill( hist3[i][j].begin(), hist3[i][j].end(), 0 );
            }
        }
    }*/
    
    //ColorHistogram(int dim):dim(dim){ }  // empty constructor

  /*void computeHistogram(PointCloudT &cloud, std::vector<std::vector<std::vector<uint> > > &hist3, std::vector<double> &hist3_double_vector) {
        //ROS_INFO("Computing color histogram...");
        int cloud_size = cloud.points.size();
        for (int i = 0; i < cloud_size; i++) {
            // Max value of 255. We want 256 because we want the rgb division to result in
            // a value always slightly smaller than the dim due to index starting from 0
            double round = (double)256 / dim;
            int r = (int)((double)(cloud.points[i].r) / round);
            int g = (int)((double)(cloud.points[i].g) / round);
            int b = (int)((double)(cloud.points[i].b) / round);
            
            //std::cout<<"\nValue before updating: "<<hist3[r][g][b];
            
            hist3[r][g][b] = hist3[r][g][b]+1;
            
            //std::cout<<"\tValue after updating: "<<hist3[r][g][b] << " ";
            //std::cout<< r << " " << g << " " << b << " "  << std::endl;
            /*if (i==cloud_size-1)
            {
				//std::cout << hist3.size()
				std::cout<<"Value before exit:" << hist3[33][0][1] << " ";
			}*/
        /*}
        
        std::cout<<"\t\tRandom value:" << hist3[33][0][1] << " ";
        
        // Convert to a double vector
        std::cout<<"Dim: "<<dim;
        int i_offset = dim * dim;
        int j_offset = dim;
       
        /*for (int i = 0; i < dim; i++) {
            for (int j = 0; j < dim; j++) {
                for (int k = 0; k < dim; k++) {
                    hist3_double_vector[i * i_offset + j * j_offset + k] = hist3[i][j][k];
                    //std::cout <<"\nValue of hist3: "<<hist3[i][j][k]; 
                    if (i == 33 && j == 0 && k == 1)
                    {
						std::cout <<"\nValue of hist3: "<< hist3[i][j][k] << " and the vector " << hist3_double_vector[i * i_offset + j * j_offset + k] <<std::endl;
					}
                }
            }
        }
        
        /*for (int i = 0; i < hist3_double_vector.size(); i++) {
            hist3_double_vector[i] /= (double) cloud_size;
            //hist3_double_vector[i] = 1.;
        }*/
        //return hist3_double_vector;
    //}
    
    /*
    uint get(int r, int g, int b) {
        return hist3[r][g][b];
    }
    
    std::vector<double> toDoubleVector() {
        int i_offset = dim * dim;
        int j_offset = dim;
        std::vector<double> hist3_double_vector (dim * dim * dim, 0);
        for (int i = 0; i < dim; i++) {
            for (int j = 0; j < dim; j++) {
                for (int k = 0; k < dim; k++) {
                    hist3_double_vector[i * i_offset + j * j_offset + k] = hist3[i][j][k];
                    std::cout <<"\nValue of hist3: "<<hist3[i][j][k]; 
                }
            }
        }
        return hist3_double_vector;
    }
    
    std::vector<double> toDoubleVectorNormalized() {
        std::vector<double> hist_double_vector = toDoubleVector();
        for (int i = 0; i < hist_double_vector.size(); i++) {
            hist_double_vector[i] /= cloud_size;
        }
        return hist_double_vector;
    }*/
//};
#endif /* OBJECTFEATURES_HPP_ */

