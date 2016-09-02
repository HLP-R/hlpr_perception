/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <common.hpp>
#include <pc_segmentation.hpp>

#include <pcl/common/time.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
//#include <utils_pcl_ros.hpp>

// Set up the visualization window
boost::shared_ptr<pcl::visualization::PCLVisualizer>
cloudViewer (pcl::PointCloud<PointT>::ConstPtr cloud)
{
  boost::shared_ptr < pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> single_color (cloud, 0, 255, 0);
  viewer->addPointCloud<PointT> (cloud, single_color, "cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void
OpenNIOrganizedMultiPlaneSegmentation::removePreviousDataFromScreen (size_t prev_models_size)
{
  char name[1024];
  for (size_t i = 0; i < prev_models_size; i++)
  {
    sprintf (name, "normal_%zu", i);
    viewer->removeShape (name);

    sprintf (name, "plane_%02zu", i);
    viewer->removePointCloud (name);
  }
}

void
OpenNIOrganizedMultiPlaneSegmentation::removePreviousCLustersFromScreen (size_t num_clusters)
{
  for (size_t i = 0; i < num_clusters; i++)
  {
    sprintf (name, "cluster_%lu", i);
    viewer->removeShape (name);
  }
}

// display clusters in with color
void
OpenNIOrganizedMultiPlaneSegmentation::displayEuclideanClusters (const pcl::PointCloud<PointT>::CloudVectorType &clusters, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  unsigned char red [6] = {255,   0,   0, 255, 255,   0};
  unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
  unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
  unsigned char color[3];

  for (size_t i = 0; i < clusters.size (); i++)
  {
    color[0] = red[i%6]; color[1] = grn[i%6]; color[2] = blu[i%6];
    displayEuclideanCluster (clusters[i], viewer, color, i);
  }
}

// display each cluster given its color
void
OpenNIOrganizedMultiPlaneSegmentation::displayEuclideanCluster (const pcl::PointCloud<PointT> &cluster, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, unsigned char color[], size_t clusterNum)
{
    sprintf (name, "cluster_%lu",clusterNum);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color0(boost::make_shared<pcl::PointCloud<PointT> >(cluster),color[0],color[1],color[2]);
    if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<PointT> >(cluster),color0,name))
      viewer->addPointCloud (boost::make_shared<pcl::PointCloud<PointT> >(cluster),color0,name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
}

void
OpenNIOrganizedMultiPlaneSegmentation::displayPlane(pcl::PointCloud<PointT>::Ptr contour)
{
  size_t bs = 0;
  sprintf(name,"plane_%02zu", bs);
  pcl::visualization::PointCloudColorHandlerCustom <PointT> color2 (contour, 255,62,62);//red[i], grn[i], blu[i]);
  viewer->addPointCloud (contour, color2, name);
}


void
OpenNIOrganizedMultiPlaneSegmentation::setWorkingVolumeThresholds(float* a)
{
  for(int i = 0; i < 6; i++)
    threshs[i] = a[i];

}

//allow for reading a file
void
OpenNIOrganizedMultiPlaneSegmentation::initSegmentation(int color_seg, float distance_thresh, float color_thresh)
{
  // configure normal estimation
  ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor (0.025f); //(0.01f);//0.03f
  ne.setNormalSmoothingSize (20.0f);//15.0f//20.0f

  // create a euclidean cluster comparator
switch (color_seg)
{
case 0:
  euclidean_cluster_comparator_ = pcl::MyEuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr (new pcl::MyEuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());
  break;
case 1:
  euclidean_cluster_comparator_ = pcl::RGBEuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr (new pcl::RGBEuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());
  break;
case 2:
default:
  euclidean_cluster_comparator_ = pcl::HueEuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr (new pcl::HueEuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());
break;
}
  euclidean_cluster_comparator_->setDistanceThreshold (distance_thresh, false);
  euclidean_cluster_comparator_->setColorThreshold(color_thresh*color_thresh);

  // configure the multi plane segmentor
  mps.setMinInliers (10000); //(10000);
  mps.setAngularThreshold (0.017453 * 3.0);// ); //4.5// 3 degrees
  mps.setDistanceThreshold (0.01); //0.01 in meters
  mps.setMaximumCurvature(1000.005);//0.001

  //set up color segmentation
  tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);

  reg.setDistanceThreshold (10); //10
  reg.setPointColorThreshold (25); //6
  reg.setRegionColorThreshold (20); //5
  reg.setMinClusterSize (200); //600
  reg.setSearchMethod (tree);

  // setup vfh (this is for feature extraction and not for segmentatio, consider moving
  vfh.setSearchMethod (tree);

  sor = pcl::StatisticalOutlierRemoval<PointT>::Ptr(new pcl::StatisticalOutlierRemoval<PointT> ());
  sor->setMeanK (10);
  sor->setStddevMulThresh (1.0);
  sor->setNegative (true);


}

//isFilter: aray of 6 denoting whether to threshold xmin,xmax,ymin,ymax,zmin,zmax and limits is the corresponding limits
//probably there is a better built in way of doing this
void
OpenNIOrganizedMultiPlaneSegmentation::threshXYZ(pcl::PointCloud<PointT>::Ptr out_cloud, bool isFilter[], float limits[])
{
  for (size_t i = 0; i < out_cloud->size(); i++)
  {
          if (out_cloud->points[i].x < limits[0] && isFilter[0])
      out_cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
    else  if (out_cloud->points[i].x > limits[1] && isFilter[1])
      out_cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
    else  if (out_cloud->points[i].y < limits[2] && isFilter[2])
      out_cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
    else  if (out_cloud->points[i].y > limits[3] && isFilter[3])
      out_cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
    else  if (out_cloud->points[i].z < limits[4] && isFilter[4])
      out_cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
    else  if (out_cloud->points[i].z > limits[5] && isFilter[5])
      out_cloud->points[i].x = std::numeric_limits<float>::quiet_NaN();
  }
}

std::vector<int> indices;
void OpenNIOrganizedMultiPlaneSegmentation::preProcPointCloud(pcl::PointCloud<PointT>::Ptr filtered_cloud, bool noiseFilter)
{
  bool isFilter[] = {true, true, true, true, true, true};
  threshXYZ(filtered_cloud, isFilter, threshs);

  if(noiseFilter)
  {
	  sor->setInputCloud(filtered_cloud);
	  sor->filter(indices);//filter(*filtered_cloud);

	  for (size_t i = 0; i < indices.size(); i++)
	  {
		filtered_cloud->points[indices[i]].z = std::numeric_limits<float>::quiet_NaN();
	  }
  }
}

void
OpenNIOrganizedMultiPlaneSegmentation::planeExtract(
    pcl::PointCloud<PointT>::Ptr filtered_cloud,
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions,
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud,
    std::vector<pcl::ModelCoefficients> &model_coefficients,
    std::vector<pcl::PointIndices> &inlier_indices,
    pcl::PointCloud<pcl::Label>::Ptr labels,
    std::vector<pcl::PointIndices> &label_indices,
    std::vector<pcl::PointIndices> &boundary_indices,
    bool estimate_normals
    )
{
  regions.clear();

  if(estimate_normals)
  {
    // Estimate normals
    double normal_start = pcl::getTime ();
    ne.setInputCloud (filtered_cloud);
    ne.compute (*normal_cloud);
    double normal_end = pcl::getTime ();
    if(verbose)
      std::cout << "Normal Estimation took " << double (normal_end - normal_start) << std::endl;
  }

// Segment planes
  double plane_extract_start = pcl::getTime ();
  mps.setInputNormals (normal_cloud);
  mps.setInputCloud (filtered_cloud);
  mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
  double plane_extract_end = pcl::getTime ();
  if(verbose)
    std::cout << "Plane extraction took " << double (plane_extract_end - plane_extract_start) << std::endl;
}

float OpenNIOrganizedMultiPlaneSegmentation::getClosestPlaneModel(std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions, int &closestIndex)
{
  // get the closest plane
  int nRegions = 0;
  float closestDepth = 10.0;//MAX_Z_THRESH;
  closestIndex = -1;
  if (verbose)
    std::cout<<"Found " << regions.size() << " regions" << std::endl;
  for (size_t i = 0; i < regions.size (); i++)
  {
    Eigen::Vector3f centroid = regions[i].getCentroid ();
    Eigen::Vector4f model = regions[i].getCoefficients ();
    if (model[3] < closestDepth)
    {
      closestDepth = model[3];
      closestIndex = i;
    }
    nRegions++;
  }

  pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);

  return closestDepth;
}

void
OpenNIOrganizedMultiPlaneSegmentation::segmentPointCloud(pcl::PointCloud<PointT>::Ptr filtered_cloud,
                                                         pcl::PointCloud<PointT>::CloudVectorType &clusters,
                                                         std::vector<pcl::PointCloud<pcl::Normal> > &cluster_normals,
                                                         pcl::PointCloud<PointT>::Ptr &contour,
                                                         Eigen::Vector4f &used_plane_model,
							 std::vector<std::vector<int>> &clusterIndices
                                                         )
{
  std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);

  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;
  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;
  planeExtract(filtered_cloud, regions, normal_cloud,model_coefficients,inlier_indices,labels,label_indices,boundary_indices);

  unsigned int min_cluster_size = 30;

// Segment objects
  double segment_start = pcl::getTime ();
  //std::vector<size_t> clusterIndices;
  if (regions.size () > 0)
  {
    Eigen::Vector3f used_plane_centroid;
    int closest_index;
    float closestDepth = getClosestPlaneModel(regions, closest_index);
    used_plane_centroid = regions[closest_index].getCentroid ();
    used_plane_model    = regions[closest_index].getCoefficients ();

    if(contour != NULL)
      contour->points = regions[closest_index].getContour();

    std::vector<bool> plane_labels;
    plane_labels.resize (label_indices.size (), false);
    for (size_t i = 0; i < label_indices.size (); i++)
      if (label_indices[i].indices.size () > MIN_PLANE_SIZE)
        plane_labels[i] = true;

    euclidean_cluster_comparator_->setInputCloud (filtered_cloud);
    euclidean_cluster_comparator_->setLabels (labels);
    euclidean_cluster_comparator_->setExcludeLabels (plane_labels);

    pcl::PointCloud<pcl::Label> euclidean_labels;
    std::vector<pcl::PointIndices> euclidean_label_indices;
    pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
    euclidean_segmentation.setInputCloud (filtered_cloud);
    euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

    for (size_t i = 0; i < euclidean_label_indices.size (); i++)
    {
      if (euclidean_label_indices[i].indices.size () > MIN_CLUSTER_SIZE)
      {
        pcl::PointCloud<PointT> cluster;
        pcl::PointCloud<pcl::Normal> cluster_normal;

        pcl::copyPointCloud (*filtered_cloud,euclidean_label_indices[i].indices,cluster);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(cluster, centroid);
        // skip clusters that are not on the viewpoint side of the table
        float tmp = used_plane_model[0]*centroid[0]+used_plane_model[1]*centroid[1]+used_plane_model[2]*centroid[2]+used_plane_model[3];
        //if (tmp < 0)
        //  continue;

        if(cluster.size() < min_cluster_size)
          continue;

        clusters.push_back (cluster);
        clusterIndices.push_back(euclidean_label_indices[i].indices); // Store indices to remove points
        pcl::copyPointCloud (*normal_cloud, euclidean_label_indices[i], cluster_normal);
        cluster_normals.push_back(cluster_normal);
      }
    }
    if(verbose)
      PCL_INFO ("Got %d euclidean clusters!\n", clusters.size ());
  }
  else 
  {
    std::cout << "No planar regions found!" << std::endl;
  };

  double segment_end = pcl::getTime();
  if(verbose)
    std::cout << "Segmentation took " << double (segment_end - segment_start) << std::endl;
}


int OpenNIOrganizedMultiPlaneSegmentation::processOnce (
		pcl::PointCloud<PointT>::ConstPtr prev_cl,
		pcl::PointCloud<PointT>::CloudVectorType &clustersOut,
		std::vector<pcl::PointCloud<pcl::Normal> > &clustersOutNormals,
		Eigen::Vector4f &plane,
		std::vector<std::vector<int>> &clusterIndices,
		bool preProc, bool merge_clusters, bool viewer_enabled, bool noiseFilter)
{
  if(!prev_cl)
    return 0;

  static size_t prev_models_size = 0;
  static size_t prev_cluster_num = 0;
  double start = pcl::getTime();

  pcl::PointCloud<PointT>::Ptr filtered_prev_cloud(new pcl::PointCloud<PointT>(*prev_cl));
  cloud_mutex.unlock ();
  if(preProc)
    preProcPointCloud(filtered_prev_cloud, noiseFilter);

  // variables for the used plane contours
  pcl::PointCloud<PointT>::CloudVectorType clusters;
  std::vector<pcl::PointCloud<pcl::Normal> > cluster_normals;
  pcl::PointCloud<PointT>::CloudVectorType merged_clusters;
  std::vector<pcl::PointCloud<pcl::Normal> > merged_normals;
  pcl::PointCloud<PointT>::CloudVectorType *used_clusters;
  std::vector<pcl::PointCloud<pcl::Normal> > *used_cluster_normals;
  pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);

  segmentPointCloud(filtered_prev_cloud, clusters, cluster_normals, contour, plane, clusterIndices);

  if(clusters.empty())
  {
    if(viewer_enabled)
    {
      if (!viewer->updatePointCloud<PointT> (filtered_prev_cloud, "cloud"))
          viewer->addPointCloud<PointT> (filtered_prev_cloud, "cloud");
      viewer->removeShape ("plane_00");
      //displayPlane(contour);

      // clear the visualizer
      removePreviousDataFromScreen (prev_models_size);
      removePreviousCLustersFromScreen(prev_cluster_num);
    }
    return -1;
  }
  clustersOut.clear();
  used_clusters = &clusters;
  used_cluster_normals = &cluster_normals;
 
  for(int i = 0; i < clusters.size(); i++) 
    clustersOut.push_back(clusters[i]);
  for(int i = 0; i < cluster_normals.size(); i++)
    clustersOutNormals.push_back(cluster_normals[i]);

  if(verbose)
    std::cout << "Number of clusters of interest: " << used_clusters->size() << std::endl;

  if(viewer_enabled)
  {
    if (!viewer->updatePointCloud<PointT> (filtered_prev_cloud, "cloud"))
      viewer->addPointCloud<PointT> (filtered_prev_cloud, "cloud");

    // clear the visualizer
    removePreviousDataFromScreen (prev_models_size);
    removePreviousCLustersFromScreen(prev_cluster_num);

    // Draw Visualization
    displayPlane(contour);
    displayEuclideanClusters (*used_clusters, viewer);
  }

  prev_models_size = used_clusters->size();
  prev_cluster_num = used_clusters->size();

  double end = pcl::getTime();
  if(verbose)
    std::cout << "Processing loop took " << double (end - start) << std::endl;

  return 0;
}
