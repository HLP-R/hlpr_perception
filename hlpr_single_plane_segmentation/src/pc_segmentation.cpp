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
RansacSinglePlaneSegmentation::removePreviousDataFromScreen (size_t prev_models_size)
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
RansacSinglePlaneSegmentation::removePreviousCLustersFromScreen (size_t num_clusters)
{
    for (size_t i = 0; i < num_clusters; i++)
    {
        sprintf (name, "cluster_%lu", i);
        viewer->removeShape (name);
    }
}

// display clusters in with color
    void
RansacSinglePlaneSegmentation::displayEuclideanClusters (const pcl::PointCloud<PointT>::CloudVectorType &clusters, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
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
RansacSinglePlaneSegmentation::displayEuclideanCluster (const pcl::PointCloud<PointT> &cluster, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, unsigned char color[], size_t clusterNum)
{
    sprintf (name, "cluster_%lu",clusterNum);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color0(boost::make_shared<pcl::PointCloud<PointT> >(cluster),color[0],color[1],color[2]);
    if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<PointT> >(cluster),color0,name))
        viewer->addPointCloud (boost::make_shared<pcl::PointCloud<PointT> >(cluster),color0,name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
}

    void
RansacSinglePlaneSegmentation::displayPlane(pcl::PointCloud<PointT>::Ptr contour)
{
    size_t bs = 0;
    sprintf(name,"plane_%02zu", bs);
    pcl::visualization::PointCloudColorHandlerCustom <PointT> color2 (contour, 255,62,62);//red[i], grn[i], blu[i]);
    viewer->addPointCloud (contour, color2, name);
}




//allow for reading a file
    void
RansacSinglePlaneSegmentation::initSegmentation(int color_seg, float distance_thresh, float color_thresh)
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

    //RANSAC plane extraction
    seg.setOptimizeCoefficients (true);
    // Mandatory
    /*
       SAC_RANSAC  = 0;
       SAC_LMEDS   = 1;
       SAC_MSAC    = 2;
       SAC_RRANSAC = 3;
       SAC_RMSAC   = 4;
       SAC_MLESAC  = 5;
       SAC_PROSAC  = 6;
       */
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC); //SAC_MSAC works too
    seg.setDistanceThreshold (0.01);
    seg.setMaxIterations(1000);

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
RansacSinglePlaneSegmentation::threshXYZ(pcl::PointCloud<PointT>::Ptr out_cloud, bool isFilter[], float limits[])
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
void RansacSinglePlaneSegmentation::preProcPointCloud(pcl::PointCloud<PointT>::Ptr filtered_cloud, bool noiseFilter)
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
RansacSinglePlaneSegmentation::planeExtract(
        pcl::PointCloud<PointT>::Ptr &filtered_cloud,
        pcl::PointCloud<PointT>::Ptr &cloud_plane,
        pcl::ModelCoefficients::Ptr &coefficients,
        pcl::PointIndices::Ptr &inliers,
        pcl::PointCloud<pcl::Normal>::Ptr normal_cloud
        )
{

    // Segment planes
    double plane_extract_start = pcl::getTime ();

    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
    downSample(filtered_cloud, 0.008f, cloud_filtered);


    int i=0, nr_points = (int) cloud_filtered->points.size ();
    //while (cloud_filtered->points.size () > 0.3 * nr_points)
    //{
    //RANSAC---------------------------------//
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    for(int itr=0; itr < 3; itr++)
    {
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () >= cloud_filtered->points.size ()*0.6) break;
    } 
    /*if (inliers->indices.size () < std::min<float>(6000,cloud_filtered->points.size () * 0.5))
      {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
      }*/

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
    //}



    double plane_extract_end = pcl::getTime ();
    if(verbose)
        std::cout << "Plane extraction (RANSAC) took " << double (plane_extract_end - plane_extract_start) << std::endl;

    // Estimate normals
    double normal_start = pcl::getTime ();
    norm_est.setInputCloud (cloud_filtered);
    norm_est.setSearchSurface (cloud_filtered);
    norm_est.setSearchMethod (pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
    norm_est.setRadiusSearch (0.02);
    norm_est.compute (*normal_cloud);
    double normal_end = pcl::getTime ();
    if(verbose)
        std::cout << "Normal Estimation (RANSAC) took " << double (normal_end - normal_start) << std::endl;

    *filtered_cloud = *cloud_filtered;

}

void
RansacSinglePlaneSegmentation::setWorkingVolumeThresholds(float* a)
{
  for(int i = 0; i < 6; i++)
    threshs[i] = a[i];

}

//segment with RANSAC
    void
RansacSinglePlaneSegmentation::segmentPointCloud(pcl::ModelCoefficients::Ptr &coefficients,
        pcl::PointCloud<PointT>::Ptr cloud_filtered,
        pcl::PointCloud<PointT>::CloudVectorType &clusters, 
        std::vector<pcl::PointCloud<pcl::Normal> > &cluster_normals,
        pcl::PointCloud<PointT>::Ptr &cloud_plane
        )
{


    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    //pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);

    planeExtract(cloud_filtered, cloud_plane, coefficients,inliers, normal_cloud);


    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr kdtree (new pcl::search::KdTree<PointT> ());
    kdtree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.01f); // 2cm
    ec.setMinClusterSize (30);
    ec.setMaxClusterSize (6000);
    ec.setSearchMethod (kdtree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal> cluster_normal;

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster, centroid);

        // skip clusters that are under the plane
        float tmp = coefficients->values[0]*centroid[0]+coefficients->values[1]*centroid[1]+coefficients->values[2]*centroid[2]+coefficients->values[3];
        if (tmp < 0)
            continue;


        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points. (Centroid:" << centroid[0] << "," << centroid[1] << "," << centroid[2] << ")" << std::endl;
        clusters.push_back(*cloud_cluster);
        pcl::copyPointCloud (*normal_cloud, it->indices, cluster_normal);
        cluster_normals.push_back(cluster_normal);
        //displayNormalCloud(cloud_filtered, normal_cloud);
        j++;
    }

}

    void
RansacSinglePlaneSegmentation::downSample(pcl::PointCloud<PointT>::Ptr &input_cloud, 
        float leaf_size, 
        pcl::PointCloud<PointT>::Ptr &output_cloud)
{
    vg.setInputCloud (input_cloud);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*output_cloud);
}



int RansacSinglePlaneSegmentation::processOnce (
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
    //variables for RANSAC
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);


    segmentPointCloud(coefficients, filtered_prev_cloud, clusters, cluster_normals, contour);

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
