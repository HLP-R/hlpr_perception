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
#include <cluster_processing.hpp>

#include <pcl/common/time.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <utils_pcl_ros.hpp>

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

void
OpenNIOrganizedMultiPlaneSegmentation::removeBoundingBoxesAndArrows(size_t numBoundingBoxes)
{
  for(int bb = 0; bb < numBoundingBoxes; bb++)
  {
    sprintf(name, "cube %d", bb);
    viewer->removeShape(name);
    sprintf(name, "line %d-a", bb);
    viewer->removeShape(name);
    sprintf(name, "line %d-b", bb);
    viewer->removeShape(name);
    sprintf(name, "line %d-c", bb);
    viewer->removeShape(name);
  }
  viewer->removeShape("arrow");
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

//display clusters given thier color
void
OpenNIOrganizedMultiPlaneSegmentation::displayEuclideanClusters (const pcl::PointCloud<PointT>::CloudVectorType &clusters, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, std::vector<ColorVec > &colors)
{
  unsigned char color[3];

  for (size_t i = 0; i < clusters.size (); i++)
  {
    color[0] = colors[i][0]; color[1] = colors[i][1]; color[2] = colors[i][2];
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
OpenNIOrganizedMultiPlaneSegmentation::displayRegion(std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, size_t regionIndex)
{
    pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);
    pcl::PlanarRegion<PointT> asd;

    if(regionIndex < 0)
    {
      for (size_t i = 0; i < regions.size (); i++)
      {
        Eigen::Vector3f centroid = regions[i].getCentroid ();
        Eigen::Vector4f model = regions[i].getCoefficients ();
        pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
        pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                                           centroid[1] + (0.5f * model[1]),
                                           centroid[2] + (0.5f * model[2]));
        sprintf (name, "normal_%zu", i);
        viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);
        contour->points = regions[i].getContour ();
        sprintf (name, "plane_%02zu", i);
        pcl::visualization::PointCloudColorHandlerCustom <PointT> color (contour, 255,125,125);//red[i], grn[i], blu[i]);
        viewer->addPointCloud (contour, color, name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
      }
    }
    else
    {
      Eigen::Vector3f centroid = regions[regionIndex].getCentroid ();
      Eigen::Vector4f model = regions[regionIndex].getCoefficients ();
      pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
      pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                                         centroid[1] + (0.5f * model[1]),
                                         centroid[2] + (0.5f * model[2]));
      sprintf (name, "normal_%zu", regionIndex);
      viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);
      contour->points = regions[regionIndex].getContour ();
      sprintf (name, "plane_%02zu", regionIndex);
      pcl::visualization::PointCloudColorHandlerCustom <PointT> color (contour, 255,62,62);//red[i], grn[i], blu[i]);
      viewer->addPointCloud (contour, color, name);
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
    }
}

void
OpenNIOrganizedMultiPlaneSegmentation::displayNormalCloud(pcl::PointCloud<pcl::Normal>::Ptr &normal_cloud)
{
  viewer->removePointCloud ("normals");
  viewer->addPointCloudNormals<PointT,pcl::Normal>(prev_cloud, normal_cloud, 10, 0.05f, "normals");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
}

void
OpenNIOrganizedMultiPlaneSegmentation::displayNormalCloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal_cloud)
  {
    viewer->removePointCloud ("normals");
    viewer->addPointCloudNormals<PointT,pcl::Normal>(cloud, normal_cloud, 10, 0.05f, "normals");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "normals");
  }

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
OpenNIOrganizedMultiPlaneSegmentation::displayBoundingBox(Box3D &box, int box_num)
{
    // bounding box
    pcl::ModelCoefficients cube;
    Box3D_2_MC(box,cube); // fill in the box values
    //THERE IS EIGEN ALIGNEDBOX IF YOU WANT EASIER FUNCTIONALITY
    sprintf(name, "cube %d", box_num);
    viewer->addCube(cube, name);
    Cube_2_Arrows(cube, viewer, box_num);
}

void
OpenNIOrganizedMultiPlaneSegmentation::displayBoundingBoxes(std::vector<Box3D> &fittedBoxes)
{
  for(int i = 0; i < fittedBoxes.size(); i++)
  {
    Box3D box = fittedBoxes[i];
    displayBoundingBox(fittedBoxes[i],i);
  }
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

void OpenNIOrganizedMultiPlaneSegmentation::featureExtraction(pcl::PointCloud<PointT>::CloudVectorType &clusters, std::vector<pcl::PointCloud<pcl::Normal> > &normals, std::vector<pc_cluster_features> &feats)
{
  std::vector<Box3D> fittedBoxes;
  std::vector<ColorVec > cluster_colors;
  featureExtraction(clusters,normals,feats,fittedBoxes,cluster_colors);
}

void OpenNIOrganizedMultiPlaneSegmentation::featureExtraction(
    pcl::PointCloud<PointT>::CloudVectorType &clusters,
    std::vector<pcl::PointCloud<pcl::Normal> > &normals,
    std::vector<pc_cluster_features> &feats,
    std::vector<Box3D> &fittedBoxes,
    std::vector<ColorVec > &cluster_colors)
{
  Eigen::Vector4f used_plane_model(0,0,0,0);
  featureExtraction(clusters,normals,feats,fittedBoxes,cluster_colors,used_plane_model);
}

void OpenNIOrganizedMultiPlaneSegmentation::featureExtraction(
    pcl::PointCloud<PointT>::CloudVectorType &clusters,
    std::vector<pcl::PointCloud<pcl::Normal> > &normals,
    std::vector<pc_cluster_features> &feats,
    std::vector<Box3D> &fittedBoxes,
    std::vector<ColorVec > &cluster_colors,
    Eigen::Vector4f &used_plane_model    )
{
  double start = pcl::getTime();
  double extract_start = pcl::getTime();
  pcl::PointCloud<PointT>::CloudVectorType used_clusters;
  //pcl::PointCloud<PointT>::CloudVectorType myProjectedClusters;

  cluster_colors.clear();
  fittedBoxes.clear();
  feats.clear();

  std::vector<int> indices;
  //plotter->clearPlots();

  for (int i = 0; i < clusters.size(); i++)
  {
    //double f_start = pcl::getTime ();
    Eigen::Vector4f centroid;
    pcl::PointCloud<PointT> cluster = clusters[i];
    pcl::compute3DCentroid(cluster, centroid);

    //remove points with nan normals? why would they occur?

  // add cluster to used_clusters
    used_clusters.push_back(cluster);
    indices.push_back(i);

  //Feature Extraction
    pc_cluster_features feat(cluster, normals[i], used_plane_model);

    fittedBoxes.push_back(feat.oriented_bounding_box);
    cluster_colors.push_back(feat.color);

    vfh.setInputCloud (cluster.makeShared());
    vfh.setInputNormals (normals[i].makeShared());

    //this is new:
    //vfh.setNormalizeBins(true); //so that we don't have to on the matlab side
    //vfh.setNormalizeDistance(true);

    vfh.compute(feat.vfhs);//*vfhs);

    //double f_end = pcl::getTime ();

    feats.push_back(feat);
    //std::cout << "VFH calculation took " << double (f_end - f_start) << std::endl;
  }
  double extract_end = pcl::getTime();
  if(verbose)
    std::cout << "Feature extraction took " << double (extract_end - extract_start) << std::endl;
}

std::vector<int> BFS(std::vector<bool> &adjacency_matrix, int vertex_count, int given_vertex, std::vector<bool>  &is_used)
{
  bool *mark = new bool[vertex_count]();

  std::vector<int> merge_list;

  std::queue<int> q;
  q.push(given_vertex);
  merge_list.push_back(given_vertex);
  mark[given_vertex] = true;

  while (!q.empty())
  {
    int current = q.front(); q.pop();

    for (int i = 0; i < vertex_count; ++i)
    {
      if (adjacency_matrix[current*vertex_count + i] && !mark[i])
      {
          mark[i] = true;
          is_used[i] = true;
          q.push(i);
          merge_list.push_back(i);
      }
    }
  }

  delete [] mark;

  return merge_list;
}

void
OpenNIOrganizedMultiPlaneSegmentation::mergeClusters(
    pcl::PointCloud<PointT>::CloudVectorType &clusters,
    std::vector<pcl::PointCloud<pcl::Normal> > &cluster_normals,
    std::vector<pc_cluster_features> &feats,
    pcl::PointCloud<PointT>::CloudVectorType &merged_clusters,
    std::vector<pcl::PointCloud<pcl::Normal> > &merged_normals,
    double hue_thresh, double z_thresh, double euc_thresh)
{

  int num_clusters = feats.size();

  std::vector<bool> adjacency_matrix(num_clusters*num_clusters); // merge_matrix[i][j] = merge_matrix[i*num_clusters + j];

  bool checkForFullDist = euc_thresh > 0;

  for(int i = 0; i < num_clusters; i++)
  {
    for(int j = i+1; j < num_clusters; j++)
    {
      //std::cout << feats[i].hue << " " << feats[j].hue << " " << hueDiff(feats[i].hue, feats[j].hue) << std::endl;
      bool setTrue = false;
      // the below merges based on centroid distance, fyi
      if((hueDiff(feats[i].hue, feats[j].hue) < hue_thresh))
      {
    	  if(checkForFullDist)
    	  {
    		double d = 0;
    		double a= 0;
    		for(int k = 0; k < 3; k++)
    		{
    		  a = (feats[i].centroid[k] - feats[j].centroid[k]);
    		  d += a*a;
    		}
    		d = sqrt (d);
    		if(d < euc_thresh)
    		{
    		  setTrue = true;
    		}
    	  }
    	  else if(myAbs((float)(feats[i].centroid[2] - feats[j].centroid[2])) < z_thresh)
    	  {
    		setTrue = true;
    	  }

    	  if(setTrue)
    	  {
    		adjacency_matrix[i*num_clusters+j] = true;
    		adjacency_matrix[j*num_clusters+i] = true;
    	  }
      }
    }//std::cout << std::endl;
  }

  std::vector<bool> is_used(num_clusters+1);

  std::vector<std::vector <int> > merge_list_list;

  for(int i = 0; i < num_clusters; i++)
  {
      if(is_used[i])
        continue;

      merge_list_list.push_back(BFS(adjacency_matrix, num_clusters, i, is_used));
  }

  /*for(int i = 0; i < merge_list_list.size(); i++)
  {
    for(int j = 0; j < merge_list_list[i].size(); j++)
    {
      std::cout << merge_list_list[i][j] << " " ;
    } std::cout << std::endl;
  }*/


  for(int j = 0; j < merge_list_list.size(); j++)
  {
    //merged_cluster.clear();
    //merged_cluster_normal.clear();
    merged_clusters.push_back(clusters[merge_list_list[j][0]]);
    merged_normals.push_back(cluster_normals[merge_list_list[j][0]]);


    for(int i = 1; i < merge_list_list[j].size(); i++)
    {
      merged_clusters[j] += clusters[merge_list_list[j][i]];
      merged_normals[j]  += cluster_normals[merge_list_list[j][i]];
    }
    /*
    merged_cluster += (*used_clusters)[merge_list[1]];

    merged_cluster_normal += (*used_cluster_normals)[merge_list[1]];
    */
  }

}

//a lot of objects created and destroyed
int OpenNIOrganizedMultiPlaneSegmentation::processOnce (
		pcl::PointCloud<PointT>::CloudVectorType &clusters,
		std::vector<pcl::PointCloud<pcl::Normal> > &normals,
		std::vector<pc_cluster_features> &feats, 
		Eigen::Vector4f &plane,
		double main_hue, bool merge_clusters, bool viewer_enabled, double hue_thresh, double z_thresh, double euc_thresh)
{

// misc variables, find a better way than static
  static size_t prev_models_size = 0;
  static size_t prev_cluster_num = 0;

  static int prev_boundingbox_num = 0;

  double outer_start = pcl::getTime();

  int color_index = -1;

  std::vector<int> indices;
  std::vector<Box3D> fittedBoxes;
  std::vector<ColorVec > cluster_colors;
  pcl::PointCloud<PointT>::CloudVectorType merged_clusters;
  std::vector<pcl::PointCloud<pcl::Normal> > merged_normals;
  featureExtraction(clusters, normals, feats, fittedBoxes, cluster_colors,plane);

  if(merge_clusters)
  {
    mergeClusters(clusters, normals, feats, merged_clusters, merged_normals, hue_thresh, z_thresh, euc_thresh);
    clusters = merged_clusters;
    normals = merged_normals;
 //   std::vector<Box3D> fittedBoxes;
 //   std::vector<ColorVec > cluster_colors;
    featureExtraction(clusters, normals, feats, fittedBoxes, cluster_colors,plane);
  }

  double min_diff = 1000;
  std::vector<double> hue_diffs (feats.size());

  for(int i = 0; i < feats.size(); i++)
  {
    hue_diffs[i] = hueDiff(feats[i].hue, main_hue);//abs(feats[i].hue - main_hue);
    if(hue_diffs[i] < min_diff)
    {
      min_diff =hue_diffs[i];
      color_index = i;
    }
  }
    //
 // if(verbose)
 //   std::cout << "Number of clusters of interest: " << *clusters->size() << std::endl;

  double extract_end = pcl::getTime();
  if(verbose)
    std::cout << "Feature extraction and clusters of interest detection took " << double (extract_end - outer_start) << std::endl;

  if(viewer_enabled)
  {
   // if (!viewer->updatePointCloud<PointT> (filtered_prev_cloud, "cloud"))
   // {
   //   viewer->addPointCloud<PointT> (filtered_prev_cloud, "cloud");
   // }
    // clear the visualizer
    removePreviousDataFromScreen (prev_models_size);
    removePreviousCLustersFromScreen(prev_cluster_num);
    removeBoundingBoxesAndArrows(prev_boundingbox_num);

    // Draw Visualization
    //displayPlane(contour);
    displayEuclideanClusters (clusters, viewer, cluster_colors);
    //if(displayAllBb)
     // displayBoundingBoxes(fittedBoxes);
    //else11
    //if(fittedBoxes.size() > 0)
    // displayBoundingBox(fittedBoxes[color_index],0);
  }

  prev_models_size = clusters.size();
  prev_cluster_num = clusters.size();
  prev_boundingbox_num = fittedBoxes.size();

  //cloud_mutex.unlock ();
  //double end = pcl::getTime();
  //if(verbose)
  //  std::cout << "Processing loop took " << double (end - start) << std::endl;

  double outer_end = pcl::getTime();

  if(verbose)
    std::cout << "Overall loop took " << double (outer_end - outer_start) << std::endl << std::endl;

  return color_index;
}
