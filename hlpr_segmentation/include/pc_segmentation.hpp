#ifndef PC_SEGMENTATION_HPP_
#define PC_SEGMENTATION_HPP_

#include <common.hpp>

#include <stddef.h>
#include <utility>
#include <vector>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/thread/pthread/mutex.hpp>

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Memory.h>
#include <Eigen/src/StlSupport/StdVector.h>

#include <pcl/features/integral_image_normal.h>
#include <pcl/features/vfh.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>
#include<utils.hpp>
#include<vector>
//#include<objectFeatures.hpp>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <visualization_msgs/Marker.h>
//#include <PcFeatures.h>

#include <rgb_euclidean_comparator.h>
#include <utils.hpp>
//#include <misc_structures.hpp>
//#include <objectFeatures.hpp>

#define MIN_PLANE_SIZE 10000
#define MIN_CLUSTER_SIZE 50
#define DIST_THRESH 0.03f

#ifdef SIMON
	#define MIN_Y_THRESH -0.3
	#define MAX_Y_THRESH  0.2
	#define MIN_Z_THRESH  0.8
	#define MAX_Z_THRESH  1.4
	#define MIN_X_THRESH -0.4
	#define MAX_X_THRESH  0.5
#elif defined WORLD_FRAME
        #define MIN_Y_THRESH -0.5
        #define MAX_Y_THRESH  0.5
        #define MIN_Z_THRESH  -1.1
        #define MAX_Z_THRESH  -0.6
        #define MIN_X_THRESH -0.5
        #define MAX_X_THRESH  0.5
#else
	#define MIN_Y_THRESH -0.4
	#define MAX_Y_THRESH  0.3
	#define MIN_Z_THRESH  0.6
	#define MAX_Z_THRESH  1.1
	#define MIN_X_THRESH -0.5
	#define MAX_X_THRESH  0.5
#endif

#define COLOR_SEG 2 //0 none, 1 rg, 2 within cc
#define MERGE_CLUSTERS 1 //0 none, 1 rg, 2 within cc

// Useful definitions
//typedef pcl::PointXYZRGBA PointT;

template<class T>
static inline T myAbs(T _a)
{
    return _a < 0 ? -_a : _a;
}

// Set up the visualization window
boost::shared_ptr<pcl::visualization::PCLVisualizer>
cloudViewer (pcl::PointCloud<PointT>::ConstPtr cloud);

class OpenNIOrganizedMultiPlaneSegmentation
{
private:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  pcl::PointCloud<PointT>::ConstPtr prev_cloud;
  boost::mutex cloud_mutex;

  char name[1024];
  float threshs[6];

  bool viewer_enabled;

public:
  OpenNIOrganizedMultiPlaneSegmentation () : viewer_enabled(false), verbose(true)
  {
  }
  /*~OpenNIOrganizedMultiPlaneSegmentation ()
  {
  }*/

  bool verbose;

  void setViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> in_viewer){
    viewer = in_viewer;
  }

  void removeCoordinateSystem()
  {
    viewer->removeCoordinateSystem();
  }

  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  pcl::MyEuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;
//#if COLOR_SEG==2
  //pcl::HueEuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;

//#else
//  pcl::RGBEuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;
//#endif
  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
  pcl::search::Search <PointT>::Ptr tree;

  pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh;

  pcl::RegionGrowingRGB<PointT> reg;

  pcl::StatisticalOutlierRemoval<PointT>::Ptr sor;
 // void pcl::compute3DCentroid(pcl::PointCloud<PointT> &cluster, Eigen::Vector4f centroid);

  void
  removePreviousDataFromScreen (size_t prev_models_size);

  void
  removePreviousCLustersFromScreen (size_t num_clusters);

  // display clusters in with color
  void
  displayEuclideanClusters (const pcl::PointCloud<PointT>::CloudVectorType &clusters, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

  // display each cluster given its color
  void
  displayEuclideanCluster (const pcl::PointCloud<PointT> &cluster, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, unsigned char color[], size_t clusterNum);

  void
  displayPlane(pcl::PointCloud<PointT>::Ptr contour);

  //allow for reading a file
  void
  initSegmentation(int color_seg = COLOR_SEG, float distance_thresh = 0.05, float color_thresh = 25);

  //isFilter: aray of 6 denoting whether to threshold xmin,xmax,ymin,ymax,zmin,zmax and limits is the corresponding limits
  //probably there is a better built in way of doing this
  void
  threshXYZ(pcl::PointCloud<PointT>::Ptr out_cloud, bool isFilter[], float limits[]);

  void
  preProcPointCloud(pcl::PointCloud<PointT>::Ptr filtered_cloud, bool noiseFilter = false);

  //this one has a lot of stuff, maybe merge with segmentation
  void
  planeExtract(
      pcl::PointCloud<PointT>::Ptr filtered_cloud,
      std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions,
      pcl::PointCloud<pcl::Normal>::Ptr normal_cloud,
      std::vector<pcl::ModelCoefficients> &model_coefficients,
      std::vector<pcl::PointIndices> &inlier_indices,
      pcl::PointCloud<pcl::Label>::Ptr labels,
      std::vector<pcl::PointIndices> &label_indices,
      std::vector<pcl::PointIndices> &boundary_indices,
      bool estimate_normals = true
      );

  float
  getClosestPlaneModel(std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions, int &closestIndex);

  void
  segmentPointCloud(pcl::PointCloud<PointT>::Ptr filtered_cloud,
                   pcl::PointCloud<PointT>::CloudVectorType &clusters,
                   std::vector<pcl::PointCloud<pcl::Normal> > &cluster_normals,
                   pcl::PointCloud<PointT>::Ptr &contour,
                   Eigen::Vector4f &used_plane_model,
		   std::vector<std::vector<int>> &clusterIndices);

  void
  setWorkingVolumeThresholds(float* a);

  int
  processOnce (pcl::PointCloud<PointT>::ConstPtr prev_cl,
                  pcl::PointCloud<PointT>::CloudVectorType &clustersOut,
                  std::vector<pcl::PointCloud<pcl::Normal> > &clustersOutNormals,
                  Eigen::Vector4f &plane,
		  std::vector<std::vector<int>> &clusterIndices,
		  bool preProc = true, bool merge_clusters = true,
		  bool viewer_enabled = true, bool noiseFilter = false);

};

#endif
