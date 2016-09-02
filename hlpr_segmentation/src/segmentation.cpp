/*
 * ircp_loop.cpp
 *
 *  Created on: Sep 4, 2014
 *      Author: baris
 */

#include <pc_segmentation.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/openni_grabber.h>

#include <utils.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <hlpr_segmentation/SegmentedClusters.h>
#include <pcl/filters/extract_indices.h>
//#include <utils_pcl_ros.hpp>

#include <k2g.h>

#include <signal.h>

pcl::PointCloud<PointT>::ConstPtr prev_cloud;
pcl::PointCloud<PointT> cloud_store;
boost::mutex cloud_mutex;
boost::mutex imageName_mutex;
bool writePCD2File = false;
char imageName[100];
bool gotFirst = false;
bool interrupt = false;
const char *clusterOutRostopic = "/beliefs/clusters";
const char *normalOutRostopic = "/beliefs/normals";
const char *planeOutRostopic = "/beliefs/plane";
const char *segOutRostopic = "/beliefs/clusters";
bool viz_ = false;

void
cloud_cb_ros_ (const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PCLPointCloud2 pcl_pc;

  if (msg->height == 1){
    sensor_msgs::PointCloud2 new_cloud_msg;
    new_cloud_msg.header = msg->header;
    new_cloud_msg.height = 480;
    new_cloud_msg.width = 640;
    new_cloud_msg.fields = msg->fields;
    new_cloud_msg.is_bigendian = msg->is_bigendian;
    new_cloud_msg.point_step = msg->point_step;
    new_cloud_msg.row_step = 20480;
    new_cloud_msg.data = msg->data;
    new_cloud_msg.is_dense = msg->is_dense;

    pcl_conversions::toPCL(new_cloud_msg, pcl_pc);
  }
  else
    pcl_conversions::toPCL(*msg, pcl_pc);

  pcl::PointCloud<PointT> cloud;
  pcl::fromPCLPointCloud2(pcl_pc, cloud);
  pcl::copyPointCloud(cloud, cloud_store);

  cloud_mutex.lock ();
  prev_cloud = cloud.makeShared();
  if(writePCD2File)
  {
    pcl::PointCloud<PointT>::Ptr saved_cloud(new pcl::PointCloud<PointT>(*prev_cloud));
    std::cout << imageName << std::endl;
    cloud_mutex.unlock ();
    imageName_mutex.lock();
    pcl::io::savePCDFile(imageName, *saved_cloud);
    imageName_mutex.unlock();
    writePCD2File = false;
  }
  else
    cloud_mutex.unlock ();

  gotFirst = true;
}

void interruptFn(int sig) {
  interrupt = true;
}

void
cloud_cb_direct_ (const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  cloud_mutex.lock ();
  prev_cloud = cloud;
  if(writePCD2File)
  {
    pcl::PointCloud<PointT>::Ptr saved_cloud(new pcl::PointCloud<PointT>(*prev_cloud));
    std::cout << imageName << std::endl;
    cloud_mutex.unlock ();
    imageName_mutex.lock();
    pcl::io::savePCDFile(imageName, *saved_cloud);
    imageName_mutex.unlock();
    writePCD2File = false;
  }
  else
    cloud_mutex.unlock ();

  gotFirst = true;
}

inline void
fake_cloud_cb_kinectv2_ (const pcl::PointCloud<PointT>::ConstPtr& cloud)
{
  prev_cloud = cloud;
  if(writePCD2File)
  {
    pcl::PointCloud<PointT>::Ptr saved_cloud(new pcl::PointCloud<PointT>(*prev_cloud));
    std::cout << imageName << std::endl;
    cloud_mutex.unlock ();
    imageName_mutex.lock();
    pcl::io::savePCDFile(imageName, *saved_cloud);
    imageName_mutex.unlock();
    writePCD2File = false;
  }
}

int
main (int argc, char **argv)
{
  std::cout << "main function" << std::endl;
  pcl::Grabber* interface;
  ros::NodeHandle *nh;
  ros::Subscriber sub;
  ros::Publisher clusterPub, normalPub, planePub,msgPub;

  bool spawnObject = true;

  K2G *k2g;
  processor freenectprocessor = OPENGL;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;

  std::cout << "ros node initialized" << std::endl;
  ros::init(argc, argv, "hlpr_segmentation",ros::init_options::NoSigintHandler);
  nh = new ros::NodeHandle("~");
 // nh->getParam("segmentation/viz", viz_);
 // std::cout<<"viz is set to " << viz_ << endl;

  parsedArguments pA;
  if(parseArguments(argc, argv, pA, *nh) < 0)
    return 0;

  viz_ = pA.viz;
  ridiculous_global_variables::ignore_low_sat       = pA.saturation_hack;
  ridiculous_global_variables::saturation_threshold = pA.saturation_hack_value;
  ridiculous_global_variables::saturation_mapped_value = pA.saturation_mapped_value;

  OpenNIOrganizedMultiPlaneSegmentation multi_plane_app;

  pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr ncloud_ptr (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Label>::Ptr label_ptr (new pcl::PointCloud<pcl::Label>);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  multi_plane_app.initSegmentation(pA.seg_color_ind, pA.ecc_dist_thresh, pA.ecc_color_thresh);
  if(viz_) {
    viewer = cloudViewer(cloud_ptr);
    multi_plane_app.setViewer(viewer);
  }

 // float workSpace[] = {-0.3,0.4,-0.25,0.35,0.3,2.0};
  float workSpace[] = {-0.55,0.5,-0.2,0.45,0.3,2.0};
  multi_plane_app.setWorkingVolumeThresholds(workSpace);

  //if(pA.output_type == comType::cROS)
  //{
    //pub = nh->advertise<pc_segmentation::PcFeatures>(outRostopic,5);
 //   clusterPub = nh->advertise<pcl::PointCloud<pcl::PointXYZRGB>>(clusterOutRostopic,5);
 //   normalPub = nh->advertise<pcl::PointCloud<pcl::Normal>>(normalOutRostopic,5);
 //   planePub = nh->advertise<std_msgs::Float32MultiArray>(planeOutRostopic,5);
    msgPub = nh->advertise<hlpr_segmentation::SegmentedClusters>(segOutRostopic,5);
 // }

  switch (pA.pc_source)
  {
    case 0:
    {
      std::cout << "Using ros topic as input" << std::endl;
      sub = nh->subscribe<sensor_msgs::PointCloud2>(pA.ros_topic, 1, cloud_cb_ros_);
      break;
    }
    case 1:
    {
      std::cout << "Using the openni device" << std::endl;

      interface = new pcl::OpenNIGrabber ();

      boost::function<void(const pcl::PointCloud<PointT>::ConstPtr&)> f = boost::bind (&cloud_cb_direct_,_1);
      boost::signals2::connection c = interface->registerCallback (f);

      interface->start ();
      break;
    }
    case 2:
    default:
    {
      std::cout << "Using kinect v2" << std::endl;

      freenectprocessor = static_cast<processor>(pA.freenectProcessor);

      k2g = new K2G(freenectprocessor, true);
      cloud = k2g->getCloud();
      prev_cloud = cloud;
      gotFirst = true;
      break;
    }
  }

  signal(SIGINT, interruptFn);
  while (!interrupt && (!viz_ || !viewer->wasStopped ()))
  {
    if(pA.ros_node)
    {
      ros::spinOnce();
    }
    if(viz_)
      viewer->spinOnce(20);
    if(!gotFirst)
      continue;
    int selected_cluster_index = -1;
    if(cloud_mutex.try_lock ())
    {
      if(pA.pc_source == 2)
      {
        cloud = k2g->getCloud();
        fake_cloud_cb_kinectv2_(cloud);
      }

  	  if(viz_ && pA.justViewPointCloud)
  	  {
  		pcl::PointCloud<PointT>::Ptr filtered_prev_cloud(new pcl::PointCloud<PointT>(*prev_cloud));
  		multi_plane_app.preProcPointCloud(filtered_prev_cloud);
  		if (!viewer->updatePointCloud<PointT> (filtered_prev_cloud, "cloud"))
          {
              viewer->addPointCloud<PointT> (filtered_prev_cloud, "cloud");
          }
          selected_cluster_index = -1;
  	  }
  	  else
  	  {
		pcl::PointCloud<PointT>::CloudVectorType clusters;
            	std::vector<pcl::PointCloud<pcl::Normal> > clusterNormals;
		Eigen::Vector4f plane;
		std::vector<size_t> clusterIndices;
		std::vector<std::vector<int>> clusterIndicesStore;

  		selected_cluster_index = multi_plane_app.processOnce(prev_cloud,clusters,clusterNormals,plane,clusterIndicesStore,
			pA.pre_proc,
  		 	pA.merge_clusters, viz_, pA.filterNoise); //true is for the viewer
		hlpr_segmentation::SegmentedClusters msg;
		msg.header.stamp = ros::Time::now();

		// Pull out the cluster indices and put in msg
		for (int ti = 0; ti < clusterIndicesStore.size(); ti++){
		  hlpr_segmentation::ClusterIndex cluster_idx_msg;
		  for (int j = 0; j < clusterIndicesStore[ti].size(); j++){
		    std_msgs::Int32 temp_msg;
		    temp_msg.data = clusterIndicesStore[ti][j];
		    cluster_idx_msg.indices.push_back(temp_msg);
		  }
		  msg.cluster_ids.push_back(cluster_idx_msg);
		  //clusterIndices.insert(clusterIndices.end(),clusterIndicesStore[ti].begin(), clusterIndicesStore[ti].end()); // For removing ALL cluster points
		}

		for(int i = 0; i < clusters.size(); i++) {
		    sensor_msgs::PointCloud2 out;
		    pcl::PCLPointCloud2 tmp;
		    pcl::toPCLPointCloud2(clusters[i], tmp);
		    pcl_conversions::fromPCL(tmp, out);
		    msg.clusters.push_back(out);
		}

		for(int i = 0; i < clusterNormals.size(); i++) {
		    sensor_msgs::PointCloud2 out;
		    pcl::PCLPointCloud2 tmp;
		    pcl::toPCLPointCloud2(clusterNormals[i], tmp);
		    pcl_conversions::fromPCL(tmp, out);
		    msg.normals.push_back(out);
		}

           	std_msgs::Float32MultiArray planeMsg;
            	planeMsg.data.clear();
            	for(int i = 0; i < 4; i++)
                    planeMsg.data.push_back(plane[i]);
	        //planePub.publish(planeMsg);
		msg.plane = planeMsg;
		
		msgPub.publish(msg);

	    /*clusterPub.publish(clusters[0]);
	    normalPub.publish(clusterNormals[0]);
	    std_msgs::Float32MultiArray planeMsg;
	    planeMsg.data.clear();
	    for(int i = 0; i < 4; i++)
		planeMsg.data.push_back(plane[i]);
	    planePub.publish(planeMsg);*/
  	  }

      cloud_mutex.unlock();

      /*the z_thresh may result in wrong cluster to be selected. it might be a good idea to do
       * some sort of mean shift tracking, or selecting the biggest amongst the candidates (vs choosing the most similar color)
       * or sending all the plausible ones to c6 and decide there
       */
    }

    if(selected_cluster_index < 0)
      continue;
  }
  if(pA.pc_source == 1)
  {
    interface->stop ();
    delete interface;
  }
    delete nh;

    ros::shutdown();
  return 0;
}
