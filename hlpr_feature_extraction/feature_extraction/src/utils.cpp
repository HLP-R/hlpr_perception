/*
 * utils.cpp
 *
 *  Created on: Aug 16, 2013
 *      Author: siml
 */

#include <utils.hpp>

#include <sys/stat.h>
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <ros/ros.h>

namespace ridiculous_global_variables
{
  bool ignore_low_sat = true;
  float saturation_threshold = 0.2f;
  float saturation_mapped_value = -1000.0f;
}

int
nonBlockingWait(cc_t min_bytes, cc_t min_time)
{
  struct termios initial_settings, new_settings;
  int n;

  unsigned char key;

  tcgetattr(0, &initial_settings);

  new_settings = initial_settings;
  new_settings.c_lflag &= ~ICANON;
  new_settings.c_lflag &= ~ECHO;
  new_settings.c_lflag &= ~ISIG;
  new_settings.c_cc[VMIN] = min_bytes;
  new_settings.c_cc[VTIME] = min_time;

  tcsetattr(0, TCSANOW, &new_settings);

  //do
  //{
  n = getchar();

  /*if(flag) {
   printf("Signal received!\n");
   break;
 }*/

  /*if(n!=EOF) {
   printf("Key pressed!\n");
   break;
 }*/

  //} while(1);
  tcsetattr(0, TCSANOW, &initial_settings);

  return n;
}

pathType
isPath(char *path)
{
  if (access(path, 0) == 0)
  {
      struct stat status;
      stat(path, &status);

      if (status.st_mode & S_IFDIR)
      {
          return PT_DIRECTORY;
      }
      else
      {
          return PT_FILE;
      }
  }
  else
  {
      return NOT_EXISTS;
  }
}

pathType
isPath(const char *path)
{
  if (access(path, 0) == 0)
  {
      struct stat status;
      stat(path, &status);

      if (status.st_mode & S_IFDIR)
      {
          return PT_DIRECTORY;
      }
      else
      {
          return PT_FILE;
      }
  }
  else
  {
      return NOT_EXISTS;
  }
}

void
makeFolder(const char *folderName)
{
  if (isPath(folderName) == NOT_EXISTS)
  {
      std::cout << "Creating folder: " << folderName << std::endl;
      mkdir(folderName, 0700);
  }
  else if (isPath(folderName) == PT_FILE)
  {
      std::cerr << folderName << " refers to a file, not a folder."
          << std::endl;
      exit(-1);
  }
}

void fillInIndices(std::vector<int> &indices, int start, int end, bool push_back) //non-inclusive end!!
{
  if(end<0)
  {
    push_back = false;
    end = indices.size();
  }

  if(push_back)
    for (int i = start; i < end; i++) //memcopy?
      indices.push_back(i);
  else
    for (int i = start; i < end; i++) //memcopy?
      indices[i] = i;
}

int
parseArguments(int argc, char **argv, parsedArguments &pA, ros::NodeHandle &nh)
{
  int unknownArgNum = 0;
  pA.viz = true;
  float val = 0;
  bool bVal = false;
  int iVal = 1;
  std::string sVal;
 // for /int i = 1; i < argc; i++)
 // {
      if (nh.getParam("feature_extraction/v", val))
      {
          pA.hue_val = val;
      }
      if (nh.getParam("feature_extraction/t", val))
      {
          pA.hue_thresh = val;
      }
      if (nh.getParam("feature_extraction/z", val))
      {
          pA.z_thresh = val;
      }
      if (nh.getParam("feature_extraction/e", val))
      {
          pA.euc_thresh = val;
      }
      if (nh.getParam("feature_extraction/p", bVal))
      {
          pA.pre_proc = bVal;
      }
      if (nh.getParam("feature_extraction/c", iVal))
      {
          pA.seg_color_ind = iVal;
      }
      if (nh.getParam("feature_extraction/m", bVal))
      {
          pA.merge_clusters = bVal;
      }
      if (nh.getParam("feature_extraction/dt", val))
      {
          pA.ecc_dist_thresh = val;
      }
      if (nh.getParam("feature_extraction/ct", val))
      {
          pA.ecc_color_thresh = val;
      }
      if (nh.getParam("feature_extraction/src", iVal))
      {
    	  pA.pc_source = iVal;
      }
      if (nh.getParam("feature_extraction/out", iVal))
      {
          pA.output_type = iVal;
      }
      if (nh.getParam("feature_extraction/comm", iVal))
      {
    	  pA.comm_medium = iVal;
      }
      if (nh.getParam("feature_extraction/rt", sVal))
      {
          sprintf(pA.ros_topic, "%s", sVal.c_str());
      }
      if (nh.getParam("feature_extraction/b", iVal))
      {
          pA.displayAllBb = iVal;
      }
      if (nh.getParam("feature_extraction/sh", iVal))
      {
          pA.saturation_hack = iVal;
      }
      if (nh.getParam("feature_extraction/st", val))
      {
          pA.saturation_hack_value = val;
      }
      if (nh.getParam("feature_extraction/sv", val))
      {
          pA.saturation_mapped_value = val;
      }
      if (nh.getParam("feature_extraction/ri", iVal))
      {
          pA.robot_id = iVal;
      }
      if (nh.getParam("feature_extraction/pr", iVal))
      {
    	  pA.freenectProcessor = iVal;
      }
      if (nh.getParam("feature_extraction/fn", iVal))
      {
    	  pA.filterNoise = iVal;
      }
      if (nh.getParam("feature_extraction/nv", bVal))
      {
    	  pA.viz = !bVal;
      }
      if (nh.getParam("feature_extraction/view", iVal))
      {
    	  pA.justViewPointCloud = iVal; 
      }
      if (nh.getParam("feature_extraction/h", bVal))
      {
        std::cout << "-dt: distance threshold for feature_extraction (float)"  << std::endl;
        std::cout << "-ct: color threshold for feature_extraction (float) (currently set a very high (e.g. 100000) threshold to do proximity only feature_extraction)" << std::endl;
        std::cout << "-v: set hue value to select the cluster of interest (float)"  << std::endl;
        std::cout << "-m: set whether to do merging or not (boolean)" << std::endl;
        std::cout << "-t: set hue threshold for merging" << std::endl;
        std::cout << "-z: set depth threshold for merging (float)" << std::endl;
        std::cout << "-p: set whether to pre process the point cloud or not (boolean)" << std::endl;
        std::cout << "-c: set color feature_extraction options, 0,1,2 correspond to none, rgb, hue" << std::endl;
        std::cout << "-src: point cloud source options 0,1,2 correspond to ROS, OPENNI, and KinectV2 respectively (int)." << std::endl;
        std::cout << "-rt: name of the ros topic of the incoming poin cloud (string)" << std::endl;
        std::cout << "-out: output options 0,1,2 corresponding to none, IRCP, ROS (int). IRCP support may not exist, exercise caution." << std::endl;
        std::cout << "-comm: Communication options 0,1,2 corresponding to none, IRCP, ROS (int). IRCP support may not exist, exercise caution." << std::endl;
        std::cout << "-b: set whether to display all bounding boxes or just the selected one (boolean)" << std::endl;
        std::cout << "-sh: set saturation hack on/off (boolean)" << std::endl;
        std::cout << "-st: set saturation threshold for the hack (float)" << std::endl;
        std::cout << "-sv: set saturation value to be mapped to (float)" << std::endl;
        std::cout << "-ri: set robot id for ircp  (byte)" << std::endl;
        std::cout << "-pr: set the freenect2 processor options 0,1,2 correspond to CPU, OPENCL, and OPENGL respectively (int)" << std::endl;
        std::cout << "-fn: filter noise, warning slows things down (bool)" << std::endl;
        std::cout << "-nv: no vizualization" << std::endl;
        std::cout << "-h: this help" << std::endl;
        return -1;
      }
   /*   lse
      {
          std::cout << "Unknown argument: " << argv[i] << std::endl;
          unknownArgNum++;
      }
  }*/
  std::cout << "hue_val set to: " << pA.hue_val << std::endl;
  std::cout << "hue_thresh set to: " << pA.hue_thresh << std::endl;
  std::cout << "z_thresh set to: " << pA.z_thresh << std::endl;
  std::cout << "euc_thresh set to: " << pA.euc_thresh << std::endl;
  std::cout << "pre_proc set to: " << pA.pre_proc << std::endl;
  std::cout << "seg_color_ind set to: " << pA.seg_color_ind << std::endl;
  std::cout << "merge_clusters set to: " << pA.merge_clusters << std::endl;
  std::cout << "ecc_dist_thresh set to: " << pA.ecc_dist_thresh << std::endl;
  std::cout << "ecc_color_thresh set to: " << pA.ecc_color_thresh << std::endl;

  pA.ros_node = pA.ros_node || (pA.pc_source == 0) || (pA.comm_medium == cROS) || (pA.output_type == cROS);

  return unknownArgNum;
}

/*int
 parseArguments2(int argc, char **argv, parsedArguments2 &pA)
 {
 int unknownArgNum = 0;
 for (int i = 1; i < argc; i++)
 {
 bool isContinue = false;
 for (int j = 0; j < pA.values.size(); j++)
 {
 if(!strcmp(argv[i],pA.values[j]->argument.c_str()))
 {
 pA.values[j]->setValue(argv[++i]);
 isContinue = true;
 continue;
 }
 }
 if (isContinue)
 continue;
 if (!strcmp(argv[i],"-h"))
 {
 for (int j = 1; j < pA.values.size(); j++)
 {
 std::cout << pA.values[j]->argument << ": " << pA.values[j]->name << std::endl;
 }
 std::cout << " -h: this help" << std::endl << std::endl;
 return -1;
 }
 else
 {
 std::cout << "Unknown argument: " << argv[i] << std::endl;
 unknownArgNum++;
 }
 }

 //mae operator << frined of value2parse
 for (int j = 0; j < pA.values.size(); j++)
 {
 std::cout << pA.values[j]->name << " " << (*pA.values[j]) << std::cout << std::endl;
 }

 pA.copyValuesToVariables();
 return unknownArgNum;
 }*/

