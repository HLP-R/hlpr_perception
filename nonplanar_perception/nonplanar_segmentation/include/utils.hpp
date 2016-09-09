#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <time.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cstdio>
#include <termios.h>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <ros/ros.h>

#define MIN(X,Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X,Y) (((X) > (Y)) ? (X) : (Y))


namespace ridiculous_global_variables
{ 
	extern bool ignore_low_sat;
	extern float saturation_threshold;
	extern float saturation_mapped_value;
}

//#define M_PI 3.1415926535898

enum comType { cNONE = 0, cIRCP, cROS };

struct parsedArguments
{
  double hue_val;
  double hue_thresh;
  double z_thresh;

  double euc_thresh;

  bool pre_proc;
  int seg_color_ind;
  bool merge_clusters;

  float ecc_dist_thresh;
  float ecc_color_thresh;

  int pc_source;   //0,1,2 -> rostopic, asus, kinect2. add file and keyboard commands at some point?
  int output_type; //0,1,2 -> none,ircp,ros
  int comm_medium; //0,1,2 -> none,ircp,ros
  bool ros_node;

  char ros_topic[300];
  
  bool displayAllBb;
  
  bool saturation_hack;
  float saturation_hack_value;
  float saturation_mapped_value;

  unsigned char robot_id;

  bool filterNoise;
  bool justViewPointCloud;
  bool viz;

  int freenectProcessor; //processor options 0,1,2 correspond to CPU, OPENCL, and OPENGL respectively

  parsedArguments() : hue_val(230.0), hue_thresh(5.0), z_thresh(0.07), euc_thresh(-1.0),
		  	  	  	  pre_proc(true),seg_color_ind(2),merge_clusters(true),
		  	  	      ecc_dist_thresh(0.05f), ecc_color_thresh(25), 
					  //use_rostopic(false), use_openni(false), use_kinectv2(true), displayAllBb(false),
					  pc_source(2), output_type(cIRCP), comm_medium(cIRCP), ros_node(false), displayAllBb(false),
		  	  	      saturation_hack(true),saturation_hack_value(0.2f), saturation_mapped_value(-1000.0f),
		  	  	      robot_id(13), freenectProcessor(2), filterNoise(false), justViewPointCloud(false), viz(true)
		  	  	      {sprintf(ros_topic,"asus_filtered"); ros_node = ros_node || (pc_source == 0) || (comm_medium == cROS) || (output_type == cROS); }
};

template <class pcType>
bool loadPCD(char *name, pcType cloud_ptr);

float inline
rgb2hue(std::vector<unsigned int>& rgb_color)
{
  const unsigned char max = std::max (rgb_color[0], std::max (rgb_color[1], rgb_color[2]));
  const unsigned char min = std::min (rgb_color[0], std::min (rgb_color[1], rgb_color[2]));

  float hue;

  if (max == 0) // division by zero
  {
    hue = 0.f; //-1??
    return hue;
  }

  const float diff = static_cast <float> (max - min);

  if (min == max) // diff == 0 -> division by zero
  {
   hue = 0;
   return hue;
  }

  if      (max == rgb_color[0]) hue = 60.f * (      static_cast <float> (rgb_color[1] - rgb_color[2]) / diff);
  else if (max == rgb_color[1]) hue = 60.f * (2.f + static_cast <float> (rgb_color[2] - rgb_color[0]) / diff);
  else                          hue = 60.f * (4.f + static_cast <float> (rgb_color[0] - rgb_color[1]) / diff); // max == b

  if (hue < 0.f) hue += 360.f;

  return hue;
}

float inline
rgb2hue(int *rgb_color)
{
  const unsigned char max = std::max (rgb_color[0], std::max (rgb_color[1], rgb_color[2]));
  const unsigned char min = std::min (rgb_color[0], std::min (rgb_color[1], rgb_color[2]));

  float hue;

  if (max == 0) // division by zero
  {
    hue = 0.f; //-1??
    return hue;
  }

  const float diff = static_cast <float> (max - min);

  if (min == max) // diff == 0 -> division by zero
  {
   hue = 0;
   return hue;
  }

  if      (max == rgb_color[0]) hue = 60.f * (      static_cast <float> (rgb_color[1] - rgb_color[2]) / diff);
  else if (max == rgb_color[1]) hue = 60.f * (2.f + static_cast <float> (rgb_color[2] - rgb_color[0]) / diff);
  else                          hue = 60.f * (4.f + static_cast <float> (rgb_color[0] - rgb_color[1]) / diff); // max == b

  if (hue < 0.f) hue += 360.f;

  return hue;
}

float inline
rgb2hue(int r, int g, int b)
{
  const unsigned char max = std::max (r, std::max (g, b));
  const unsigned char min = std::min (r, std::min (g, b));

  float hue;
  float sat;

  const float diff = static_cast <float> (max - min);

  if (max == 0) // division by zero
  {
    sat = 0;
    //hue undefined! set to your favorite value
  }
  else
  {
    sat = diff/max;

    if (min == max) // diff == 0 -> division by zero
    {
     sat = 0;
     //hue undefined! set to your favorite value
    }
    else
    {
      if      (max == r) hue = 60.f * (      static_cast <float> (g - b) / diff);
      else if (max == g) hue = 60.f * (2.f + static_cast <float> (b - r) / diff);
      else                          hue = 60.f * (4.f + static_cast <float> (r - g) / diff); // max == b

      if (hue < 0.f) hue += 360.f;
    }

  }

  if (sat < ridiculous_global_variables::saturation_threshold && ridiculous_global_variables::ignore_low_sat)
    hue = ridiculous_global_variables::saturation_mapped_value; //hackzz oh the hackz

  return hue;
}

std::pair<double, double> inline
rgb2huesat(int r, int g, int b){
  const unsigned char max = std::max (r, std::max (g, b));
  const unsigned char min = std::min (r, std::min (g, b));

  float hue;
  float sat;

  const float diff = static_cast <float> (max - min);

  if (max == 0) // division by zero
  {
	sat = 0;
	//hue undefined! set to your favorite value (any value with zero saturation will produce white)
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

float
inline hueDiff(float hue1, float hue2)
{
  //my hack for eliminating low saturation colors. in rgb2hue function, i tag them as having -1000 hue
  if(hue1 < -360.0 || hue2 < -360.0)
    return 360.0;
  float diff = abs(hue1 - hue2);
  diff = MIN(360.0-diff,diff);
  return diff;
}

struct ColorVec2 {
  unsigned char colors[3];
  unsigned char operator [](int i) const    {return colors[i];}
  unsigned char & operator [](int i) {return colors[i];}
};

typedef Eigen::Vector3f ColorVec;//Eigen::Matrix<unsigned char, 3, 1> ColorVec;

enum pathType {
  NOT_EXISTS,
  PT_DIRECTORY,
  PT_FILE
};

class timingHelper {
private:
  timespec start, end;
  timeval start1, end1;
  double fps;
  int counter;
  double sec;
  public:
  timingHelper() : counter(0), sec(0), fps(0), start(), end() {}
  void init() {clock_gettime(CLOCK_MONOTONIC, &start); counter=0;}
  //this might be better with windowing but it is easier to write it like this
  inline double check() {clock_gettime(CLOCK_MONOTONIC, &end); counter++; sec = diffSec(start,end); fps = counter/sec; return fps;}
  //getter functions get the latest in the data structure! make sure to call check before getting these
  double getFPS()      {return fps;}
  int getCount()    {return counter;}
  double getDuration() {return sec;}

  inline timespec diff(timespec start, timespec end)
  {
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
    temp.tv_sec = end.tv_sec-start.tv_sec-1;
    temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
  } else {
    temp.tv_sec = end.tv_sec-start.tv_sec;
    temp.tv_nsec = end.tv_nsec-start.tv_nsec;
  }
  return temp;
  }

  inline double diffSec(timespec start, timespec end)
  {
    timespec temp = diff(start,end);
    return temp.tv_sec + ((double)temp.tv_nsec)/(1e9);
  }

};

template <class T>
class circBuff {
private:
  int size;
  int start;
  int end;
  int count;

public:
  std::vector<T> cont;

  enum pt {
    START = -1,
    END = -2
  };

  circBuff (int initSize = 10) {
    reset();
    setSize(initSize);
  }

  void setSize(int initSize) {
    size = initSize;
    cont.resize(initSize);
  }

  void reset() {
    cont.clear();
    start = 0;
    end = 0;
    count = 0;
  };

  void push(T newElem) {
    cont[end] = newElem;
    end = (end + 1)%size;

    if(++count > size)
      count = size;
  };

  T pop() {
    if(--count < 0) {
      std::cerr << "No elements left! Returning default object type." << std::endl;
      count = 0;
      return T();
    }
    T retElem = cont[start];
    start = (start+1)%size;
    return retElem;
  }

  T getReadOnly(int idx = END)
  {
    if (idx == END)
      idx = end;
    else if (idx == START)
      idx = START;

      return cont[end];
  }

  int getSize() {
    return size;
  }

  int getCount() {
    return count;
  }
};


int nonBlockingWait(cc_t min_bytes = 0, cc_t min_time = 0);

pathType isPath(char *path);

pathType isPath(const char *path);

void makeFolder(const char *folderName);

template <class T>
void
vector2file(const std::vector<T> &vec, char *fileName = "tmp.txt", char *delim = "\n");

template <class T>
void
file2vector(std::vector<T> &vec, char *fileName = "tmp.txt", char delim = ',');

template <class T>
void
doubleVector2file(const std::vector<std::vector<T> > &vec, char *fileName = "tmp.txt", char *delimiter = ",", char *delimiter2 = "\n");

template <class T>
void
file2doubleVector(std::vector<std::vector<T> > &vec, char *fileName = "tmp.txt", char delimiter = ',', char delimiter2 = '\n');

template <class T>
void
array2file(const T *arr, int size, char *fileName = "tmp.txt", char *delim = "\n");

template <class T>
void
array2file(const T *arr, int size, std::ofstream &myfile, char *delim = "\n");

template <class T>
void
vector2file(const std::vector<T> &vec, char *fileName = "tmp.txt", char *delim = "\n")
{
  std::ofstream myfile;
  myfile.open (fileName);
  for(int i = 0; i < vec.size()-1; i++)
    myfile << vec[i] << delim;
  myfile << vec[vec.size()-1];
  myfile.close();
}

template <class T>
void
file2vector(std::vector<T> &vec, char *fileName = "tmp.txt", char delim = ',')
{
  std::ifstream myfile;
  myfile.open (fileName);
  char cNum[10] ;

  if (myfile.is_open())
  {
    while (myfile.good())
    {
        myfile.getline(cNum, 256, delim);
        vec.push_back(atoi(cNum));
    }
  }
  else
  {
    std::cout << "Error opening file";
  }

  myfile.close();
}

template <class T>
void
doubleVector2file(const std::vector<std::vector<T> > &vec, char *fileName = "tmp.txt", char *delimiter = ",", char *delimiter2 = "\n")
{
  std::ofstream myfile;
  myfile.open (fileName);
  for(int i = 0; i < vec.size(); i++)
  {
    for(int j=0; j < vec[i].size()-1; j++)
    {
      myfile << vec[i][j] << delimiter;
    }
    myfile << vec[i][vec[i].size()-1]<< delimiter2;
  }
  myfile.close();
}

template <class T>
void
file2doubleVector(std::vector<std::vector<T> > &vec, char *fileName = "tmp.txt", char delimiter = ',', char delimiter2 = '\n')
{
  std::ifstream myfile;
  myfile.open (fileName);
  char cNum[10] ;

  std::vector<T> tmp_vec;

  if (myfile.is_open())
  {
    bool isAny;
    while (myfile.good())
    {
        isAny = false;

        //first read line, then read individuals
        std::string         line;
        std::getline(myfile,line);

        std::stringstream   lineStream(line);
        std::string         cell;

        tmp_vec.clear();
        while(lineStream.getline(cNum,256,','))
        {
         isAny = true;
         tmp_vec.push_back(atoi(cNum));
        }
        if(isAny)
          vec.push_back(tmp_vec);
    }
  }
  else
  {
    std::cout << "Error opening file";
  }

  std::cout << "Read row number: " << vec.size(); std::cout << std::endl;

  myfile.close();
}

template <class T>
void
array2file(const T *arr, int size, char *fileName = "tmp.txt", char *delim = "\n")
{
  std::ofstream myfile;
  myfile.open (fileName);
  for(int i = 0; i < size; i++)
    myfile << arr[i] << delim;
  myfile.close();
}

template <class T>
void
array2file(const T *arr, int size, std::ofstream &myfile, char *delim = "\n")
{
  for(int i = 0; i < size; i++)
    myfile << arr[i] << delim;
}

void
fillInIndices(std::vector<int> &indices, int start = 0, int end = -1, bool push_back = true);

int
parseArguments(int argc, char **argv, parsedArguments &pA, ros::NodeHandle &nh);

//not my fault, c++ should have implemented reflection and lambda functions sooner
//and no i am not going to try to use rtii, boost or some other library
/*int
parseArguments2(int argc, char **argv, parsedArguments2 &pA);*/

/*enum parseType
{
  INT = 0,
  BOOL,
  FLOAT,
  DOUBLE,
  CHAR,
};

class value2parse
{
  ~value2parse()
  {
    if(val != NULL)
      free(val);
  }
public:
  void *val;

  parseType type;
  std::string name;
  std::string argument;

  value2parse(parseType t, std::string n, std::string arg, void *v = NULL) : type(t), name(n), argument(arg)
  {
    setValue(v);
  }

  void setValue(void *v)
  {
    if(v == NULL)
      val = NULL;
    else if(type == INT)
    {
      val = (int *)malloc(sizeof(int));
      *(int *)(val) = *(int *)(v);
    }
    else if(type == BOOL)
    {
        val = (bool *)malloc(sizeof(bool));
        *(bool *)(val) = *(bool *)(v);
    }
    else if(type == FLOAT)
    {
        val = (float *)malloc(sizeof(float));
        *(float *)(val) = *(float *)(v);
    }
    else if(type == DOUBLE)
    {
        val = (double *)malloc(sizeof(double));
        *(double *)(val) = *(double *)(v);
    }
    else if(type == CHAR) //lol dunno what i am doing
    {
        val = v;
    }
  }

  void setValue(const char *argv)
  {
    if(type == INT)
    {
      val = (int *)malloc(sizeof(int));
      *(int *)(val) = (atoi(argv));
    }
    else if(type == BOOL)
    {
      val = (bool *)malloc(sizeof(bool));
      *(bool *)(val) = (bool)(atoi(argv));
    }
    else if(type == FLOAT)
    {
      val = (float *)malloc(sizeof(float));
      *(float *)(val) = (float)(atof(argv));
    }
    else if(type == DOUBLE)
    {
      val = (double *)malloc(sizeof(double));
      *(double *)(val) = (atof(argv));
    }
    else if(type == CHAR) //lol dunno what i am doing
    {
        val = NULL;
    }
  }

  //std::ostream& operator<<(std::ostream& os, const value2parse& v2p)
   friend std::ostream& operator<<(std::ostream& os, const value2parse& v2p)
   {
     if(v2p.type == INT)
     {
        os << *(int *)(v2p.val);
     }
     else if(v2p.type == BOOL)
     {
         os << *(bool *)(v2p.val);
     }
     else if(v2p.type == FLOAT)
     {
         os << *(float *)(v2p.val);
     }
     else if(v2p.type == DOUBLE)
     {
         os << *(double *)(v2p.val);
     }
     else if(v2p.type == CHAR) //lol dunno what i am doing
     {
         os << *(char *)(v2p.val);
     }
       return os;
   }

};

struct parsedArguments2
{
private:
  value2parse *_hue_val;
  value2parse *_hue_thresh;
  value2parse *_z_thresh;

  value2parse *_euc_thresh;

  value2parse *_pre_proc;
  value2parse *_seg_color_ind;
  value2parse *_merge_clusters;

  value2parse *_ecc_dist_thresh;
  value2parse *_ecc_color_thresh;

public:
  double hue_val;
  double hue_thresh;

  double z_thresh;
  double euc_thresh;

  bool pre_proc;
  int seg_color_ind;
  bool merge_clusters;

  float ecc_dist_thresh;
  float ecc_color_thresh;

  std::vector<value2parse*> values;

  ~parsedArguments2()
  {
    for(int i = 0; i < values.size(); i++)
    {
      if(values[i] != NULL)
        free(values[i]);
    }
  }

  parsedArguments2()
  {
    hue_val = 230.0;
    _hue_val = new value2parse(DOUBLE, "hue value", "-v", (void *) (&hue_val));
    hue_thresh = 5.0;
    _hue_thresh = new value2parse(DOUBLE, "hue threshold", "-t", (void *) (&hue_thresh));

    z_thresh = 0.07;
    _z_thresh = new value2parse(DOUBLE, "z threshold", "-z", (void *) (&z_thresh));
    euc_thresh = 0.05;
    _euc_thresh = new value2parse(DOUBLE, "eucledian threshold", "-e", (void *) (&euc_thresh));

    pre_proc = true;
    _pre_proc = new value2parse(BOOL, "pre process", "-p", (void *) (& pre_proc));
    seg_color_ind = 2;
    _seg_color_ind = new value2parse(INT, "segmentation color index", "-c", (void *) (& seg_color_ind));
    merge_clusters = true;
    _merge_clusters = new value2parse(BOOL, "merge clusters", "-m", (void *) (& merge_clusters));

    ecc_dist_thresh = 0.05;
    _ecc_dist_thresh = new value2parse(FLOAT, "ecc distance threshold", "-dt", (float *) (& ecc_dist_thresh));
    ecc_color_thresh = 625.0;;
    _ecc_color_thresh = new value2parse(FLOAT, "ecc color threshold", "-ct", (float *) (& ecc_color_thresh));

    values.push_back(_hue_val);
    values.push_back(_hue_thresh);

    values.push_back(_z_thresh);
    values.push_back(_euc_thresh);

    values.push_back(_pre_proc);
    values.push_back(_seg_color_ind);
    values.push_back(_merge_clusters);

    values.push_back(_ecc_dist_thresh);
    values.push_back(_ecc_color_thresh);
  }

  void copyValuesToVariables()
  {
    hue_val = *(double *)_hue_val->val;
    hue_thresh = *(double *)_hue_thresh->val;

    z_thresh = *(double *)_z_thresh->val;
    euc_thresh = *(double *)_euc_thresh->val;

    pre_proc = *(bool *)_pre_proc->val;
    seg_color_ind = *(int *)_seg_color_ind->val;
    merge_clusters = *(bool *)_merge_clusters->val;

    ecc_dist_thresh = *(float *)_ecc_dist_thresh->val;
    ecc_color_thresh = *(float *)_ecc_color_thresh->val;
  }
};*/

#endif
