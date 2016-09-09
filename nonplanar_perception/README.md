Welcome to the nonplanar_perception wiki!

# Installing perception nodes
1. Follow the Kinect 2 installation tutorial [here](https://github.com/OpenKinect/libfreenect2/blob/master/README.md). Follow all the required steps, **do NOT follow the steps to install CUDA!** This is an optional step that will have you download the wrong NVIDIA drivers, and will cause you to waste a day trying to figure out why your computer has locked you out. **DON'T DO IT!**

2. Install missing packages.
  - `sudo apt-get install ros-indigo-visualization-msgs`

3. Update PCL
  - There is an issue with PCL and Boost being compiled under different c++ standards. To rebuild PCL from source:
    - `cd ~/software`
    - `git clone https://github.com/PointCloudLibrary/pcl.git`
    - `cd pcl && mkdir build && cd build`
    - `cmake ..`
    - `make -j2`
    - `sudo make -j2 install`

4. Link the perception packages to your workspace, e.g.:
   - `ln -s ~/sandbox/nonplanar_perception/ ~/vector_ws/src/`

5. Build your workspace.
   - `cd ~/vector_ws`
   - `catkin_make`
   
6. Launch the segmentation node.
   - `roslaunch nonplanar_segmentation nonplanar_seg.launch`
   
7. In another window, launch the feature extraction node.
   - `roslaunch nonplanar_feature_extraction nonplanar_ft_ex.launch`

# Parameters
- You can set any parameters using the rosparam convention in your launch file.
- You can set any of the following parameters:
  * dt: distance threshold for segmentation (float)
  * ct: color threshold for segmentation (float) (currently set a very high (e.g. 100000) threshold to do proximity only segmentation)
  * v: set hue value to select the cluster of interest (float)
  * m: set whether to do merging or not (boolean)
  * t: set hue threshold for merging
  * z: set depth threshold for merging (float)
  * p: set whether to pre process the point cloud or not (boolean)
  * c: set color segmentation options, 0,1,2 correspond to none, rgb, hue
  * src: point cloud source options 0,1,2 correspond to ROS, OPENNI, and KinectV2 respectively (int).
  * rt: name of the ros topic of the incoming poin cloud (string)
  * out: output options 0,1,2 corresponding to none, IRCP, ROS (int). IRCP support may not exist, exercise caution.
  * comm: Communication options 0,1,2 corresponding to none, IRCP, ROS (int). IRCP support may not exist, exercise caution.
  * b: set whether to display all bounding boxes or just the selected one (boolean)
  * sh: set saturation hack on/off (boolean)
  * st: set saturation threshold for the hack (float)
  * sv: set saturation value to be mapped to (float)
  * ri: set robot id for ircp  (byte)
  * pr: set the freenect2 processor options 0,1,2 correspond to CPU, OPENCL, and OPENGL respectively (int)
  * fn: filter noise, warning slows things down (bool)
  * nv: no vizualization (bool)

