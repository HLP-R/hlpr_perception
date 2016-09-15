Header header

# The following two are used to store clusters of objects
sensor_msgs/PointCloud2[] clusters
sensor_msgs/PointCloud2[] normals

# The following are used to store information about planes
bool ifPlanesUsed                              # Set this flag depending on if you plan to use the planes info or 
std_msgs/Float32MultiArray[] plane_coefficients
sensor_msgs/PointCloud2[] plane_contours
