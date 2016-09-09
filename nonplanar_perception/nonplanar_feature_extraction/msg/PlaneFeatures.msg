# Features for a single plane cluster

Header header

#################################
# Raw point Related

# Cluster centroid, min, max and number of points
geometry_msgs/Vector3 points_centroid
geometry_msgs/Vector3 points_min   #<pc min x, pc min y, pc min z>
geometry_msgs/Vector3 points_max   #<pc max x, pc max y, pc max z>

# Average color (RGBA nad hue)
std_msgs/ColorRGBA rgba_color
float64 hue

#################################
#Oriented Bounding box

#position wrt sensor 
geometry_msgs/Vector3 bb_center

# Bounding box dimensions
geometry_msgs/Vector3 bb_dims


