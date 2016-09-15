# Raw point Related

# Cluster centroid, min, max and number of points
geometry_msgs/Vector3 points_centroid
geometry_msgs/Vector3 points_min   #<pc min x, pc min y, pc min z>
geometry_msgs/Vector3 points_max   #<pc max x, pc max y, pc max z>

bool setNumPoints                # flag is required for planes as they only have contours, 
float64 num_points               # the num_points is not applicable to planes

# Average color (RGBA and hue)
std_msgs/ColorRGBA rgba_color
float64 hue
