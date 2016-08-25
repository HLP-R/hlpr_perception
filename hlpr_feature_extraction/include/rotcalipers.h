#ifndef ROTCALIPERS_H_
#define ROTCALIPERS_H_

#include <common.hpp>

#include <float.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <misc_structures.hpp>

typedef struct
{
    int bottom;
    int left;
    float height;
    float width;
    float base_a;
    float base_b;
}
min_area_state;

#define CALIPERS_MAXHEIGHT      0
#define CALIPERS_MINAREARECT    1
#define CALIPERS_MAXDIST        2

/*template<typename PointT>
static void
rotatingCalipers( PointT* points, int n, int mode, float* out );*/

Box3D
minAreaRect(pcl::PointCloud<PointT>::Ptr &input_cloud);

#endif
