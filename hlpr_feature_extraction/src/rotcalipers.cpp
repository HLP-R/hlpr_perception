/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
// Additional Notes: Majority of this code has been taken from the opencv library and
// and modified to be used with pcl. This code may be buggy or inoperable. Use at your 
// own risk. 
// Baris Akgun 2013
//M*/

#include <float.h>
#include <rotcalipers.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

/*F///////////////////////////////////////////////////////////////////////////////////////
//    Name:    rotatingCalipers
//    Purpose:
//      Rotating calipers algorithm with some applications
//
//    Context:
//    Parameters:
//      pt_cl->points      - convex hull vertices ( any orientation )
//      n           - number of vertices
//      mode        - concrete application of algorithm
//                    can be  CALIPERS_MAXDIST   or
//                            CALIPERS_MINAREARECT
//      left, bottom, right, top - indexes of extremal pt_cl->points
//      out         - output info.
//                    In case CALIPERS_MAXDIST it pt_cl->points to float value -
//                    maximal ySize of polygon.
//                    In case CALIPERS_MINAREARECT
//                    ((PointT*)out)[0] - corner
//                    ((PointT*)out)[1] - vector1
//                    ((PointT*)out)[2] - vector2
//
//                      ^
//                      |
//              vector2 |
//                      |
//                      |____________\
//                    corner         /
//                               vector1
//
//    Returns:
//    Notes:
//F*/

/* we will use usual cartesian coordinates */

//template<typename PointT>
static void
rotatingCalipers(pcl::PointCloud<PointT>::ConstPtr pt_cl, int n, int mode, PointT* out )
{
    float minarea = FLT_MAX;
    float max_dist = 0;
    char buffer[32] = {};
    int i, k;

    pcl::PointCloud<PointT> vect;// (new pcl::PointCloud<PointT>);

    float* inv_vect_length = new float[n];

    int left = 0, bottom = 0, right = 0, top = 0;
    int seq[4] = { -1, -1, -1, -1 };

    /* rotating calipers sides will always have coordinates
       (a,b) (-b,a) (-a,-b) (b, -a)
     */
    /* this is a first base bector (a,b) initialized by (1,0) */
    float orientation = 0;
    float base_a;
    float base_b = 0;

    float left_x, right_x, top_y, bottom_y;
    PointT pt0 = pt_cl->points[0];

    left_x = right_x = pt0.x;
    top_y = bottom_y = pt0.y;

    for( i = 0; i < n; i++ )
    {
        double dx, dy;

        if( pt0.x < left_x )
            left_x = pt0.x, left = i;

        if( pt0.x > right_x )
            right_x = pt0.x, right = i;

        if( pt0.y > top_y )
            top_y = pt0.y, top = i;

        if( pt0.y < bottom_y )
            bottom_y = pt0.y, bottom = i;

        PointT pt = pt_cl->points[(i+1) & (i+1 < n ? -1 : 0)];

        dx = pt.x - pt0.x;
        dy = pt.y - pt0.y;

        PointT tmp;
        tmp.x = dx;
        tmp.y = dy;
        vect.push_back(tmp);
        inv_vect_length[i] = (float)(1./sqrt(dx*dx + dy*dy));

        pt0 = pt;
    }

    //cvbInvSqrt( inv_vect_length, inv_vect_length, n );

    /* find convex hull orientation */
    {
        double ax = vect[n-1].x;
        double ay = vect[n-1].y;

        for( i = 0; i < n; i++ )
        {
            double bx = vect[i].x;
            double by = vect[i].y;

            double convexity = ax * by - ay * bx;

            if( convexity != 0 )
            {
                orientation = (convexity > 0) ? 1.f : (-1.f);
                break;
            }
            ax = bx;
            ay = by;
        }
        assert( orientation != 0 );
    }
    base_a = orientation;

/*****************************************************************************************/
/*                         init calipers position                                        */
    seq[0] = bottom;
    seq[1] = right;
    seq[2] = top;
    seq[3] = left;
/*****************************************************************************************/
/*                         Main loop - evaluate angles and rotate calipers               */

    /* all of edges will be checked while rotating calipers by 90 degrees */
    for( k = 0; k < n; k++ )
    {
        /* sinus of minimal angle */
        /*float sinus;*/

        /* compute cosine of angle between calipers side and polygon edge */
        /* dp - dot product */
        float dp0 = base_a * vect[seq[0]].x + base_b * vect[seq[0]].y;
        float dp1 = -base_b * vect[seq[1]].x + base_a * vect[seq[1]].y;
        float dp2 = -base_a * vect[seq[2]].x - base_b * vect[seq[2]].y;
        float dp3 = base_b * vect[seq[3]].x - base_a * vect[seq[3]].y;

        float cosalpha = dp0 * inv_vect_length[seq[0]];
        float maxcos = cosalpha;

        /* number of calipers edges, that has minimal angle with edge */
        int main_element = 0;

        /* choose minimal angle */
        cosalpha = dp1 * inv_vect_length[seq[1]];
        maxcos = (cosalpha > maxcos) ? (main_element = 1, cosalpha) : maxcos;
        cosalpha = dp2 * inv_vect_length[seq[2]];
        maxcos = (cosalpha > maxcos) ? (main_element = 2, cosalpha) : maxcos;
        cosalpha = dp3 * inv_vect_length[seq[3]];
        maxcos = (cosalpha > maxcos) ? (main_element = 3, cosalpha) : maxcos;

        /*rotate calipers*/
        {
            //get next base
            int pindex = seq[main_element];
            float lead_x = vect[pindex].x*inv_vect_length[pindex];
            float lead_y = vect[pindex].y*inv_vect_length[pindex];
            switch( main_element )
            {
            case 0:
                base_a = lead_x;
                base_b = lead_y;
                break;
            case 1:
                base_a = lead_y;
                base_b = -lead_x;
                break;
            case 2:
                base_a = -lead_x;
                base_b = -lead_y;
                break;
            case 3:
                base_a = -lead_y;
                base_b = lead_x;
                break;
            default: assert(0);
                break;
            }
        }
        /* change base point of main edge */
        seq[main_element] += 1;
        seq[main_element] = (seq[main_element] == n) ? 0 : seq[main_element];


        switch (mode)
        {
        case CALIPERS_MAXHEIGHT:
            {
                /* now main element lies on edge alligned to calipers side */

                /* find opposite element i.e. transform  */
                /* 0->2, 1->3, 2->0, 3->1                */
                int opposite_el = main_element ^ 2;

                float dx = pt_cl->points[seq[opposite_el]].x - pt_cl->points[seq[main_element]].x;
                float dy = pt_cl->points[seq[opposite_el]].y - pt_cl->points[seq[main_element]].y;
                float dist;

                if( main_element & 1 )
                    dist = (float)fabs(dx * base_a + dy * base_b);
                else
                    dist = (float)fabs(dx * (-base_b) + dy * base_a);

                if( dist > max_dist )
                    max_dist = dist;

                break;
            }
        case CALIPERS_MINAREARECT:
            /* find area of rectangle */
            {
                float ySize;
                float area;

                /* find vector left-right */
                float dx = pt_cl->points[seq[1]].x - pt_cl->points[seq[3]].x;
                float dy = pt_cl->points[seq[1]].y - pt_cl->points[seq[3]].y;

                /* dotproduct */
                float xSize = dx * base_a + dy * base_b;

                /* find vector left-right */
                dx = pt_cl->points[seq[2]].x - pt_cl->points[seq[0]].x;
                dy = pt_cl->points[seq[2]].y - pt_cl->points[seq[0]].y;

                /* dotproduct */
                ySize = -dx * base_b + dy * base_a;

                area = xSize * ySize;
                if( area <= minarea )
                {
                    float *buf = (float *) buffer;

                    minarea = area;
                    /* leftist point */
                    ((int *) buf)[0] = seq[3];
                    buf[1] = base_a;
                    buf[2] = xSize;
                    buf[3] = base_b;
                    buf[4] = ySize;
                    /* bottom point */
                    ((int *) buf)[5] = seq[0];
                    buf[6] = area;
                }
                break;
            }
        }                       /*switch */
    }                           /* for */

    switch (mode)
    {
    case CALIPERS_MINAREARECT:
        {
            float *buf = (float *) buffer;

            float A1 = buf[1];
            float B1 = buf[3];

            float A2 = -buf[3];
            float B2 = buf[1];

            float C1 = A1 * pt_cl->points[((int *) buf)[0]].x + pt_cl->points[((int *) buf)[0]].y * B1;
            float C2 = A2 * pt_cl->points[((int *) buf)[5]].x + pt_cl->points[((int *) buf)[5]].y * B2;

            float idet = 1.f / (A1 * B2 - A2 * B1);

            float px = (C1 * B2 - C2 * B1) * idet;
            float py = (A1 * C2 - A2 * C1) * idet;

            out[0].x = px;
            out[0].y = py;

            out[1].x = A1 * buf[2];
            out[1].y = B1 * buf[2];

            out[2].x = A2 * buf[4];
            out[2].y = B2 * buf[4];
        }
        break;
    case CALIPERS_MAXHEIGHT:
        {
            out[0].x = max_dist;
        }
        break;
    }

    delete [] inv_vect_length;
}

Box3D
minAreaRect(pcl::PointCloud<PointT>::Ptr &input_cloud)
{
  //assuming chull is already sequential, if not we'll need to sort it. It seems like it is.
  Box3D box;
  pcl::PointCloud<PointT>::Ptr cloud_hull (new pcl::PointCloud<PointT>);
  pcl::ConvexHull<PointT> chull;
  chull.setInputCloud (input_cloud);
  chull.reconstruct (*cloud_hull);
  int n = cloud_hull->size();
  PointT out[3];

  if( n > 2 )
  {
    rotatingCalipers( cloud_hull, n, CALIPERS_MINAREARECT, out );
    box.center.x = out[0].x + (out[1].x + out[2].x)*0.5f;
    box.center.y = out[0].y + (out[1].y + out[2].y)*0.5f;
    /*box.size.xSize = (float)sqrt((double)out[1].x*out[1].x + (double)out[1].y*out[1].y);
    box.size.ySize = (float)sqrt((double)out[2].x*out[2].x + (double)out[2].y*out[2].y);
    box.angle = (float)atan2( (double)out[1].y, (double)out[1].x );*/

    //xSize always the long!
    float norm1 = (float)sqrt((double)out[1].x*out[1].x + (double)out[1].y*out[1].y);
    float norm2 = (float)sqrt((double)out[2].x*out[2].x + (double)out[2].y*out[2].y);
    if(norm1 > norm2)
    {
      box.size.xSize = norm1;
      box.size.ySize = norm2;
      box.angle = (float)atan2( (double)out[1].y, (double)out[1].x );//(float)atan(out[1].y/out[1].x);//
    }
    else
    {
      box.size.xSize = norm2;
      box.size.ySize = norm1;
      box.angle = (float)atan2( (double)out[2].y, (double)out[2].x ); //(float)atan(out[2].y/out[2].x);//
    }

  } //haven't tested the below cases!
  else if( n == 2 )
  {
    box.center.x = ( cloud_hull->points[0].x +  cloud_hull->points[1].x)*0.5f;
    box.center.y = ( cloud_hull->points[0].y +  cloud_hull->points[1].y)*0.5f;
    double dx =  cloud_hull->points[1].x -  cloud_hull->points[0].x;
    double dy =  cloud_hull->points[1].y -  cloud_hull->points[0].y;
    box.size.xSize = (float)sqrt(dx*dx + dy*dy);
    box.size.ySize = 0;
    box.angle = (float)atan2( dy, dx );
  }
  else
  {
    if( n == 1 )
      box.center.x = cloud_hull->points[0].x;
      box.center.y = cloud_hull->points[0].y;

    box.angle = 0;
  }

  //box.fillQuatGivenAxisAngle();
  return box;
}
