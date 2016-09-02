/*
 * misc_structures.hpp
 *
 *  Created on: Jul 31, 2013
 *      Author: siml
 */

#ifndef MISC_STRUCTURES_HPP_
#define MISC_STRUCTURES_HPP_

#include <math.h>

//why print2cout than friending <<? Dunno, there was a silly bug.
class sizeS{
public:
  float xSize;
  float ySize;
  float zSize;

  sizeS() : xSize(0), ySize(0), zSize(0) {};

  void print2cout()
  {
    std::cout << "Sizes:" << std::endl << " x: " << xSize << " y: " << ySize << " z: " << zSize << std::endl;;
  }

  float getVolume() {
    return xSize*ySize*zSize;}

  float getArea() {
    return 2*(xSize*ySize + xSize*ySize + ySize*zSize + zSize*xSize);}

  float getAspectRatio() {
    return (ySize/xSize);}
} ;

class centerS{
public:
  float x;
  float y;
  float z;

  centerS() : x(0), y(0), z(0) {};

  void print2cout()
  {
    std::cout << "Center Coordinates:" << std::endl << " x: " << x << " y: " << y << " z: " << z << std::endl;
  }
};

class Box3D
{
public:
  sizeS size;
  centerS center;
  float angle;
  float rot_quat[4];
  float rot_axis[3];
  bool is_quat_scalar_last; //if true [qx,qy,qz,qw], else [qw,qx,qy,qz]
  float aspect_ratio;

  float volume;
  float area;

  Box3D() : size(), center(), angle(0)
  {
    is_quat_scalar_last = true;

    //default z axis
    rot_axis[0] = 0;
    rot_axis[1] = 0;
    rot_axis[2] = 1;

    volume = 0;
    area = 0;
    aspect_ratio = 1;
  };

  void fillQuatGivenAxisAngle()
  {
	std::cout << "Will need to be rewamped. Split up angle into angle wrt normal plus angle for the global rotation" << std::endl;
	float sinHalfAngle = sin(angle/2);
    if (is_quat_scalar_last)
    {
        rot_quat[0] = rot_axis[0]*sinHalfAngle;
        rot_quat[1] = rot_axis[1]*sinHalfAngle;
        rot_quat[2] = rot_axis[2]*sinHalfAngle;
        rot_quat[3] = cos(angle/2);
    }
    else
    {
        rot_quat[1] = rot_axis[0]*sinHalfAngle;
        rot_quat[2] = rot_axis[1]*sinHalfAngle;
        rot_quat[3] = rot_axis[2]*sinHalfAngle;
        rot_quat[0] = cos(angle/2);
    }

    //std::cout << rot_quat[0] << " " <<  rot_quat[1] << " " <<  rot_quat[2] << " " <<  rot_quat[3] << std::endl;
    //std::cout << rot_axis[0] << " " <<  rot_axis[1] << " " <<  rot_axis[2] <<  std::endl << std::endl;
  }

  void normalizeAxis()
  {
    float norm = 0;
    for(int i=0; i < 3; i++)
      norm += rot_axis[i]*rot_axis[i];

    norm = sqrt(norm);

    for(int i=0; i < 3; i++)
      rot_axis[i] = rot_axis[i]/norm;
  }

  void calculateProperties()
  {
    volume = size.getVolume();
    area = size.getArea();
    aspect_ratio = size.getAspectRatio();
  }

  void print2cout()
  {
    std::cout << "Box:" << std::endl;
    center.print2cout();
    size.print2cout();
    std::cout << "Angle with Z: " << angle << std::endl;

  }

//  friend std::ostream& operator<< (std::ostream& output, const Box3D& a)
//  {
//    output << "Box:" << std::endl;
//    output << a.center << std::endl;
//    output << a.size << std::endl;
//    output << "Angle with Z: " << a.angle;
//  }
};

#endif /* MISC_STRUCTURES_HPP_ */
