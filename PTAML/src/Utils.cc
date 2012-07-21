// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited

/********************************************************************

  A set of utility functions
  Author: Robert Castle, 2009, bob@robots.ox.ac.uk

********************************************************************/

#include "Utils.h"
#include <gvars3/GStringUtil.h>

#include <iostream>
#include <sstream>

using namespace std;
using namespace TooN;

namespace PTAMM {


/**
 * Edit a string so that the only whitespace is a single space between elements.
 * @param str the string to edit. This gets replaced.
 */
void PruneWhiteSpace(std::string & str)
{
  std::vector< std::string > tokens = GVars3::ChopAndUnquoteString(str);
  std::ostringstream os;
  os << tokens;

  str = os.str();

  if(str.at( str.size() - 1 ) == ' ' ) {
    str.erase(  str.end() - 1 );
  }
}

bool PointInsideRect(const CVD::ImageRef &pt, const CVD::ImageRef &start, const CVD::ImageRef &size)
{
  return pt.x >= start.x && pt.y >= start.y && pt.x < start.x + size.x && pt.y < start.y + size.y;
}

SE3<> AlignerFromPointAndUp(const TooN::Vector<3>& point,
                            const TooN::Vector<3>& normal)
{
  Matrix<3> m3Rot = Identity;
  m3Rot[2] = normal;
  m3Rot[0] = m3Rot[0] - (normal * (m3Rot[0] * normal));
  normalize(m3Rot[0]);
  m3Rot[1] = m3Rot[2] ^ m3Rot[0];

  SE3<> se3;
  se3.get_rotation() = m3Rot;
  TooN::Vector<3> v3RMean = se3 * point;
  se3.get_translation() = -v3RMean;

  return se3;
}

TooN::Vector<4> Se3ToPlane(const SE3<>& se3)
{
  TooN::Vector<3> normal = se3.get_rotation().get_matrix()[2];
  double d = -normal * se3.inverse().get_translation();
  return makeVector(normal[0], normal[1], normal[2], d);
}

TooN::SE3<> PlaneToSe3(const TooN::Vector<4>& plane)
{
  TooN::Vector<3> normal = plane.slice<0,3>();
  normalize(normal);
  TooN::Vector<3> point = -plane[3] * normal;
  return AlignerFromPointAndUp(point, normal);
}

bool PickPointOnPlane(ATANCamera camera,
                      const TooN::Vector<4> &v4Plane,
                      const TooN::Vector<2> &v2PixelCoord,
                      TooN::Vector<3> &v3PointOnPlane)
{
  // Work out plane coords:
  Vector<2> v2ImPlane = camera.UnProject(v2PixelCoord);
  Vector<3> v3C = unproject(v2ImPlane);
  Vector<3> v3Normal = v4Plane.slice<0, 3>();
  double dD = v4Plane[3];
  double denom = v3Normal * v3C;

  // Clicked the wrong side of the horizon?
  if (abs(denom) > 0.0001) {
    double k = dD/denom;
    v3PointOnPlane = -k * v3C;
    return true;
  }

  return false;
}

bool PickPointOnGround(ATANCamera camera,
                       const TooN::SE3<> &se3CamFromWorld,
                       const TooN::Vector<2> &pixelCoord,
                       TooN::Vector<3> &pointOnPlane)
{
  // Work out plane coords:
  Vector<2> v2ImPlane = camera.UnProject(pixelCoord);
  Vector<3> v3C = unproject(v2ImPlane);
  Vector<4> v4C = unproject(v3C);
  SE3<> se3CamInv = se3CamFromWorld.inverse();
  Vector<4> v4W = se3CamInv * v4C;
  double t = se3CamInv.get_translation()[2];
  double dDistToPlane = -t / (v4W[2] - t);

  if(v4W[2] -t <= 0) // Clicked the wrong side of the horizon?
  {
    v4C.slice<0,3>() *= dDistToPlane;
    Vector<4> v4Result = se3CamInv * v4C;
    pointOnPlane = v4Result.slice<0,3>(); // <--- result

    return true;
  }

  return false;
}

}
