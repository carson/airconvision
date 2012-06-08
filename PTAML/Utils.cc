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


bool PickPointOnGround(ATANCamera camera,
                       const TooN::SE3<> &se3CamFromWorld,
                       const TooN::Vector<2> &pixelCoord,
                       TooN::Vector<3> &pointOnPlane)
{
  Vector<2> v2VidCoords = pixelCoord;

  Vector<2> v2UFBCoords;
#ifdef WIN32
  Vector<2> v2PlaneCoords;   v2PlaneCoords[0] = numeric_limits<double>::quiet_NaN();   v2PlaneCoords[1] = numeric_limits<double>::quiet_NaN();
#else
  Vector<2> v2PlaneCoords;   v2PlaneCoords[0] = NAN;   v2PlaneCoords[1] = NAN;
#endif
  Vector<3> v3RayDirn_W;

  // Work out image coords 0..1:
  v2UFBCoords[0] = (v2VidCoords[0] + 0.5) / camera.GetImageSize()[0];
  v2UFBCoords[1] = (v2VidCoords[1] + 0.5) / camera.GetImageSize()[1];

  // Work out plane coords:
  Vector<2> v2ImPlane = camera.UnProject(v2VidCoords);
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
