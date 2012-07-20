// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited

/********************************************************************

  A set of utility functions
  Author: Robert Castle, 2009, bob@robots.ox.ac.uk

********************************************************************/

#ifndef __PTAMM_UTILS__
#define __PTAMM_UTILS__

#include "ATANCamera.h"

#include <cvd/image_ref.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>

#include <string>
#include <iostream>
#include <iterator>
#include <vector>
#include <algorithm>
#include <numeric>

namespace PTAMM {

/**
 * Output a vector as a stream that is space separated.
 * @param os Output stream
 * @param v Vector to output
 * @return the stream output
 */
template<class T>
  std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
  std::copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " "));
  return os;
}

/**
 * Returns the indices of the sorted vector.
 *
 * Taken from http://stackoverflow.com/questions/10580982/c-sort-keeping-track-of-indices
 */
template <typename T>
std::vector<size_t> ordered(std::vector<T> const& values) {
    std::vector<size_t> indices(values.size());
    std::iota(begin(indices), end(indices), static_cast<size_t>(0));

    std::sort(
        begin(indices), end(indices),
        [&](size_t a, size_t b) { return values[a] > values[b]; }
    );
    return indices;
}


void PruneWhiteSpace(std::string & str);

bool PointInsideRect(const CVD::ImageRef &pt, const CVD::ImageRef &start, const CVD::ImageRef &size);

SE3<> AlignerFromPointAndUp(const TooN::Vector<3>& point,
                            const TooN::Vector<3>& normal);

TooN::Vector<4> Se3ToPlane(const SE3<>& se3);
TooN::SE3<> PlaneToSe3(const TooN::Vector<4>& plane);

bool PickPointOnPlane(ATANCamera camera,
                      const TooN::Vector<4> &v4Plane,
                      const TooN::Vector<2> &v2PixelCoord,
                      TooN::Vector<3> &v3PointOnPlane);

bool PickPointOnGround(ATANCamera camera,
                       const TooN::SE3<> &se3CamFromWorld,
                       const TooN::Vector<2> &pixelCoord,
                       TooN::Vector<3> &pointOnPlane);


}

#endif
