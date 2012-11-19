#ifndef __MATH_UTILS_H
#define __MATH_UTILS_H

#include <TooN/TooN.h>
#include <TooN/se3.h>

namespace PTAMM {

using namespace TooN;

// Finds 3d coords of point in reference frame B from two z=1 plane projections
Vector<3> ReprojectPoint(const SE3<>& se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B);

Vector<3> So3ToYawPitchRoll(const SO3<> &so3Orientation);

}

#endif
