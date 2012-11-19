#include "MathUtils.h"

#include <TooN/SVD.h>

namespace PTAMM {

// Finds 3d coords of point in reference frame B from two z=1 plane projections
Vector<3> ReprojectPoint(const SE3<>& se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B)
{
  Matrix<3,4> PDash;
  PDash.slice<0,0,3,3>() = se3AfromB.get_rotation().get_matrix();
  PDash.slice<0,3,3,1>() = se3AfromB.get_translation().as_col();

  Matrix<4> A;
  A[0][0] = -1.0; A[0][1] =  0.0; A[0][2] = v2B[0]; A[0][3] = 0.0;
  A[1][0] =  0.0; A[1][1] = -1.0; A[1][2] = v2B[1]; A[1][3] = 0.0;
  A[2] = v2A[0] * PDash[2] - PDash[0];
  A[3] = v2A[1] * PDash[2] - PDash[1];

  SVD<4,4> svd(A);
  Vector<4> v4Smallest = svd.get_VT()[3];
  if(v4Smallest[3] == 0.0)
    v4Smallest[3] = 0.00001;
  return project(v4Smallest);
}


Vector<3> So3ToYawPitchRoll(const SO3<> &so3Orientation)
{
  const Matrix<3> m33Rot = so3Orientation.get_matrix().T();
  double dYaw = atan2(m33Rot[0][1], m33Rot[0][0]);
  double d1 = norm(makeVector(m33Rot[0][0], m33Rot[0][1]));
  double dPitch = atan2(-m33Rot[0][2], d1);
  double dRoll = atan2(m33Rot[2][1], m33Rot[2][2]);
  return makeVector(dYaw, dPitch, dRoll);
}

}
