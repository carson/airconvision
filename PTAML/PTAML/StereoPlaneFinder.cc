#include "StereoPlaneFinder.h"
#include "Utils.h"

#include <TooN/SymEigen.h>

#include <vector>

using namespace std;
using namespace TooN;

namespace PTAMM {

bool FindPlaneAligner(const std::vector<TooN::Vector<3> >& points,
                      bool bFlipNormal, double inlierThreshold,
                      SE3<>& planeAligner)
{
  size_t nPoints = points.size();
  if(nPoints < 10) {
    std::cerr << "FindPlaneAligner needs more point to calculate plane" << std::endl;
    return false;
  }

  int nRansacs = 500;
  TooN::Vector<3> v3BestMean;
  TooN::Vector<3> v3BestNormal;
  double dBestDistSquared = std::numeric_limits<double>::max();

  for (int i = 0; i < nRansacs; ++i) {
    int nA = rand()%nPoints;
    int nB = nA;
    int nC = nA;
    while(nB == nA)
      nB = rand()%nPoints;
    while(nC == nA || nC==nB)
      nC = rand()%nPoints;

    TooN::Vector<3> v3Mean = (1.0/3.0) * (points[nA] +
                                          points[nB] +
                                          points[nC]);

    TooN::Vector<3> v3CA = points[nC]  - points[nA];
    TooN::Vector<3> v3BA = points[nB]  - points[nA];
    TooN::Vector<3> v3Normal = v3CA ^ v3BA;
    if ((v3Normal * v3Normal) == 0) {
      continue;
    }
    normalize(v3Normal);

    double dSumError = 0.0;
    for (size_t i = 0; i < nPoints; ++i) {
      TooN::Vector<3> v3Diff = points[i] - v3Mean;
      double dDistSq = v3Diff * v3Diff;
      if (dDistSq == 0.0) {
        continue;
      }
      double dNormDist = fabs(v3Diff * v3Normal);
      if(dNormDist > inlierThreshold)
        dNormDist = inlierThreshold;
      dSumError += dNormDist;
    }

    if (dSumError < dBestDistSquared) {
      dBestDistSquared = dSumError;
      v3BestMean = v3Mean;
      v3BestNormal = v3Normal;
    }
  }

  // Done the ransacs, now collect the supposed inlier set
  vector<TooN::Vector<3> > vv3Inliers;
  for (size_t i = 0; i < nPoints; ++i) {
    TooN::Vector<3> v3Diff = points[i] - v3BestMean;
    double dDistSq = v3Diff * v3Diff;
    if (dDistSq == 0.0)
      continue;
    double dNormDist = fabs(v3Diff * v3BestNormal);
    if (dNormDist < inlierThreshold)
      vv3Inliers.push_back(points[i]);
  }

  // With these inliers, calculate mean and cov
  TooN::Vector<3> v3MeanOfInliers = Zeros;
  for (size_t i = 0; i < vv3Inliers.size(); ++i) {
    v3MeanOfInliers += vv3Inliers[i];
  }

  v3MeanOfInliers *= (1.0 / vv3Inliers.size());

  Matrix<3> m3Cov = Zeros;
  for (size_t i = 0; i < vv3Inliers.size(); ++i) {
    TooN::Vector<3> v3Diff = vv3Inliers[i] - v3MeanOfInliers;
    m3Cov += v3Diff.as_col() * v3Diff.as_row();
  }

  // Find the principal component with the minimal variance: this is the plane normal
  SymEigen<3> sym(m3Cov);
  TooN::Vector<3> v3Normal = sym.get_evectors()[0];

  if (bFlipNormal) {
    // Use the version of the normal which points towards the cam center
    if(v3Normal[2] > 0) {
      v3Normal *= -1.0;
    }
  } else {
    // Use the positive Z
    if(v3Normal[2] < 0) {
      v3Normal *= -1.0;
    }
  }

  planeAligner = AlignerFromPointAndUp(v3MeanOfInliers, v3Normal);
  return true;
}

StereoPlaneFinder::StereoPlaneFinder()
  : mbFirstUpdate(true)
{
  Matrix<4> R = Zeros;
  R.diagonal_slice() = makeVector(0.4, 0.4, 0.4, 8.0); // Prediction noise
  Matrix<4> Q = Zeros;
  Q.diagonal_slice() = makeVector(0.2, 0.2, 0.2, 4.0); // Observation noise
  mFilter = KalmanFilter<4, 4, 1>(Identity, Zeros, R, Identity, Q);
}

void StereoPlaneFinder::Update(const std::vector<TooN::Vector<3> >& pointCloud)
{
  SE3<> plane;
  if (FindPlaneAligner(pointCloud, false, 1.0, plane)) {
    // Send it through the Kalman filter
    UpdateFilter(plane);
  }
}

void StereoPlaneFinder::UpdateFilter(const SE3<>& planeAligner)
{
  if (mbFirstUpdate) {
    mFilter.init(Se3ToPlane(planeAligner), Identity);
    mbFirstUpdate = false;
  } else {
    mFilter.predict(Zeros);
    mFilter.observe(Se3ToPlane(planeAligner));
  }
}

SE3<> StereoPlaneFinder::GetPlane() const {
  return PlaneToSe3(mFilter.mu());
}

}
