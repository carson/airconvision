#include "StereoPlaneFinder.h"
#include "Utils.h"

#include <TooN/SymEigen.h>

#include <vector>
#include <cfloat>

using namespace std;
using namespace TooN;
using namespace CVD;

namespace PTAMM {

bool FindPlane(const std::vector<TooN::Vector<3> >& points, double inlierThreshold,
               Vector<3> &v3MeanOfInliers, Vector<3> &v3Normal)
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
  v3MeanOfInliers = Zeros;
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
  v3Normal = sym.get_evectors()[0];

  if(v3Normal[2] < 0) {
    v3Normal *= -1.0;
  }

  return true;
}



bool FindPlane(const SubImage<float> &imDisp, double inlierThreshold,
               Vector<3> &v3MeanOfInliers, Vector<3> &v3Normal)
{
  const float fMinDisparity = -1.0f;
  const ImageRef &irSize = imDisp.size();

  vector<Vector<3>> vv3Points;
  vv3Points.reserve(irSize.x * irSize.y);

  for (int y = 0; y < irSize.y; ++y) {
    const float *sptr = imDisp[y];
    for (int x = 0; x < irSize.x; ++x) {
      float d = sptr[x];
      if (abs(d - fMinDisparity) > numeric_limits<float>::epsilon()) {
        vv3Points.push_back(makeVector(x, y, d));
      }
    }
  }


  size_t nPoints = vv3Points.size();
  if(nPoints < 10) {
    std::cerr << "FindPlane needs more points to calculate plane" << std::endl;
    return false;
  }

  int nRansacs = 200;
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

    TooN::Vector<3> v3Mean = (1.0/3.0) *
        (vv3Points[nA] + vv3Points[nB] + vv3Points[nC]);

    TooN::Vector<3> v3CA = vv3Points[nC] - vv3Points[nA];
    TooN::Vector<3> v3BA = vv3Points[nB] - vv3Points[nA];
    TooN::Vector<3> v3Normal = v3CA ^ v3BA;
    if ((v3Normal * v3Normal) == 0) {
      continue;
    }
    normalize(v3Normal);

    double d = - v3Normal * v3Mean;

    double dSumError = 0.0;
    for (size_t i = 0; i < nPoints; ++i) {
      TooN::Vector<3> v3Diff = vv3Points[i] - v3Mean;
      double dDistSq = v3Diff * v3Diff;
      if (dDistSq == 0.0) {
        continue;
      }
//      double dNormDist = fabs(v3Diff * v3Normal);

      double dEstDisp = -(v3Normal.slice<0,2>() * vv3Points[i].slice<0,2>() + d)/v3Normal[2];
      double dNormDist = abs(1.0/dEstDisp - 1.0/vv3Points[i][2]);

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

  double d = - v3BestNormal * v3BestMean;


  // Done the ransacs, now collect the supposed inlier set
  vector<TooN::Vector<3> > vv3Inliers;
  for (size_t i = 0; i < nPoints; ++i) {
    TooN::Vector<3> v3Diff = vv3Points[i] - v3BestMean;
    double dDistSq = v3Diff * v3Diff;
    if (dDistSq == 0.0)
      continue;
//    double dNormDist = abs(v3Diff * v3BestNormal);

    double dEstDisp = -(v3BestNormal.slice<0,2>() * vv3Points[i].slice<0,2>() + d)/v3BestNormal[2];
    double dNormDist = abs(1.0/dEstDisp - 1.0/vv3Points[i][2]);

    if (dNormDist < inlierThreshold)
      vv3Inliers.push_back(vv3Points[i]);
  }

  // With these inliers, calculate mean and cov
  v3MeanOfInliers = Zeros;
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
  v3Normal = sym.get_evectors()[0];

  v3Normal = sym.get_evectors()[2] ^ sym.get_evectors()[1];

  cout << "Inliers: " << vv3Inliers.size() << " / " << vv3Points.size() << endl;

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
  Vector<3> v3Mean, v3Normal;
  if (FindPlane(pointCloud, 20.0, v3Mean, v3Normal)) {
    normalize(v3Normal);
    Vector<4> v4Plane;
    v4Plane.slice<0,3>() = v3Normal;
    v4Plane[3] = -v3Normal * v3Mean;
    // Send it through the Kalman filter
    //UpdateFilter(v4Plane);
    mv4Plane = v4Plane;
  }
}

void StereoPlaneFinder::Update(const CVD::SubImage<float> &imDisp, double f, double L)
{
  Vector<3> v3Mean, v3Normal;
  double dThreshold = abs(10.0/(L*f)); // 10 cm in Euclidean space
  if (FindPlane(imDisp, dThreshold, v3Mean, v3Normal)) {
    normalize(v3Normal);
    Vector<4> v4PlaneDisp;
    v4PlaneDisp.slice<0,3>() = v3Normal;
    v4PlaneDisp[3] = -v3Normal * v3Mean;

    // Convert the plane from disparity image space to 3-space
    double dCoefInv = -1.0 / v4PlaneDisp[2];
    Vector<4> v4Plane3 = makeVector(v4PlaneDisp[0] * dCoefInv,
                                    v4PlaneDisp[1] * dCoefInv,
                                    v4PlaneDisp[3] * dCoefInv/f,
                                    L);

    // Normalize so the plane normal becomes a unit vector. Should not be needed.
    Vector<3> v3N = v4Plane3.slice<0,3>();
    double d = sqrt(v3N * v3N);
    v4Plane3 *= -(1.0/d);

    cout << v4Plane3 << endl;
    // Send it through the Kalman filter
    //UpdateFilter(v4Plane);
    mv4Plane = v4Plane3;
  }
}

void StereoPlaneFinder::UpdateFilter(const TooN::Vector<4>& v4Plane)
{
  if (mbFirstUpdate) {
    mFilter.init(v4Plane, Identity);
    mbFirstUpdate = false;
  } else {
    mFilter.predict(Zeros);
    mFilter.observe(v4Plane);
  }
}

const TooN::Vector<4>& StereoPlaneFinder::GetPlane() const
{
  return mv4Plane;
  return mFilter.mu();
}

}
