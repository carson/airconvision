// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
#ifndef __BUNDLE_H
#define __BUNDLE_H
// Bundle.h
// 
// This file declares the Bundle class along with a few helper classes.
// Bundle is the bundle adjustment core of the mapping system; instances
// of Bundle are generated by MapMaker to adjust the positions of 
// keyframes (called Cameras in this file) and map points.
//
// It's a pretty straight-forward Levenberg-Marquardt bundle adjustment 
// implementation closely following Hartley and Zisserman's MVG book, with
// the addition of a robust M-Estimator.
//
// Unfortunately, having undergone a few tweaks, the code is now
// not the easiest to read!
//
// Basic operation: MapMaker creates a new Bundle object;
// then adds map points and keyframes to adjust;
// then adds measurements of map points in keyframes;
// then calls Compute() to do bundle adjustment;
// then reads results back to update the map.

#include "ATANCamera.h"
#include <TooN/TooN.h>

#include <TooN/se3.h>
#include <vector>
#include <map>
#include <set>
#include <list>

namespace PTAMM {

using namespace TooN;
             
// An index into the big measurement map which stores all the measurements.

// Camera struct holds the pose of a keyframe
// and some computation intermediates
struct Camera
{
  bool bFixed;
  SE3<> se3CfW;
  SE3<> se3CfWNew;
  Matrix<6> m6U;          // Accumulator
  Vector<6> v6EpsilonA;   // Accumulator
  int nStartRow;
};

// Camera-camera pair index
struct OffDiagScriptEntry
{
  int j;
  int k;
};

// A map point, plus computation intermediates.
struct Point
{
  inline Point()
  { nMeasurements = 0; nOutliers = 0;}
  Vector<3> v3Pos;
  Vector<3> v3PosNew;
  Matrix<3> m3V;          // Accumulator
  Vector<3> v3EpsilonB;   // Accumulator 
  Matrix<3> m3VStarInv;
  
  int nMeasurements;
  int nOutliers;
  std::set<int> sCameras; // Which cameras observe this point?
  std::vector<OffDiagScriptEntry> vOffDiagonalScript; // A record of all camera-camera pairs observing this point
};

// A measurement of a point by a camera, plus
// computation intermediates.
struct Meas
{
  inline Meas()
  {bBad = false;}
  
  // Which camera/point did this measurement come from?
  int p; // The point  - called i in MVG
  int c; // The camera - called j in MVG

  inline bool operator<(const Meas &rhs) const
  {  return(c<rhs.c ||(c==rhs.c && p < rhs.p)); }
  
  bool bBad;
  
  Vector<2> v2Found;
  Vector<2> v2Epsilon;
  Matrix<2,6> m26A;
  Matrix<2,3> m23B;
  Matrix<6,3> m63W; 
  Matrix<6,3> m63Y;
  double dSqrtInvNoise;
  
  // Temporary projection quantities
  Vector<3> v3Cam;
  double dErrorSquared;
  Matrix<2> m2CamDerivs;
};

// Core bundle adjustment class
class Bundle {
  public:
    Bundle();

    // Add a viewpoint. bFixed signifies that this one is not to be adjusted.
    int AddCamera(const SE3<>& se3CamFromWorld, bool bFixed);
    // Add a map point.
    int AddPoint(const Vector<3>& v3Pos);
    // Add a measurement of a map point in a camera
    void AddMeas(int nCam, int nPoint, const Vector<2>& v2Pos, double dSigmaSquared, const Matrix<2>& m2CamDerivs);

    // Perform bundle adjustment. Aborts if *pbAbortSignal gets set to true. Returns number of accepted update iterations, or negative on error.
    int Compute(bool *pbAbortSignal);
    // Has bundle adjustment converged?
    bool Converged() const { return mbConverged;}

    // Point coords after adjustment
    const Vector<3>& GetPoint(int n) const;
    // Camera pose after adjustment
    const SE3<>& GetCamera(int n) const;
    // Measurements flagged as outliers
    const std::vector<std::pair<int,int> >& GetOutlierMeasurements() const;
    // Points flagged as outliers
    std::set<int> GetOutliers() const;

  private:
    inline void ProjectAndFindSquaredError(Meas &meas); // Project a single point in a single view, compare to measurement
    template<class MEstimator> bool Do_LM_Step(bool *pbAbortSignal);
    template<class MEstimator> double FindNewError();
    void GenerateMeasLUTs();
    void GenerateOffDiagScripts();
    void ClearAccumulators(); // Zero temporary quantities stored in cameras and points
    void ModifyLambda_GoodStep();
    void ModifyLambda_BadStep();

  private:
    std::vector<Point> mvPoints;
    std::vector<Camera> mvCameras;
    std::list<Meas> mMeasList;
    std::vector<std::pair<int,int> > mvOutlierMeasurementIdx;  // p-c pair
    std::vector<std::vector<Meas*> > mvMeasLUTs;  //Each camera gets a per-point table of pointers to valid measurements

    int mnCamsToUpdate;
    int mnStartRow;
    double mdSigmaSquared;
    double mdLambda;
    double mdLambdaFactor;
    bool mbConverged;
    bool mbHitMaxIterations;
    int mnCounter;
    int mnAccepted;

    GVars3::gvar3<int> mgvnMaxIterations;
    GVars3::gvar3<double> mgvdUpdateConvergenceLimit;
    GVars3::gvar3<int> mgvnBundleCout;

    bool *mpbAbortSignal;
};

}

#endif
