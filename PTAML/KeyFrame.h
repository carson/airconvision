// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the data structures to do with keyframes:
// structs KeyFrame, Level, Measurement, Candidate.
//
// A KeyFrame contains an image pyramid stored as array of Level;
// A KeyFrame also has associated map-point mesurements stored as a vector of Measurment;
// Each individual Level contains an image, corner points, and special corner points
// which are promoted to Candidate status (the mapmaker tries to make new map points from those.)
//
// KeyFrames are stored in the Map class and manipulated by the MapMaker.
// However, the tracker also stores its current frame as a half-populated
// KeyFrame struct.


#ifndef __KEYFRAME_H
#define __KEYFRAME_H

#include "ATANCamera.h"

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>

#include <vector>
#include <set>
#include <map>

namespace PTAMM {

using namespace TooN;

struct MapPoint;
class SmallBlurryImage;

const int LEVELS = 4;

extern int g_nNumFeaturesFound[LEVELS];

// Candidate: a feature in an image which could be made into a map point
struct Candidate
{
  Candidate() {}
  Candidate(const CVD::ImageRef &irLevelPos, double dSTScore)
    : irLevelPos(irLevelPos), dSTScore(dSTScore) {}
  CVD::ImageRef irLevelPos;
  double dSTScore;
};

// Measurement: A 2D image measurement of a map point. Each keyframe stores a bunch of these.
struct Measurement
{
  int nLevel;   // Which image level?
  bool bSubPix; // Has this measurement been refined to sub-pixel level?
  Vector<2> v2RootPos;  // Position of the measurement, REFERED TO PYRAMID LEVEL ZERO
  Vector<2> v2ImplanePos;  // Position of the measurement, in Z=1 coords
  Matrix<2> m2CamDerivs; // Cam derivs at found location
  enum {SRC_TRACKER, SRC_REFIND, SRC_ROOT, SRC_TRAIL, SRC_EPIPOLAR} Source; // Where has this measurement come frome?
};

// Each keyframe is made of LEVELS pyramid levels, stored in struct Level.
// This contains image data and corner points.
class Level
{
  friend class KeyFrame;

  public:
    Level() : bImplaneCornersCached(false)
    {
    }

    Level& operator=(const Level &rhs);

    void FindCorners(int barrier);
    void FindMaxCornersAndCandidates(double dCandidateMinSTScore);

    const std::vector<CVD::ImageRef>& GetCorners() const { return vCorners; }
    const std::vector<int>& GetCornerRowLUT() const { return vCornerRowLUT; }
    const std::vector<CVD::ImageRef>& GetMaxCorners() const { return vMaxCorners; }
    const std::vector<Candidate>& GetCandidates() const { return vCandidates; }

  public:
    CVD::Image<CVD::byte> im;                // The pyramid level pixels

    bool bImplaneCornersCached;           // Also keep image-plane (z=1) positions of FAST corners to speed up epipolar search
    std::vector<Vector<2> > vImplaneCorners; // Corner points un-projected into z=1-plane coordinates

  private:
    void FindCornersInCell(int barrier, const CVD::ImageRef &start, const CVD::ImageRef &size);
    void FindCandidatesInCell(const CVD::ImageRef &start, const CVD::ImageRef &size);

  private:
    std::vector<CVD::ImageRef> vCorners;     // All FAST corners on this level
    std::vector<int> vCornerRowLUT;          // Row-index into the FAST corners, speeds up access
    std::vector<CVD::ImageRef> vMaxCorners;  // The maximal FAST corners
    std::vector<Candidate> vCandidates;      // Potential locations of new map points

    int mBarrier;
};

// The actual KeyFrame struct. The map contains of a bunch of these. However, the tracker uses this
// struct as well: every incoming frame is turned into a keyframe before tracking; most of these
// are then simply discarded, but sometimes they're then just added to the map.
class KeyFrame
{
  public:
    KeyFrame(const ATANCamera &cam);
    KeyFrame(const KeyFrame &k);
    ~KeyFrame();

    KeyFrame& operator=(const KeyFrame &rhs);

    void ThinCandidates(int nLevel);
    void RefreshSceneDepth();

    // This takes an image and calculates pyramid levels etc to fill the
    // keyframe data structures with everything that's needed by the tracker..
    void MakeKeyFrame_Lite(const CVD::BasicImage<CVD::byte> &im, int* aFastCornerBarriers);
    // ... while this calculates the rest of the data which the mapmaker needs.
    void MakeKeyFrame_Rest();

  public:
    SE3<> se3CfromW;    // The coordinate frame of this key-frame as a Camera-From-World transformation
    bool bFixed;      // Is the coordinate frame of this keyframe fixed? (only true for first KF!)
    Level aLevels[LEVELS];  // Images, corners, etc lives in this array of pyramid levels
    std::map<MapPoint*, Measurement> mMeasurements;           // All the measurements associated with the keyframe

    double dSceneDepthMean;      // Hacky hueristics to improve epipolar search.
    double dSceneDepthSigma;

    SmallBlurryImage *pSBI; // The relocaliser uses this

    ATANCamera Camera;                                        // The camera model which this KF came from
};

}

#endif

