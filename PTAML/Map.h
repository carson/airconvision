// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited
//
// This header declares the Map class.
// This is pretty light-weight: All it contains is
// a vector of MapPoints and a vector of KeyFrames.
//
// N.b. since I don't do proper thread safety,
// everything is stored as lists of pointers,
// and map points are not erased if they are bad:
// they are moved to the trash list. That way
// old pointers which other threads are using are not
// invalidated!

#ifndef __MAP_H
#define __MAP_H

#include <vector>
#include <TooN/se3.h>
#include <cvd/image.h>
#include <deque>

#include "MapLockManager.h"
#include "OpenGL.h"
#include "TakFrame.h"
#include "Bundle.h"

namespace PTAMM {

struct MapPoint;
class KeyFrame;
class Map;
class MapSerializer;

class BundleAdjustmentJob {
  public:
    BundleAdjustmentJob(Map *pMap,
                        const std::set<KeyFrame*>& sAdjustSet,
                        const std::set<KeyFrame*>& sFixedSet,
                        const std::set<MapPoint*>& sMapPoints,
                        bool bRecent);

    bool Run(bool *pbAbortSignal);

  private:
    Map *mpMap;
    Bundle mBundle;
    bool mbRecent;

    // The bundle adjuster does different accounting of keyframes and map points;
    // Translation maps are stored:
    std::map<int, MapPoint*> mBundleID_Point;
    std::map<int, KeyFrame*> mBundleID_View;
    std::map<MapPoint*, int> mPoint_BundleID;
    std::map<KeyFrame*, int> mView_BundleID;
};

class Map
{
  friend class BundleAdjustmentJob;
  friend class MapSerializer;

  public:
    Map();
    ~Map();

    int MapID() const { return mnMapNum; }

    bool IsGood() const { return bGood; }
    void Reset();

    // Make a map from scratch. Called by the tracker.
    bool InitFromStereo(KeyFrame &kFirst, KeyFrame &kSecond,
                        std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
                        TooN::SE3<> &se3CameraPos);

    // Keyframe queue
    size_t QueueSize() const { return vpKeyFrameQueue.size(); } // How many KFs in the queue waiting to be added?
    void QueueKeyFrame(const KeyFrame &k);
    void AddKeyFrameFromTopOfQueue();

    // Keyframe distance functions
    double DistToNearestKeyFrame(const KeyFrame &kCurrent);
    double KeyFrameLinearDist(const KeyFrame &k1, const KeyFrame &k2) const;
    KeyFrame* ClosestKeyFrame(const KeyFrame &k);

    // Bundle adjustments
    bool FullBundleAdjust(bool *pbAbortSignal = NULL);
    bool RecentBundleAdjust(bool *pbAbortSignal = NULL);
    bool FullBundleAdjustConverged() const { return bBundleConverged_Full; }
    bool RecentBundleAdjustConverged() const { return bBundleConverged_Recent; }

    void ReFindFromFailureQueue();
    void ReFindNewlyMade();

    // Functions for starting the map from scratch:
    TooN::SE3<> CalcPlaneAligner() const;

    // World transformation
    void ApplyGlobalTransformation(const TooN::SE3<>& se3NewFromOld);
    void ApplyGlobalScale(double dScale);

    // Uses a heuristic to mark certain points as bad and move the to the thrash list
    void HandleBadPoints();
    // Frees the memory of the points in the thrash list
    void EmptyTrash();

    // Leaking constness but better than having it completly public as before
    const std::vector<MapPoint*>& GetMapPoints() const { return vpPoints; }
    const std::vector<KeyFrame*>& GetKeyFrames() const { return vpKeyFrames; }

    // camparijets hack for keyframe visualization
    void InitTexture();

  public:
    // What is the point of this?? -- dhenell
    bool bEditLocked;                                // Is the map locked from being edited?
    MapLockManager mapLockManager;                   // All threads must register to this and
                                                     // use when need complete control of a map

  private:
    // Returns point in ref frame B
    TooN::Vector<3> ReprojectPoint(const TooN::SE3<>& se3AfromB,
                                   const TooN::Vector<2> &v2A,
                                   const TooN::Vector<2> &v2B) const;
    // For map generation
    void AddSomeMapPoints(int nLevel);
    bool AddPointEpipolar(KeyFrame &kSrc, KeyFrame &kTarget, int nLevel, int nCandidate);

    std::vector<KeyFrame*> NClosestKeyFrames(const KeyFrame &k, unsigned int N) const;

    // Data association functions:
    int ReFindInSingleKeyFrame(KeyFrame &k);
    void ReFindAll();
    bool ReFind_Common(KeyFrame &k, MapPoint &p);
    void SubPixelRefineMatches(KeyFrame &k, int nLevel);

    // Point memory handling
    void MoveBadPointsToTrash();

    // camparijets hack for keyframe visualization
    void MakeTextureFromKF(KeyFrame &k);
    void ReleaseTexture();

  private:
    std::vector<MapPoint*> vpPoints;
    std::vector<KeyFrame*> vpKeyFrames;
    std::vector<MapPoint*> vpPointsTrash;

    std::vector<KeyFrame*> vpKeyFrameQueue;  // Queue of keyframes from the tracker waiting to be processed
    std::vector<std::pair<KeyFrame*, MapPoint*> > vFailureQueue; // Queue of failed observations to re-find
    std::deque<MapPoint*> qNewQueue;   // Queue of newly-made map points to re-find in other KeyFrames

    bool bBundleConverged_Full;                      // Has global bundle adjustment converged?
    bool bBundleConverged_Recent;                    // Has local bundle adjustment converged?

    bool bGood;                                      // Is the map valid (has content)?

    std::string sSaveFile;                           // where the map was loaded from
    GLuint texName[10000];//@hack by camparijet for texture
    int N;//@hack for texture number

    std::vector<TakFrame> poolAllFrame; //@hack for serialize

    int mnMapNum;                                    // The map number
    int nTex;//@hack for index of texture

    double mdWiggleScale; // Distance between keyframes
    double mdWiggleScaleDepthNormalized;
};

}

#endif

