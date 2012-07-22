// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

//
// This header declares the MapMaker class
// MapMaker makes and maintains the Map struct
// Starting with stereo initialisation from a bunch of matches
// over keyframe insertion, continual bundle adjustment and
// data-association refinement.
// MapMaker runs in its own thread, although some functions
// (notably stereo init) are called by the tracker and run in the
// tracker's thread.

#ifndef __MAPMAKER_H
#define __MAPMAKER_H

#include "Map.h"
#include "KeyFrame.h"
#include "ATANCamera.h"

#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/thread.h>

#include <queue>
#include <mutex>

namespace PTAMM {

class ActionDispatcher {
  public:
    typedef std::function<void()> Action;

    bool Empty() const;

    void RunQueuedActions();
    void PushAction(const Action &a);
    void PushActionAndWait(const Action &a);

  private:
    mutable std::mutex m;
    std::queue<Action> mvQueuedActions;
};

// MapMaker dervives from CVD::Thread, so everything in void run() is its own thread.
class MapMaker
{
  public:
    MapMaker(Map *m);
    ~MapMaker();

    void operator()();

    void StopThread() { mbDone = true; }

    // All the public methods here are threadsafe!

    // This doesn't neccessarily have to be done on this thread. Makes the interface different from the
    // ScaViSlam backend
    // Make a map from scratch. Called by the tracker.
    void InitFromStereo(KeyFrame &kFirst, KeyFrame &kSecond,
                        std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
                        SE3<> *se3CameraPos, bool async = true);

    void InitFromStereo(KeyFrame &kFirst, KeyFrame &kSecond,
                        const SE3<> &se3SecondCameraPos);

    bool StereoInitDone() const { return mbStereoInitDone; }

    void InitFromKnownPlane(KeyFrame &kKeyFrame, const TooN::Vector<4> &se3GroundPlane, SE3<> &se3TrackerPose);

    // Add a key-frame to the map. Called by the tracker.
    void AddKeyFrame(const KeyFrame &k);

    // These should all be made synchronous or return promises
    void Reset(bool async = false);              // Request that the we reset. Called by the tracker.

    // Should these be synchronous as well?
    void RealignGroundPlane(bool async = false);
    void TransformMapPoints(const SE3<>& se3NewFromOld, bool async = false);
    void ScaleMapPoints(double dScale, bool async = false);

  private:
    bool mbDone;

    // Used for thread synchronization
    ActionDispatcher mDispatcher;

    Map *mpMap;                       // The current map

    // Thread interaction signaling stuff
    bool mbAbortRequested;      // We should stop bundle adjustment and stereo init
    bool mbStereoInitDone;
};

}

#endif

