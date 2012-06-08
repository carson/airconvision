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
    MapMaker(std::vector<Map*> &maps, Map *m);
    ~MapMaker();

    void operator()();

    // Make a map from scratch. Called by the tracker.
    void InitFromStereo(KeyFrame &kFirst, KeyFrame &kSecond,
                        std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
                        SE3<> &se3CameraPos);
    bool StereoInitDone() const { return mbStereoInitDone; }

    // Add a key-frame to the map. Called by the tracker.
    void AddKeyFrame(const KeyFrame &k);


    void RequestRealignment();

    // These should all be made synchronous or return promises
    void Reset();              // Request that the we reset. Called by the tracker.
    void ReInit(Map * map);    // Request that the we reset. Called by the tracker.
    bool Switch(Map * map);    // Request a switch to map

    void RequestMapTransformation(const SE3<>& se3NewFromOld);
    void RequestMapScaling(double dScale);

  private:
    // General Maintenance/Utility:
    void ResetImpl() { ResetImpl2(mpMap); }
    void ResetImpl2(Map * map);
    void ReInitImpl();                //call this when switching to a new map
    void SwitchMapImpl();

  // Member variables:
  private:
    std::vector<Map*> &mvpMaps;       // The vector of maps
    Map *mpMap;                       // The current map
    Map *mpNewMap;                     // The new map, used as a temp placeholder
    Map *mpSwitchMap;                  // The switch map, used as a temp placeholder

    // Used for thread synchronization
    ActionDispatcher mDispatcher;

    // Thread interaction signaling stuff
    bool mbAbortRequested;      // We should stop bundle adjustment and stereo init
    bool mbStereoInitDone;
};

}

#endif

