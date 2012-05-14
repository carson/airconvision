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

#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/thread.h>

#include "Map.h"
#include "KeyFrame.h"
#include "ATANCamera.h"

namespace PTAMM {

// MapMaker dervives from CVD::Thread, so everything in void run() is its own thread.
class MapMaker : protected CVD::Thread
{
  public:
    MapMaker(std::vector<Map*> &maps, Map *m);
    ~MapMaker();

    // Make a map from scratch. Called by the tracker.
    bool InitFromStereo(KeyFrame &kFirst, KeyFrame &kSecond,
                        std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
                        SE3<> &se3CameraPos);

    void AddKeyFrame(const KeyFrame &k);   // Add a key-frame to the map. Called by the tracker.
    void RequestReset();   // Request that the we reset. Called by the tracker.
    bool ResetDone();      // Returns true if the has been done.

    bool NeedNewKeyFrame(const KeyFrame &kCurrent);            // Is it a good camera pose to add another KeyFrame?
    bool IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent);  // Is the camera far away from the nearest KeyFrame (i.e. maybe lost?)

    //New PTAMM functions
    void RequestReInit(Map * map);    // Request that the we reset. Called by the tracker.
    bool ReInitDone();                // Returns true if the ReInit has been done.
    bool RequestSwitch(Map * map);    // Request a switch to map
    bool SwitchDone();                // Returns true if the Switch map has been done.

    void RequestMapTransformation(const SE3<>& se3NewFromOld);
    void RequestMapScaling(double dScale);

  private:
    virtual void run();      // The MapMaker thread code lives here

    // General Maintenance/Utility:
    //PTAMM
    void Reset() { Reset(mpMap); }
    void Reset(Map * map);
    void ReInit();                //call this when switching to a new map
    void SwitchMap();

    // GUI Interface:
    void GUICommandHandler(std::string sCommand, std::string sParams);
    static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);


  // Member variables:
  private:
    std::vector<Map*> &mvpMaps;       // The vector of maps
    Map *mpMap;                       // The current map
    Map *mpNewMap;                     // The new map, used as a temp placeholder
    Map *mpSwitchMap;                  // The switch map, used as a temp placeholder

    // GUI Interface:
    struct Command {std::string sCommand; std::string sParams; };
    std::vector<Command> mvQueuedCommands;

    double mdMaxKFDistWiggleMult;
    double mdWiggleScale;  // Metric distance between the first two KeyFrames (copied from GVar)
                          // This sets the scale of the map
    double mdWiggleScaleDepthNormalized;  // The above normalized against scene depth,
                                          // this controls keyframe separation

    // Thread interaction signalling stuff
    bool mbMapTransformRequested;
    bool mbMapScaleRequested;
    bool mbResetRequested;            // A reset has been requested
    bool mbResetDone;                 // The reset was done.
    bool mbBundleAbortRequested;      // We should stop bundle adjustment
    bool mbBundleRunning;             // Bundle adjustment is running
    bool mbBundleRunningIsRecent;     //    ... and it's a local bundle adjustment.

    // Used for requesting map transforms
    SE3<> mse3NewFromOld;
    double mdMapScaleFactor;

    //PTAMM
    bool mbReInitRequested;           // map reinitialization requested
    bool mbReInitDone;                // map reinitialization done
    bool mbSwitchRequested;           // switch to another map requested
    bool mbSwitchDone;                // switch to another map done
};

}

#endif

