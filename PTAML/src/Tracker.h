//-*- C++ -*-
// Copyright 2008 Isis Innovation Limited
//
// This header declares the Tracker class.
// The Tracker is one of main components of the system,
// and is responsible for determining the pose of a camera
// from a video feed. It uses the Map to track, and communicates
// with the MapMaker (which runs in a different thread)
// to help construct this map.
//
// Initially there is no map, so the Tracker also has a mode to
// do simple patch tracking across a stereo pair. This is handled
// by the TrackForInitialMap() method and associated sub-methods.
// Once there is a map, TrackMap() is used.
//
// Externally, the tracker should be used by calling TrackFrame()
// with every new input video frame. This then calls either
// TrackForInitialMap() or TrackMap() as appropriate.
//

#ifndef __TRACKER_H
#define __TRACKER_H

#include "MapMaker.h"
#include "ATANCamera.h"
#include "MiniPatch.h"
#include "Relocaliser.h"
#include "ARToolkit.h"

#include <cvd/image_ref.h>
#include <TooN/TooN.h>
#include <TooN/sim3.h>

#include <sstream>
#include <vector>

namespace PTAMM {

const int NUM_LOST_FRAMES = 3;

struct TrackerDrawData {
  std::vector<CVD::ImageRef> vCorners;
  std::vector<std::pair<int, TooN::Vector<2>>> vMapPoints;
  bool bDidCoarse;
  TooN::SE3<> se3CamFromWorld;
};

class TrackerData;
class PerformanceMonitor;

class Tracker
{
  public:
    Tracker(const CVD::ImageRef &irVideoSize, const ATANCamera &c, Map *m,
            MapMaker *mm, Relocaliser *pRelocaliser, PerformanceMonitor *pPerfMon);

    // TrackFrame is the main working part of the tracker: call this every frame.
    void ProcessFrame(KeyFrame &keyFrame, bool bRunTracker);

    void GetDrawData(TrackerDrawData &drawData);
    // Gets messages to be printed on-screen for the user.
    std::string GetMessageForUser() const;

    bool IsLost() const { return mnLostFrames > NUM_LOST_FRAMES; }
    const SE3<>& GetCurrentPose() const{ return mse3CamFromWorld; }
    void SetCurrentPose(const SE3<> &se3Pose) {
     mse3StartPos = mse3CamFromWorld = se3Pose;
    }
    Vector<3> RealWorldCoordinate() const {
      return mse3CamFromWorld.inverse().get_translation();
    }

    void ForceRecovery() { if (mnLostFrames < NUM_LOST_FRAMES) mnLostFrames = NUM_LOST_FRAMES; }
    void Reset();                   // Restart from scratch. Also tells the mapmaker to reset itself.

  private:
    void InitTracking();
    void ResetCommon();              // Common reset code for the following two functions

    // Methods for tracking the map once it has been made:
    void TrackMap();                // Called by TrackFrame if there is a map.

    void FindPVS(std::vector<TrackerData*> avPVS[]);
    void TrackCoarse(std::vector<TrackerData*> avPVS[]);
    void TrackFine(std::vector<TrackerData*> avPVS[]);
    void UpdateCurrentKeyframeWithNewTrackingData();

    void UpdateStatsMessage();

    bool IsDistanceToNearestKeyFrameExcessive();
    void AssessTrackingQuality();   // Heuristics to choose between good, poor, bad.
    void ApplyMotionModel();        // Decaying velocity motion model applied prior to TrackMap
    void UpdateMotionModel();       // Motion model is updated after TrackMap
    int SearchForPoints(std::vector<TrackerData*> &vTD,
                        int nRange,
                        int nFineIts);  // Finds points in the image
    Vector<6> CalcPoseUpdate(std::vector<TrackerData*>& vTD,
                             double dOverrideSigma = 0.0,
                             bool bMarkOutliers = false); // Updates pose from found points.
    void CalcSBIRotation();

    bool HasGoodCoverage();
    double DistanceToClosestKeyFrame();
    bool ShouldAddNewKeyFrame();
    void AddNewKeyFrame();          // Gives the current frame to the mapmaker to use as a keyframe

    bool AttemptRecovery();         // Called by TrackFrame if tracking is lost.

  private:
    // The major components to which the tracker needs access:
    Map *mpMap;                     // The map, consisting of points and keyframes
    MapMaker *mpMapMaker;           // The class which maintains the map
    ATANCamera mCamera;             // Projection model
    Relocaliser *mpRelocaliser;     // Relocalisation module
    PerformanceMonitor *mpPerfMon;

    CVD::ImageRef mirSize;          // Image size of whole image

    KeyFrame *mpCurrentKF;            // The current working frame as a keyframe struct
    SE3<> mse3CamFromWorld;         // Camera pose: this is what the tracker updates every frame.
    SE3<> mse3StartPos;             // What the camera pose was at the start of the frame.
    Vector<6> mv6CameraVelocity;    // Motion model
    double mdVelocityMagnitude;     // Used to decide on coarse tracking
    double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.
    bool mbDidCoarse;               // Did tracking use the coarse tracking stage?

    std::vector<TrackerData*> mvIterationSet;

    // Interface with map maker:
    int mnFrame;                    // Frames processed since last reset
    int mnLastKeyFrameDropped;      // Counter of last keyframe inserted.

    // Tracking quality control:
    int manMeasAttempted[LEVELS];
    int manMeasFound[LEVELS];
    enum {BAD, DODGY, GOOD} mTrackingQuality;
    int mnLostFrames;

    // Relocalisation functions:
    bool mbJustRecoveredSoUseCoarse;// Always use coarse tracking after recovery!

    // Frame-to-frame motion init:
    SmallBlurryImage *mpSBILastFrame;
    SmallBlurryImage *mpSBIThisFrame;
    Vector<6> mv6SBIRot;
    bool mbUseSBIInit;

    std::ostringstream mMessageForUser;
};

}

#endif

