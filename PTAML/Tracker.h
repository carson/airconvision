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
#include "TakFrame.h" //@hack by camparijet

#include <TooN/sim3.h>

#include <sstream>
#include <vector>
#include <list>

namespace PTAMM {

const int NUM_LOST_FRAMES = 3;

class TrackerData;

// This struct is used for initial correspondences of the first stereo pair.
struct Trail
{
  MiniPatch mPatch;
  CVD::ImageRef irCurrentPos;
  CVD::ImageRef irInitialPos;
};

class Tracker
{
  public:
    Tracker(CVD::ImageRef irVideoSize, const ATANCamera &c, std::vector<Map*> &maps, Map *m, MapMaker &mm, ARToolkitTracker& arTracker);

    // TrackFrame is the main working part of the tracker: call this every frame.
    void TrackFrame(const CVD::Image<CVD::byte> &imFrame, bool bDraw);

    const SE3<>& GetCurrentPose() const{ return mse3CamFromWorld; }
    bool IsLost() const { return (mnLostFrames > NUM_LOST_FRAMES); }

    Vector<3> RealWorldCoordinate() const {
      return mse3CamFromWorld.inverse().get_translation();
    }

    // Gets messages to be printed on-screen for the user.
    std::string GetMessageForUser() const;

    bool SwitchMap(Map *map);
    void SetNewMap(Map * map);
    void ForceRecovery() { if(mnLostFrames < NUM_LOST_FRAMES) mnLostFrames = NUM_LOST_FRAMES; }
    void Reset();                   // Restart from scratch. Also tells the mapmaker to reset itself.

    bool HandleKeyPress( const std::string& sKey );    // act on a key press (new addition for PTAMM)

  private:
    void ResetCommon();              // Common reset code for the following two functions
    void RenderGrid();              // Draws the reference grid
    // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
    void TrackForInitialMap();      // This is called by TrackFrame if there is not a map yet.

    void TrailTracking_Start();     // First frame of initial trail tracking. Called by TrackForInitialMap.
    int  TrailTracking_Advance();   // Steady-state of initial trail tracking. Called by TrackForInitialMap.
    void SampleTrailPatches(const CVD::ImageRef &start, const CVD::ImageRef &size, int nFeaturesToAdd);

    // Methods for tracking the map once it has been made:
    void TrackMap();                // Called by TrackFrame if there is a map.

    void FindPVS(std::vector<TrackerData*> avPVS[]);
    void TrackCoarse(std::vector<TrackerData*> avPVS[]);
    void TrackFine(std::vector<TrackerData*> avPVS[]);
    void UpdateCurrentKeyframeWithNewTrackingData();
    void DrawMapPoints() const;

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

    bool ShouldAddNewKeyFrame();
    void AddNewKeyFrame();          // Gives the current frame to the mapmaker to use as a keyframe

    bool AttemptRecovery();         // Called by TrackFrame if tracking is lost.

    // Scale initialization with markers -- dhenell
    Vector<2> ProjectPoint(const Vector<3> &v3Point);
    void DrawMarkerPose(const SE3<> &se3WorldFromNormWorld);
    void DetermineScaleFromMarker(const CVD::Image<CVD::byte> &imFrame);
    bool PickPointOnGround(const TooN::Vector<2>& pixelCoord,
                           TooN::Vector<3>& pointOnPlane);

    void GUICommandHandler(const std::string& sCommand, const std::string& sParams);
    static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);

  private:
    KeyFrame mCurrentKF;            // The current working frame as a keyframe struct

    // The major components to which the tracker needs access:
    std::vector<Map*> & mvpMaps;     // Reference to all of the maps
    Map *mpMap;                     // The map, consisting of points and keyframes
    MapMaker &mMapMaker;            // The class which maintains the map
    ATANCamera mCamera;             // Projection model
    Relocaliser mRelocaliser;       // Relocalisation module
    ARToolkitTracker& mARTracker;   // Tracker of markers

    CVD::ImageRef mirSize;          // Image size of whole image

    enum {TRAIL_TRACKING_NOT_STARTED,
          TRAIL_TRACKING_STARTED,
          WAITING_FOR_STEREO_INIT,
          TRAIL_TRACKING_COMPLETE,
          MARKER_INIT_COMPLETE} mnInitialStage;  // How far are we towards making the initial map?

    std::list<Trail> mlTrails;      // Used by trail tracking
    std::vector<CVD::ImageRef> mvDeadTrails;

    KeyFrame mFirstKF;              // First of the stereo pair
    KeyFrame mPreviousFrameKF;      // Used by trail tracking to check married matches

    std::vector<TrackerData*> mvIterationSet;

    int maFastCornerBarriers[LEVELS];

    SE3<> mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.
    SE3<> mse3StartPos;               // What the camera pose was at the start of the frame.
    Vector<6> mv6CameraVelocity;    // Motion model
    double mdVelocityMagnitude;     // Used to decide on coarse tracking
    double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.
    bool mbDidCoarse;               // Did tracking use the coarse tracking stage?

    bool mbDraw;                    // Should the tracker draw anything to OpenGL?

    bool mbFreezeTracking;
    bool mbForceAddNewKeyFrame;     // Forces the adding of the next keyframe

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

    // User interaction for initial tracking:
    bool mbUserPressedSpacebar;
    std::ostringstream mMessageForUser;

    // GUI interface:
    struct Command {std::string sCommand; std::string sParams; };
    std::vector<Command> mvQueuedCommands;
    //@hack for serialize tracked frame
    int frameIndex;
    int isKeyFrame;
    bool bSave;

    bool mHasDeterminedScale;
};

}

#endif

