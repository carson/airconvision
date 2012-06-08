#ifndef __INITIAL_TRACKER_H
#define __INITIAL_TRACKER_H

#include "MapMaker.h"
#include "ATANCamera.h"
#include "MiniPatch.h"

#include <TooN/sim3.h>

#include <sstream>
#include <vector>

namespace PTAMM {

class TrackerData;

// This struct is used for initial correspondences of the first stereo pair.
struct Trail
{
  MiniPatch mPatch;
  CVD::ImageRef irCurrentPos;
  CVD::ImageRef irInitialPos;
};


struct InitialTrackerDrawData {
  std::vector<std::pair<CVD::ImageRef, CVD::ImageRef>> vTrails;
  std::vector<CVD::ImageRef> vDeadTrails;
  std::vector<CVD::ImageRef> vCorners;
};

class InitialTracker
{
  public:
    InitialTracker(const CVD::ImageRef &irVideoSize, const ATANCamera &c, Map *m, MapMaker &mm);

    // TrackFrame is the main working part of the tracker: call this every frame.
    void ProcessFrame(const CVD::Image<CVD::byte> &imFrame);

    void GetDrawData(InitialTrackerDrawData &drawData);

    bool IsDone() const { return mStage == TRAIL_TRACKING_COMPLETE; }

    void Reset();

    void UserInvoke() { mbUserPressedSpacebar = true; }

    // Gets messages to be printed on-screen for the user.
    std::string GetMessageForUser() const;

    bool HandleKeyPress( const std::string& sKey );    // act on a key press (new addition for PTAMM)

  private:
    // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
    void TrackForInitialMap();      // This is called by TrackFrame if there is not a map yet.
    void TrailTracking_Start();     // First frame of initial trail tracking. Called by TrackForInitialMap.
    int  TrailTracking_Advance();   // Steady-state of initial trail tracking. Called by TrackForInitialMap.
    void SampleTrailPatches(const CVD::ImageRef &start, const CVD::ImageRef &size, int nFeaturesToAdd);

  private:
    enum InitialTrackingStage {
      TRAIL_TRACKING_NOT_STARTED,
      TRAIL_TRACKING_STARTED,
      WAITING_FOR_STEREO_INIT,
      TRAIL_TRACKING_COMPLETE
    };

    KeyFrame mCurrentKF;            // The current working frame as a keyframe struct
    KeyFrame mFirstKF;              // First of the stereo pair
    KeyFrame mPreviousFrameKF;      // Used by trail tracking to check married matches

    // The major components to which the tracker needs access:
    Map *mpMap;                     // The map, consisting of points and keyframes
    MapMaker &mMapMaker;            // The class which maintains the map
    ATANCamera mCamera;             // Projection model

    CVD::ImageRef mirSize;          // Image size of whole image

    InitialTrackingStage mStage;  // How far are we towards making the initial map?

    std::vector<Trail> mlTrails;      // Used by trail tracking
    std::vector<CVD::ImageRef> mvDeadTrails;
    SE3<> mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.

    int maFastCornerBarriers[LEVELS];

    // User interaction for initial tracking:
    bool mbUserPressedSpacebar;
    std::ostringstream mMessageForUser;
};

}

#endif
