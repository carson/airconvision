#include "InitialTracker.h"

#include "MEstimator.h"
#include "ShiTomasi.h"
#include "SmallMatrixOpts.h"
#include "PatchFinder.h"
#include "TrackerData.h"
#include "Utils.h"
#include "Timing.h"

#include <cvd/utility.h>
#include <cvd/gl_helpers.h>
#include <cvd/fast_corner.h>
#include <cvd/vision.h>
#include <TooN/wls.h>
#include <gvars3/instances.h>
#include <gvars3/GStringUtil.h>

#include <fstream>
#include <fcntl.h>

namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;

/**
 * The constructor mostly sets up interal reference variables to the other classes..
 * @param irVideoSize video image size
 * @param c camera model
 * @param maps list of maps
 * @param m current map
 * @param mm map maker
 */
InitialTracker::InitialTracker(const ImageRef &irVideoSize, const ATANCamera &c,
                               Map *m, MapMaker *mm)
  : mpMap(m)
  , mpMapMaker(mm)
  , mCamera(c)
  , mirSize(irVideoSize)
  , mpCurrentKF(nullptr)
  , mFirstKF(c)
  , mPreviousFrameKF(c)
{
  // Most of the initialisation is done in Reset()
  Reset();
}

/**
 * Resets the tracker, wipes the map.
 * This is the main Reset-handler-entry-point of the program!
 * Other classes' resets propagate from here.
 * It's always called in the Tracker's thread, often as a GUI command.
 */
void InitialTracker::Reset()
{
  mbUserPressedSpacebar = false;
  mStage = TRAIL_TRACKING_NOT_STARTED;
  mlTrails.clear();
  mvDeadTrails.clear();
  mCamera.SetImageSize(mirSize);
}

// TrackFrame is called by System.cc with each incoming video frame.
// It figures out what state the tracker is in, and calls appropriate internal tracking
// functions. bDraw tells the tracker wether it should output any GL graphics
// or not (it should not draw, for example, when AR stuff is being shown.)
void InitialTracker::ProcessFrame(KeyFrame &keyFrame)
{
  mMessageForUser.str("");   // Wipe the user message clean
  mpCurrentKF = &keyFrame;
  TrackForInitialMap();
}

void InitialTracker::GetDrawData(InitialTrackerDrawData &drawData)
{
  drawData.vTrails.clear();
  for (auto it = mlTrails.begin(); it != mlTrails.end(); ++it) {
    drawData.vTrails.emplace_back(it->irInitialPos, it->irCurrentPos);
  }
  drawData.vDeadTrails = mvDeadTrails;
  mpCurrentKF->aLevels[0].GetAllFeatures(drawData.vCorners);
}

/**
 * Routine for establishing the initial map. This requires two spacebar presses from the user
 * to define the first two key-frames. Salient points are tracked between the two keyframes
 * using cheap frame-to-frame tracking (which is very brittle - quick camera motion will
 * break it.) The salient points are stored in a list of `Trail' data structures.
 * What action TrackForInitialMap() takes depends on the mnInitialStage enum variable..
 */
void InitialTracker::TrackForInitialMap()
{
  // MiniPatch tracking threshhold.
  static gvar3<int> gvnMaxSSD("Tracker.MiniPatchMaxSSD", 100000, SILENT);
  MiniPatch::mnMaxSSD = *gvnMaxSSD;

  // What stage of initial tracking are we at?
  if(mStage == TRAIL_TRACKING_NOT_STARTED)
  {
    if(mbUserPressedSpacebar)  // First spacebar = this is the first keyframe
    {
      mbUserPressedSpacebar = false;
      TrailTracking_Start();
      mStage = TRAIL_TRACKING_STARTED;
    }
    else
    {
      mMessageForUser << "Point camera at planar scene and press spacebar to start tracking for initial map.";
    }
    return;
  }

  if(mStage == TRAIL_TRACKING_STARTED)
  {
    int nGoodTrails = TrailTracking_Advance();  // This call actually tracks the trails
    if(nGoodTrails < 10) // if most trails have been wiped out, no point continuing.
    {
      Reset();
      return;
    }

    // If the user pressed spacebar here, use trails to run stereo and make the intial map..
    if(mbUserPressedSpacebar)
    {
      mbUserPressedSpacebar = false;
      vector<pair<ImageRef, ImageRef> > vMatches;   // This is the format the mapmaker wants for the stereo pairs
      for (auto i = mlTrails.begin(); i!=mlTrails.end(); ++i) {
        vMatches.push_back(pair<ImageRef, ImageRef>(i->irInitialPos, i->irCurrentPos));
      }

      mpMapMaker->InitFromStereo(mFirstKF, *mpCurrentKF, vMatches, &mse3CamFromWorld); // This is an async operation that will take some time
      mStage = WAITING_FOR_STEREO_INIT;
    }
    else
    {
      mMessageForUser << "Translate the camera slowly sideways, and press spacebar again to perform stereo init.";
    }
  }

  if(mStage == WAITING_FOR_STEREO_INIT) {
    if (mpMapMaker->StereoInitDone()) {
      if (mpMap->IsGood()) {
        mStage = TRAIL_TRACKING_COMPLETE;
        cout << "Stereo init done!" << endl;
      } else {
        Reset();
      }
    } else {
      // Give the user some feedback
      mMessageForUser << "Waiting for stereo initialization...";
    }
  }
}

/**
 * The current frame is to be the first keyframe!
 */
void InitialTracker::TrailTracking_Start()
{
  mFirstKF = *mpCurrentKF;

  mvDeadTrails.clear();
  mlTrails.clear();

  int nToAdd = GV3::get<int>("MaxInitialTrails", 1000, SILENT);

  std::vector<ImageRef> vFeatures;
  mpCurrentKF->aLevels[0].GetBestFeatures(nToAdd, vFeatures);

  const CVD::Image<CVD::byte>& im = mpCurrentKF->aLevels[0].GetImage();

  for (auto it = vFeatures.begin(); it != vFeatures.end(); ++it)  // Copy candidates into a trivially sortable vector
  {                                                                // so that we can choose the image corners with max ST score
    if (!im.in_image_with_border(*it, MiniPatch::mnHalfPatchSize)) {
      continue;
    }

    Trail t;
    t.mPatch.SampleFromImage(*it, im);
    t.irInitialPos = *it;
    t.irCurrentPos = t.irInitialPos;
    mlTrails.push_back(t);
  }

  mPreviousFrameKF = mFirstKF;  // Always store the previous frame so married-matching can work.
}

/**
 * Steady-state trail tracking: Advance from the previous frame, remove duds.
 * @return number of good trails
 */
int InitialTracker::TrailTracking_Advance()
{
  int nGoodTrails = 0;

  MiniPatch BackwardsPatch;
  Level &lCurrentFrame = mpCurrentKF->aLevels[0];
  Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];

  std::vector<ImageRef> vCurrentSearchPoints;
  lCurrentFrame.GetAllFeatures(vCurrentSearchPoints);

  std::vector<ImageRef> vPrevSearchPoints;
  lPreviousFrame.GetAllFeatures(vPrevSearchPoints);

  for (auto i = mlTrails.begin(); i != mlTrails.end(); ) {
    Trail &trail = *i;
    ImageRef irStart = trail.irCurrentPos;
    ImageRef irEnd = irStart;

    bool bFound = trail.mPatch.FindPatch(irEnd, lCurrentFrame.GetImage(), 10, vCurrentSearchPoints);

    if(bFound) {
      // Also find backwards in a married-matches check
      BackwardsPatch.SampleFromImage(irEnd, lCurrentFrame.GetImage());
      ImageRef irBackWardsFound = irEnd;

      bFound = BackwardsPatch.FindPatch(irBackWardsFound, lPreviousFrame.GetImage(), 10, vPrevSearchPoints);

      if((irBackWardsFound - irStart).mag_squared() > 2) {
        bFound = false;
      }

      trail.irCurrentPos = irEnd;
      nGoodTrails++;
    }

    if(!bFound) // Erase from list of trails if not found this frame.
    {
      mvDeadTrails.push_back(i->irInitialPos);
      i = mlTrails.erase(i);
    } else {
      ++i;
    }
  }

  mPreviousFrameKF = *mpCurrentKF;
  return nGoodTrails;
}

/**
 * Return the user infor message
 * @return message string
 */
string InitialTracker::GetMessageForUser() const
{
  return mMessageForUser.str();
}

}
