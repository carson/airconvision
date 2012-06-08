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
InitialTracker::InitialTracker(const ImageRef &irVideoSize, const ATANCamera &c, Map *m, MapMaker &mm) :
  mCurrentKF(c),
  mFirstKF(c),
  mPreviousFrameKF(c),
  mpMap(m),
  mMapMaker(mm),
  mCamera(c),
  mirSize(irVideoSize)
{
  mCurrentKF.bFixed = false;

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
  mCurrentKF.dSceneDepthMean = 1.0;
  mCurrentKF.dSceneDepthSigma = 1.0;
  mStage = TRAIL_TRACKING_NOT_STARTED;
  mlTrails.clear();
  mvDeadTrails.clear();
  mCamera.SetImageSize(mirSize);
  mCurrentKF.mMeasurements.clear();

  for (int i = 0; i < LEVELS; ++i) {
    maFastCornerBarriers[i] = 15;
  }
}

// TrackFrame is called by System.cc with each incoming video frame.
// It figures out what state the tracker is in, and calls appropriate internal tracking
// functions. bDraw tells the tracker wether it should output any GL graphics
// or not (it should not draw, for example, when AR stuff is being shown.)
void InitialTracker::ProcessFrame(const Image<CVD::byte> &imFrame)
{
  mMessageForUser.str("");   // Wipe the user message clean
  // Take the input video image, and convert it into the tracker's keyframe struct
  // This does things like generate the image pyramid and find FAST corners
  mCurrentKF.mMeasurements.clear();
  mCurrentKF.MakeKeyFrame_Lite(imFrame, maFastCornerBarriers);

  TrackForInitialMap();
}

void InitialTracker::GetDrawData(InitialTrackerDrawData &drawData)
{
  drawData.vTrails.clear();
  for (auto it = mlTrails.begin(); it != mlTrails.end(); ++it) {
    drawData.vTrails.emplace_back(it->irInitialPos, it->irCurrentPos);
  }
  drawData.vDeadTrails = mvDeadTrails;
  drawData.vCorners = mCurrentKF.aLevels[0].GetCorners();
}

/**
 * Handle a key press command
 * This is a change from PTAM to enable games to use keys
 * @param sKey the key pressed
 * @return true if the key was used.
 */
bool InitialTracker::HandleKeyPress( const string& sKey )
{
  // KeyPress commands are issued by GLWindow, and passed to Tracker via System
  if(sKey == "Space")
  {
    mbUserPressedSpacebar = true;
    return true;
  }

  return false;
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

      mMapMaker.InitFromStereo(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld); // This is an async operation that will take some time
      mStage = WAITING_FOR_STEREO_INIT;
    }
    else
    {
      mMessageForUser << "Translate the camera slowly sideways, and press spacebar again to perform stereo init.";
    }
  }

  if(mStage == WAITING_FOR_STEREO_INIT) {
    if (mMapMaker.StereoInitDone()) {
      if (mpMap->IsGood()) {
        mStage = TRAIL_TRACKING_COMPLETE;
        cout << "Stereo init done, attempting to relocate..." << endl;
        //AttemptRecovery();
        // TODO; MARK THIS TRACKER AS READY
      } else {
        Reset();
      }
    } else {
      // Give the user some feedback
      mMessageForUser << "Waiting for stereo initialization...";
    }
  }
}

void InitialTracker::SampleTrailPatches(const ImageRef &start, const ImageRef &size, int nFeaturesToAdd)
{
  vector<pair<double,ImageRef> > vCornersAndSTScores;
  const std::vector<Candidate>& vCandidates = mCurrentKF.aLevels[0].GetCandidates();
  for (auto c = vCandidates.begin(); c != vCandidates.end(); ++c)  // Copy candidates into a trivially sortable vector
  {                                                                // so that we can choose the image corners with max ST score
    if (!PointInsideRect(c->irLevelPos, start, size)) {
      continue;
    }
    if(!mCurrentKF.aLevels[0].im.in_image_with_border(c->irLevelPos, MiniPatch::mnHalfPatchSize)) {
      continue;
    }
    vCornersAndSTScores.emplace_back(-1.0 * c->dSTScore, c->irLevelPos); // negative so highest score first in sorted list
  }

  sort(vCornersAndSTScores.begin(), vCornersAndSTScores.end());  // Sort according to Shi-Tomasi score

  for(size_t i = 0; i<vCornersAndSTScores.size() && nFeaturesToAdd > 0; i++) {
    Trail t;
    t.mPatch.SampleFromImage(vCornersAndSTScores[i].second, mCurrentKF.aLevels[0].im);
    t.irInitialPos = vCornersAndSTScores[i].second;
    t.irCurrentPos = t.irInitialPos;
    mlTrails.push_back(t);
    nFeaturesToAdd--;
  }
}

/**
 * The current frame is to be the first keyframe!
 */
void InitialTracker::TrailTracking_Start()
{
  mCurrentKF.MakeKeyFrame_Rest();  // This populates the Candidates list, which is Shi-Tomasi thresholded.
  mFirstKF = mCurrentKF;

  mvDeadTrails.clear();

  int nToAdd = GV3::get<int>("MaxInitialTrails", 1000, SILENT);

  const int CELL_WIDTH = 64;//128;
  const int CELL_HEIGHT = 60;//120;

  int cellCols = mCurrentKF.aLevels[0].im.size().x / CELL_WIDTH;
  int cellRows = mCurrentKF.aLevels[0].im.size().y / CELL_HEIGHT;

  int nFeaturesToAddPerCell = nToAdd / (cellCols * cellRows);

  mlTrails.clear();

  for (int y = 0; y < cellRows; ++y) {
    for (int x = 0; x < cellCols; ++x) {
      SampleTrailPatches(ImageRef(x * CELL_WIDTH, y * CELL_HEIGHT),
                         ImageRef(CELL_WIDTH, CELL_HEIGHT),
                         nFeaturesToAddPerCell);
    }
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
  Level &lCurrentFrame = mCurrentKF.aLevels[0];
  Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];

  for (auto i = mlTrails.begin(); i != mlTrails.end(); ) {
    Trail &trail = *i;
    ImageRef irStart = trail.irCurrentPos;
    ImageRef irEnd = irStart;
    bool bFound = trail.mPatch.FindPatch(irEnd, lCurrentFrame.im, 10, lCurrentFrame.GetCorners());
    if(bFound)
    {
      // Also find backwards in a married-matches check
      BackwardsPatch.SampleFromImage(irEnd, lCurrentFrame.im);
      ImageRef irBackWardsFound = irEnd;
      bFound = BackwardsPatch.FindPatch(irBackWardsFound, lPreviousFrame.im, 10, lPreviousFrame.GetCorners());
      if((irBackWardsFound - irStart).mag_squared() > 2)
        bFound = false;

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

  mPreviousFrameKF = mCurrentKF;
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
