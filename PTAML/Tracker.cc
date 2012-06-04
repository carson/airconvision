// Copyright 2008 Isis Innovation Limited
#include "OpenGL.h"
#include "Tracker.h"
#include "MEstimator.h"
#include "ShiTomasi.h"
#include "SmallMatrixOpts.h"
#include "PatchFinder.h"
#include "TrackerData.h"
#include "ARToolkit.h"
#include "Utils.h"
#include "GLWindow2.h"
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

#include <cvd/image_io.h>//@hack by camaparijet
#include <sstream> //@hack by camparijet  for  saving image

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
Tracker::Tracker(ImageRef irVideoSize, const ATANCamera &c, std::vector<Map*> &maps, Map *m, MapMaker &mm, ARToolkitTracker& arTracker) :
  mCurrentKF(c),
  mvpMaps(maps),
  mpMap(m),
  mMapMaker(mm),
  mCamera(c),
  mRelocaliser(maps, mCamera),
  mARTracker(arTracker),
  mirSize(irVideoSize),
  mFirstKF(mCamera),
  mPreviousFrameKF(mCamera),
  mbFreezeTracking(false),

  frameIndex(0), //@hack by camparijet for serialize allframe
  mHasDeterminedScale(false)
{
  mCurrentKF.bFixed = false;
  GUI.RegisterCommand("Reset", GUICommandCallBack, this);
  GUI.RegisterCommand("PokeTracker", GUICommandCallBack, this);
  TrackerData::irImageSize = mirSize;

  mpSBILastFrame = NULL;
  mpSBIThisFrame = NULL;

  // Most of the initialisation is done in Reset()
  Reset();
}

/**
 * Common reset code for Reset() and ResetAll()
 */
void Tracker::ResetCommon()
{
  mbDidCoarse = false;
  mbUserPressedSpacebar = false;
  mTrackingQuality = GOOD;
  mnLostFrames = 0;
  mdMSDScaledVelocityMagnitude = 0;
  mCurrentKF.dSceneDepthMean = 1.0;
  mCurrentKF.dSceneDepthSigma = 1.0;
  mnInitialStage = TRAIL_TRACKING_NOT_STARTED;
  mlTrails.clear();
  mCamera.SetImageSize(mirSize);
  mCurrentKF.mMeasurements.clear();
  mnLastKeyFrameDropped = -20;
  mnFrame=0;
  mv6CameraVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = false;
  mHasDeterminedScale = false;

  for (int i = 0; i < LEVELS; ++i) {
    maFastCornerBarriers[i] = 15;
  }
}

/**
 * Resets the tracker, wipes the map.
 * This is the main Reset-handler-entry-point of the program!
 * Other classes' resets propagate from here.
 * It's always called in the Tracker's thread, often as a GUI command.
 */
void Tracker::Reset()
{
  if( mpMap->bEditLocked ) {
    cerr << "MAP LOCKED: Cannot reset map " << mpMap->MapID() << "." << endl;
    return;
  }

  ResetCommon();

  // Tell the MapMaker to reset itself..
  // this may take some time, since the mapmaker thread may have to wait
  // for an abort-check during calculation, so sleep while waiting.
  // MapMaker will also clear the map.
  mMapMaker.RequestReset();
  while(!mMapMaker.ResetDone()) {
#ifndef WIN32
    usleep(10);
#else
    Sleep(1);
#endif
  }
}

bool Tracker::PickPointOnGround(
  const TooN::Vector<2>& pixelCoord,
  TooN::Vector<3>& pointOnPlane)
{
  Vector<2> v2VidCoords = pixelCoord;

  Vector<2> v2UFBCoords;
#ifdef WIN32
  Vector<2> v2PlaneCoords;   v2PlaneCoords[0] = numeric_limits<double>::quiet_NaN();   v2PlaneCoords[1] = numeric_limits<double>::quiet_NaN();
#else
  Vector<2> v2PlaneCoords;   v2PlaneCoords[0] = NAN;   v2PlaneCoords[1] = NAN;
#endif
  Vector<3> v3RayDirn_W;

  // Work out image coords 0..1:
  v2UFBCoords[0] = (v2VidCoords[0] + 0.5) / mCamera.GetImageSize()[0];
  v2UFBCoords[1] = (v2VidCoords[1] + 0.5) / mCamera.GetImageSize()[1];

  // Work out plane coords:
  Vector<2> v2ImPlane = mCamera.UnProject(v2VidCoords);
  Vector<3> v3C = unproject(v2ImPlane);
  Vector<4> v4C = unproject(v3C);
  SE3<> se3CamInv = mse3CamFromWorld.inverse();
  Vector<4> v4W = se3CamInv * v4C;
  double t = se3CamInv.get_translation()[2];
  double dDistToPlane = -t / (v4W[2] - t);

  if(v4W[2] -t <= 0) // Clicked the wrong side of the horizon?
  {
    v4C.slice<0,3>() *= dDistToPlane;
    Vector<4> v4Result = se3CamInv * v4C;
    pointOnPlane = v4Result.slice<0,3>(); // <--- result

    return true;
  }

  return false;
}

Vector<2> Tracker::ProjectPoint(const Vector<3> &v3Point)
{
  Vector<3> v3Cam = mse3CamFromWorld * v3Point;

  if(v3Cam[2] < 0.001) {
    v3Cam[2] = 0.001;
  }

  return mCamera.Project(project(v3Cam));
}

void Tracker::DrawMarkerPose(const SE3<> &se3WorldFromNormWorld)
{
  glEnable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
  glLineWidth(2);
  glPointSize(15);

  glBegin(GL_POINTS);
    glColor3f(0,1,1);
    glVertex(ProjectPoint(se3WorldFromNormWorld.get_translation()));
  glEnd();

  Vector<3> wo2 = se3WorldFromNormWorld * makeVector(0, 0, 0);
  Vector<3> wx2 = se3WorldFromNormWorld * makeVector(8, 0, 0);
  Vector<3> wy2 = se3WorldFromNormWorld * makeVector(0, 8, 0);
  Vector<3> wz2 = se3WorldFromNormWorld * makeVector(0, 0, 8);

  std::vector<Vector<3> > pts;

  pts.push_back(wo2); pts.push_back(wx2);
  pts.push_back(wo2); pts.push_back(wy2);
  pts.push_back(wo2); pts.push_back(wz2);

  glBegin(GL_LINES);

  std::vector<Vector<2> > screenPts;
  for (auto it = pts.begin(); it != pts.end(); ++it) {
    glColor3f(1,0,0);
    glVertex(ProjectPoint(*it));
  }

  glEnd();

  glLineWidth(1);
  glPointSize(1);
}

void Tracker::DetermineScaleFromMarker(const Image<CVD::byte> &imFrame)
{
  if (!mHasDeterminedScale) {
    if (mARTracker.Track(imFrame, mbDraw)) {
      // Get corner points of the marker in screen space
      std::vector<Vector<2> > imPts;
      mARTracker.GetMarkerCorners(imPts);

      // Project the corners onto the ground plane to get
      // the coordinates in the PTAM world space
      std::vector<Vector<3> > pointsOnPlane;
      for (auto it = imPts.begin(); it != imPts.end(); ++it) {
        Vector<3> pointOnPlane;
        if (PickPointOnGround(*it, pointOnPlane)) {
          pointsOnPlane.push_back(pointOnPlane);
        }
      }

      // Abort if not all corners were found
      if (pointsOnPlane.size() != 4) {
        return;
      }

      // Determine scale and origin
      Vector<3> origin = Zeros;
      double edgeLengthSum = 0;
      Vector<3> prevPoint = pointsOnPlane[3];
      for (auto it = pointsOnPlane.begin(); it != pointsOnPlane.end(); ++it) {
        origin += *it;
        double edgeLength = norm(*it - prevPoint);
        edgeLengthSum += edgeLength;
        prevPoint = *it;
      }

      // origin is now the sum of all the corner points, so the marker center is the average
      origin *= 0.25;

      Vector<3> xAxis = pointsOnPlane[1] + pointsOnPlane[2] - pointsOnPlane[0] - pointsOnPlane[3];
      Vector<3> yAxis = pointsOnPlane[0] + pointsOnPlane[1] - pointsOnPlane[2] - pointsOnPlane[3];
      Vector<3> zAxis = xAxis ^ yAxis;
      yAxis = zAxis ^ xAxis;

      normalize(xAxis);
      normalize(yAxis);
      normalize(zAxis);

      Matrix<3,3> rot;
      rot[0] = xAxis;
      rot[1] = yAxis;
      rot[2] = zAxis;

      // The misalignment
      SE3<> se3WorldFromNormWorld = SE3<>(SO3<>(rot.T()), origin);

      // The scale
      static gvar3<double> gvdMarkerSize("Marker.Size", 0.08, SILENT); // Size of the marker in meters
      double scale = (4.0 * *gvdMarkerSize) / edgeLengthSum;

      if (mbDraw) {
        DrawMarkerPose(se3WorldFromNormWorld);
      }

      if (mbUserPressedSpacebar) {

        cout << "SCALE: " << scale << endl;

        mbFreezeTracking = true;
        //mMapMaker.RequestCallback([&] () { } );
        mMapMaker.RequestMapTransformation(se3WorldFromNormWorld.inverse());
        mMapMaker.RequestMapScaling(scale);
        mMapMaker.RequestCallback([&] () {
          mse3CamFromWorld = se3WorldFromNormWorld * mse3CamFromWorld;
          mse3CamFromWorld.get_translation() *= scale;
          mbFreezeTracking = false;
          ForceRecovery();
        } );

        mHasDeterminedScale = true;
        mbUserPressedSpacebar = false;
      }
    }
  }
}

bool Tracker::ShouldAddNewKeyFrame()
{
  /*
  if (mbForceAddNewKeyFrame) {
    mbForceAddNewKeyFrame = false;
    return true;
  }
  */

  return mTrackingQuality == GOOD &&
         mMapMaker.NeedNewKeyFrame(mCurrentKF) &&
         // HACK for keyframe threshold for incorporation
         // the parameter here determines how frequently keyframes are incorporateed
         mnFrame - mnLastKeyFrameDropped > 20  &&
         mpMap->QueueSize() < 200;
}

// TrackFrame is called by System.cc with each incoming video frame.
// It figures out what state the tracker is in, and calls appropriate internal tracking
// functions. bDraw tells the tracker wether it should output any GL graphics
// or not (it should not draw, for example, when AR stuff is being shown.)
void Tracker::TrackFrame(const Image<CVD::byte> &imFrame, bool bDraw)
{
  isKeyFrame = 0;//@hack by camparijet for serialize
  mbDraw = bDraw;
  mMessageForUser.str("");   // Wipe the user message clean

  // Take the input video image, and convert it into the tracker's keyframe struct
  // This does things like generate the image pyramid and find FAST corners
  mCurrentKF.mMeasurements.clear();

  mCurrentKF.MakeKeyFrame_Lite(imFrame, maFastCornerBarriers);

  // Update the small images for the rotation estimator
  static gvar3<double> gvdSBIBlur("Tracker.RotationEstimatorBlur", 0.75, SILENT);
  static gvar3<int> gvnUseSBI("Tracker.UseRotationEstimator", 1, SILENT);
  mbUseSBIInit = false; //*gvnUseSBI;

  gSBIInitTimer.Start();

  if(!mpSBIThisFrame)
  {
    mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
    mpSBILastFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
  }
  else
  {
    delete  mpSBILastFrame;
    mpSBILastFrame = mpSBIThisFrame;
    mpSBIThisFrame = new SmallBlurryImage(mCurrentKF, *gvdSBIBlur);
  }

  gSBIInitTimer.Stop();

  // From now on we only use the keyframe struct!
  mnFrame++;

  if(mbDraw)
  {
    glDrawPixels(mCurrentKF.aLevels[0].im);
    if(GV2.GetInt("Tracker.DrawFASTCorners",0, SILENT))
    {
      glColor3f(1,0,1);  glPointSize(1); glBegin(GL_POINTS);
      const std::vector<ImageRef> &vCorners = mCurrentKF.aLevels[0].GetCorners();
      for (auto c = vCorners.begin(); c != vCorners.end(); ++c) {
        glVertex(*c);
      }
      glEnd();
    }
  }

  if (!mbFreezeTracking) {
    // Decide what to do - if there is a map, try to track the map ...
    if(mnInitialStage == TRAIL_TRACKING_COMPLETE)
    {
      if(mnLostFrames < NUM_LOST_FRAMES)  // .. but only if we're not lost!
      {
        gSBITimer.Start();
        if(mbUseSBIInit) {
          CalcSBIRotation();
        }
        gSBITimer.Stop();

        ApplyMotionModel();       //

        gTrackTimer.Start();
        TrackMap();               //  These three lines do the main tracking work.
        gTrackTimer.Stop();

        UpdateMotionModel();      //

        gTrackingQualityTimer.Start();
        AssessTrackingQuality();  //  Check if we're lost or if tracking is poor.
        gTrackingQualityTimer.Stop();

        { // Provide some feedback for the user:
          mMessageForUser << "Tracking Map, quality ";
          if(mTrackingQuality == GOOD)  mMessageForUser << "good.";
          if(mTrackingQuality == DODGY) mMessageForUser << "poor.";
          if(mTrackingQuality == BAD)   mMessageForUser << "bad.";

          mMessageForUser << " F:";
          for(int i=0; i<LEVELS; i++) {
            mMessageForUser << "/" << maFastCornerBarriers[i];
          }

          mMessageForUser << " Found:";
          for(int i=0; i<LEVELS; i++) {
            mMessageForUser << " " << manMeasFound[i] << "/" << manMeasAttempted[i];
          }
          mMessageForUser << " Map " << mpMap->MapID() << ": "
                          << mpMap->GetMapPoints().size() << "P, " << mpMap->GetKeyFrames().size() << "KF";
        }

        // Heuristics to check if a key-frame should be added to the map:
        if(ShouldAddNewKeyFrame()) {
          mMessageForUser << " Adding key-frame.";
          AddNewKeyFrame();
          isKeyFrame = 1;
        }

        // Added some scale determing code here -- dhenell
        //DetermineScaleFromMarker(imFrame);
      }
      else  // what if there is a map, but tracking has been lost?
      {
        cout << "Lost tracking..." << endl;
        mMessageForUser << "** Attempting recovery **.";
        if(AttemptRecovery())
        {
          TrackMap();
          AssessTrackingQuality();
        }
      }
      if(mbDraw) {
        gDrawGridTimer.Start();
        RenderGrid();
        gDrawGridTimer.Stop();
      }
    }
    else // If there is no map, try to make one.
    {
      TrackForInitialMap();
    }

  }

  // GUI interface
  while(!mvQueuedCommands.empty())
  {
    GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
    mvQueuedCommands.erase(mvQueuedCommands.begin());
  }
};

/**
 * Try to relocalise in case tracking was lost.
 * Returns success or failure as a bool.
 * Actually, the SBI relocaliser will almost always return true, even if
 * it has no idea where it is, so graphics will go a bit
 * crazy when lost. Could use a tighter SSD threshold and return more false,
 * but the way it is now gives a snappier response and I prefer it.
 * @return success
 */
bool Tracker::AttemptRecovery()
{
  cout << "AttemptRecovery..." << endl;

  bool bRelocGood = mRelocaliser.AttemptRecovery( *mpMap, mCurrentKF );
  if(!bRelocGood)
    return false;

  SE3<> se3Best = mRelocaliser.BestPose();
  mse3CamFromWorld = mse3StartPos = se3Best;
  mv6CameraVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = true;
  return true;
}

/**
 * Draw the reference grid to give the user an idea of wether tracking is OK or not.
 */
void Tracker::RenderGrid()
{
  // The colour of the ref grid shows if the coarse stage of tracking was used
  // (it's turned off when the camera is sitting still to reduce jitter.)
  if(mbDidCoarse)
    glColor4f(0.0f, 0.5f, 0.0f, 0.6f);
  else
    glColor4f(0.0f,0.0f,0.0f,0.6f);

  // The grid is projected manually, i.e. GL receives projected 2D coords to draw.
  int nHalfCells = 4;
  int nTot = nHalfCells * 2 + 1;
  Image<Vector<2> >  imVertices(ImageRef(nTot,nTot));
  for(int i=0; i<nTot; i++)
  {
    for(int j=0; j<nTot; j++)
    {
      Vector<3> v3;
      v3[0] = (i - nHalfCells) * 0.1;
      v3[1] = (j - nHalfCells) * 0.1;
      v3[2] = 0.0;
      Vector<3> v3Cam = mse3CamFromWorld * v3;
      if(v3Cam[2] < 0.001) {
        v3Cam[2] = 0.001;
      }
      imVertices[i][j] = mCamera.Project(project(v3Cam));
    }
  }
  glEnable(GL_LINE_SMOOTH);
  //glEnable(GL_BLEND);
  glDisable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(1);
  for(int i=0; i<nTot; i++)
  {
    glBegin(GL_LINE_STRIP);
    for(int j=0; j<nTot; j++)
      glVertex(imVertices[i][j]);
    glEnd();

    glBegin(GL_LINE_STRIP);
    for(int j=0; j<nTot; j++)
      glVertex(imVertices[j][i]);
    glEnd();
  }

  glLineWidth(1);
  glColor3f(1,0,0);
}


/**
 * GUI interface. Stuff commands onto the back of a queue so the tracker handles
 * them in its own thread at the end of each frame. Note the charming lack of
 * any thread safety (no lock on mvQueuedCommands).
 * @param ptr object pointer
 * @param sCommand command string
 * @param sParams parameter string
 */
void Tracker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  Command c;
  c.sCommand = sCommand;
  c.sParams = sParams;
  ((Tracker*) ptr)->mvQueuedCommands.push_back(c);
}


/**
 * This is called in the tracker's own thread.
 * @param sCommand command string
 * @param sParams  parameter string
 */
void Tracker::GUICommandHandler(const string& sCommand, const string& sParams)  // Called by the callback func..
{
  if(sCommand=="Reset")
  {
    Reset();
    return;
  }
  else if((sCommand=="PokeTracker"))
  {
    mbUserPressedSpacebar = true;
    return;
  }

  cout << "! Tracker::GUICommandHandler: unhandled command "<< sCommand << endl;
  exit(1);
}


/**
 * Handle a key press command
 * This is a change from PTAM to enable games to use keys
 * @param sKey the key pressed
 * @return true if the key was used.
 */
bool Tracker::HandleKeyPress( const string& sKey )
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
void Tracker::TrackForInitialMap()
{
  // MiniPatch tracking threshhold.
  static gvar3<int> gvnMaxSSD("Tracker.MiniPatchMaxSSD", 100000, SILENT);
  MiniPatch::mnMaxSSD = *gvnMaxSSD;

  // What stage of initial tracking are we at?
  if(mnInitialStage == TRAIL_TRACKING_NOT_STARTED)
  {
    if(mbUserPressedSpacebar)  // First spacebar = this is the first keyframe
    {
      mbUserPressedSpacebar = false;
      TrailTracking_Start();
      mnInitialStage = TRAIL_TRACKING_STARTED;
      isKeyFrame = 2; //@hack by camaparijet for serializing
    }
    else
    {
      mMessageForUser << "Point camera at planar scene and press spacebar to start tracking for initial map.";
    }
    return;
  }

  if(mnInitialStage == TRAIL_TRACKING_STARTED)
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
      for(list<Trail>::iterator i = mlTrails.begin(); i!=mlTrails.end(); ++i)
        vMatches.push_back(pair<ImageRef, ImageRef>(i->irInitialPos, i->irCurrentPos));

      mMapMaker.InitFromStereo(mFirstKF, mCurrentKF, vMatches, mse3CamFromWorld); // This is an async operation that will take some time
      mnInitialStage = WAITING_FOR_STEREO_INIT;
    }
    else
    {
      mMessageForUser << "Translate the camera slowly sideways, and press spacebar again to perform stereo init.";
    }
  }

  if(mnInitialStage == WAITING_FOR_STEREO_INIT) {
    if (mMapMaker.StereoInitDone()) {
      if (mpMap->IsGood()) {
        mnInitialStage = TRAIL_TRACKING_COMPLETE;
        isKeyFrame = 2; //@hack by camaparijet for serializing
        cout << "Stereo init done, attempting to relocate..." << endl;
        AttemptRecovery();
      } else {
        Reset();
      }
    } else {
      // Give the user some feedback
      mMessageForUser << "Waiting for stereo initialization...";
    }
  }
}

void Tracker::SampleTrailPatches(const ImageRef &start, const ImageRef &size, int nFeaturesToAdd)
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
void Tracker::TrailTracking_Start()
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
int Tracker::TrailTracking_Advance()
{
  int nGoodTrails = 0;
  if(mbDraw)
  {
    glPointSize(5);
    glLineWidth(2);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glBegin(GL_LINES);
  }

  MiniPatch BackwardsPatch;
  Level &lCurrentFrame = mCurrentKF.aLevels[0];
  Level &lPreviousFrame = mPreviousFrameKF.aLevels[0];

  for (auto i = mlTrails.begin(); i != mlTrails.end(); ) {

    list<Trail>::iterator next = i; next++;

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

    if(mbDraw)
    {
      if(!bFound)
        glColor3f(0,1,1); // Failed trails flash purple before dying.
      else
        glColor3f(1,1,0);
      glVertex(trail.irInitialPos);
      if(bFound) glColor3f(1,0,0);
      glVertex(trail.irCurrentPos);
    }
    if(!bFound) // Erase from list of trails if not found this frame.
    {
      mvDeadTrails.push_back(i->irInitialPos);
      mlTrails.erase(i);
    }
    i = next;
  }
  if(mbDraw) {
    glEnd();

    glBegin(GL_POINTS);

    for (auto it = mvDeadTrails.begin(); it != mvDeadTrails.end(); ++it) {
      glColor3f(0.5,0.1,0.7);
      glVertex(*it);
    }

    glEnd();
  }

  mPreviousFrameKF = mCurrentKF;
  return nGoodTrails;
}

/**
 * TrackMap is the main purpose of the Tracker.
 * It first projects all map points into the image to find a potentially-visible-set (PVS);
 * Then it tries to find some points of the PVS in the image;
 * Then it updates camera pose according to any points found.
 * Above may happen twice if a coarse tracking stage is performed.
 * Finally it updates the tracker's current-frame-KeyFrame struct with any
 * measurements made.
 * A lot of low-level functionality is split into helper classes:
 * class TrackerData handles the projection of a MapPoint and stores intermediate results;
 * class PatchFinder finds a projected MapPoint in the current-frame-KeyFrame.
 */
void Tracker::TrackMap()
{
  // Some accounting which will be used for tracking quality assessment:
  for(int i=0; i<LEVELS; i++)
    manMeasAttempted[i] = manMeasFound[i] = 0;

  gPvsTimer.Start();

  // The Potentially-Visible-Set (PVS) is split into pyramid levels.
  vector<TrackerData*> avPVS[LEVELS];
  for(int i=0; i<LEVELS; i++)
    avPVS[i].reserve(500);

  // For all points in the map..
  const std::vector<MapPoint*>& mapPts = mpMap->GetMapPoints();
  for(size_t i=0; i < mapPts.size(); ++i)
  {
    MapPoint &p = *mapPts[i];
    // Ensure that this map point has an associated TrackerData struct.
    if(!p.pTData) {
      p.pTData = new TrackerData(&p);
    }

    TrackerData &TData = *p.pTData;

    // Project according to current view, and if it's not in the image, skip.
    TData.Project(mse3CamFromWorld, mCamera);
    if(!TData.bInImage)
      continue;

    // Calculate camera projection derivatives of this point.
    TData.GetDerivsUnsafe(mCamera);

    // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
    TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, mse3CamFromWorld, TData.m2CamDerivs);
    if(TData.nSearchLevel == -1)
      continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.

    // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
    TData.bSearched = false;
    TData.bFound = false;
    avPVS[TData.nSearchLevel].push_back(&TData);
  }

  // Next: A large degree of faffing about and deciding which points are going to be measured!
  // First, randomly shuffle the individual levels of the PVS.
  for(int i=0; i<LEVELS; i++)
    random_shuffle(avPVS[i].begin(), avPVS[i].end());

  gPvsTimer.Stop();

  // The next two data structs contain the list of points which will next
  // be searched for in the image, and then used in pose update.
  vector<TrackerData*> vNextToSearch;
  vector<TrackerData*> vIterationSet;

  // Tunable parameters to do with the coarse tracking stage:

  static gvar3<unsigned int> gvnCoarseMin("Tracker.CoarseMin", 20, SILENT);   // Min number of large-scale features for coarse stage
  static gvar3<unsigned int> gvnCoarseMax("Tracker.CoarseMax", 60, SILENT);   // Max number of large-scale features for coarse stage
  static gvar3<unsigned int> gvnCoarseRange("Tracker.CoarseRange", 30, SILENT);       // Pixel search radius for coarse features
  static gvar3<int> gvnCoarseSubPixIts("Tracker.CoarseSubPixIts", 8, SILENT); // Max sub-pixel iterations for coarse features
  static gvar3<int> gvnCoarseDisabled("Tracker.DisableCoarse", 0, SILENT);    // Set this to 1 to disable coarse stage (except after recovery)
  static gvar3<double> gvdCoarseMinVel("Tracker.CoarseMinVelocity", 0.006, SILENT);  // Speed above which coarse stage is used.

  unsigned int nCoarseMax = *gvnCoarseMax;
  unsigned int nCoarseRange = *gvnCoarseRange;

  mbDidCoarse = false;

  // Set of heuristics to check if we should do a coarse tracking stage.
  bool bTryCoarse = true;
  if(*gvnCoarseDisabled ||
     mdMSDScaledVelocityMagnitude < *gvdCoarseMinVel  ||
     nCoarseMax == 0)
  {
    bTryCoarse = false;
  }

  if(mbJustRecoveredSoUseCoarse)
  {
    bTryCoarse = true;
    nCoarseMax *=2;
    nCoarseRange *=2;
    mbJustRecoveredSoUseCoarse = false;
  }

  // If we do want to do a coarse stage, also check that there's enough high-level
  // PV map points. We use the lowest-res two pyramid levels (LEVELS-1 and LEVELS-2),
  // with preference to LEVELS-1.
  if(bTryCoarse && avPVS[LEVELS-1].size() + avPVS[LEVELS-2].size() > *gvnCoarseMin )
  {
    // Now, fill the vNextToSearch struct with an appropriate number of
    // TrackerDatas corresponding to coarse map points! This depends on how many
    // there are in different pyramid levels compared to CoarseMin and CoarseMax.

    if(avPVS[LEVELS-1].size() <= nCoarseMax)
    { // Fewer than CoarseMax in LEVELS-1? then take all of them, and remove them from the PVS list.
      vNextToSearch = avPVS[LEVELS-1];
      avPVS[LEVELS-1].clear();
    }
    else
    { // ..otherwise choose nCoarseMax at random, again removing from the PVS list.
      for(unsigned int i=0; i<nCoarseMax; i++) {
        vNextToSearch.push_back(avPVS[LEVELS-1][i]);
      }
      avPVS[LEVELS-1].erase(avPVS[LEVELS-1].begin(), avPVS[LEVELS-1].begin() + nCoarseMax);
    }

    // If didn't source enough from LEVELS-1, get some from LEVELS-2... same as above.
    if(vNextToSearch.size() < nCoarseMax)
    {
      unsigned int nMoreCoarseNeeded = nCoarseMax - vNextToSearch.size();
      if(avPVS[LEVELS-2].size() <= nMoreCoarseNeeded)
      {
        vNextToSearch = avPVS[LEVELS-2];
        avPVS[LEVELS-2].clear();
      }
      else
      {
        for(unsigned int i=0; i<nMoreCoarseNeeded; i++) {
          vNextToSearch.push_back(avPVS[LEVELS-2][i]);
        }
        avPVS[LEVELS-2].erase(avPVS[LEVELS-2].begin(), avPVS[LEVELS-2].begin() + nMoreCoarseNeeded);
      }
    }

    // Now go and attempt to find these points in the image!
    unsigned int nFound = SearchForPoints(vNextToSearch, nCoarseRange, *gvnCoarseSubPixIts);
    vIterationSet = vNextToSearch;  // Copy over into the to-be-optimised list.
    if(nFound >= *gvnCoarseMin)  // Were enough found to do any meaningful optimisation?
    {
      mbDidCoarse = true;
      for(int iter = 0; iter<10; iter++) // If so: do ten Gauss-Newton pose updates iterations.
      {
        if(iter != 0)
        { // Re-project the points on all but the first iteration.
          for(unsigned int i=0; i<vIterationSet.size(); i++) {
            if(vIterationSet[i]->bFound) {
              vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
            }
          }
        }
        for(unsigned int i=0; i<vIterationSet.size(); i++) {
          if(vIterationSet[i]->bFound) {
            vIterationSet[i]->CalcJacobian();
          }
        }
        double dOverrideSigma = 0.0;
        // Hack: force the MEstimator to be pretty brutal
        // with outliers beyond the fifth iteration.
        if(iter > 5)
          dOverrideSigma = 1.0;

        // Calculate and apply the pose update...
        Vector<6> v6Update = CalcPoseUpdate(vIterationSet, dOverrideSigma);
        mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
      }
    }
  }

  // So, at this stage, we may or may not have done a coarse tracking stage.
  // Now do the fine tracking stage. This needs many more points!

  int nFineRange = 10;  // Pixel search range for the fine stage.
  if(mbDidCoarse) {     // Can use a tighter search if the coarse stage was already done.
    nFineRange = 5;
  }

  // What patches shall we use this time? The high-level ones are quite important,
  // so do all of these, with sub-pixel refinement.
  {
    int l = LEVELS - 1;
    for(unsigned int i=0; i<avPVS[l].size(); i++)
      avPVS[l][i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
    SearchForPoints(avPVS[l], nFineRange, 8);
    for(unsigned int i=0; i<avPVS[l].size(); i++)
      vIterationSet.push_back(avPVS[l][i]);  // Again, plonk all searched points onto the (maybe already populate) vIterationSet.
  }

  // All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
  vNextToSearch.clear();
  for(int l=LEVELS - 2; l>=0; l--) {
    for(unsigned int i=0; i<avPVS[l].size(); i++) {
      vNextToSearch.push_back(avPVS[l][i]);
    }
  }

  // But we haven't got CPU to track _all_ patches in the map - arbitrarily limit
  // ourselves to 1000, and choose these randomly.
  static gvar3<int> gvnMaxPatchesPerFrame("Tracker.MaxPatchesPerFrame", 1000, SILENT);

  int nFinePatchesToUse = *gvnMaxPatchesPerFrame - static_cast<int>(vIterationSet.size());
  if((int) vNextToSearch.size() > nFinePatchesToUse)
  {
    random_shuffle(vNextToSearch.begin(), vNextToSearch.end());
    vNextToSearch.resize(nFinePatchesToUse); // Chop!
  }

  // If we did a coarse tracking stage: re-project and find derivs of fine points
  if(mbDidCoarse) {
    for(unsigned int i=0; i<vNextToSearch.size(); i++) {
      vNextToSearch[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
    }
  }

  // Find fine points in image:
  SearchForPoints(vNextToSearch, nFineRange, 0);
  // And attach them all to the end of the optimisation-set.
  for(unsigned int i=0; i<vNextToSearch.size(); i++) {
    vIterationSet.push_back(vNextToSearch[i]);
  }

  // Again, ten gauss-newton pose update iterations.
  Vector<6> v6LastUpdate;
  v6LastUpdate = Zeros;
  for(int iter = 0; iter<10; iter++)
  {
    bool bNonLinearIteration; // For a bit of time-saving: don't do full nonlinear
                              // reprojection at every iteration - it really isn't necessary!
    if(iter == 0 || iter == 4 || iter == 9)
      bNonLinearIteration = true;   // Even this is probably overkill, the reason we do many
    else                            // iterations is for M-Estimator convergence rather than
      bNonLinearIteration = false;  // linearisation effects.

    if(iter != 0)   // Either way: first iteration doesn't need projection update.
    {
      if(bNonLinearIteration)
      {
        for(unsigned int i=0; i<vIterationSet.size(); i++) {
          if(vIterationSet[i]->bFound)
            vIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
        }
      }
      else
      {
        for(unsigned int i=0; i<vIterationSet.size(); i++) {
          if(vIterationSet[i]->bFound)
            vIterationSet[i]->LinearUpdate(v6LastUpdate);
        }
      }
    }

    if(bNonLinearIteration) {
      for(unsigned int i=0; i<vIterationSet.size(); i++) {
        if(vIterationSet[i]->bFound)
          vIterationSet[i]->CalcJacobian();
      }
    }

    // Again, an M-Estimator hack beyond the fifth iteration.
    double dOverrideSigma = 0.0;
    if(iter > 5)
      dOverrideSigma = 16.0;

    // Calculate and update pose; also store update vector for linear iteration updates.
    Vector<6> v6Update = CalcPoseUpdate(vIterationSet, dOverrideSigma, iter==9);
    mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
    v6LastUpdate = v6Update;
  }

  if(mbDraw) {
    glPointSize(4);
    glEnable(GL_BLEND);
    glEnable(GL_POINT_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBegin(GL_POINTS);
    for(vector<TrackerData*>::reverse_iterator it = vIterationSet.rbegin();
        it!= vIterationSet.rend(); it++)
    {
      if(! (*it)->bFound)
        continue;
      glColor(gavLevelColors[(*it)->nSearchLevel]);
      glVertex((*it)->v2Image);
    }
    glEnd();
    glDisable(GL_BLEND);
  }

  // Update the current keyframe with info on what was found in the frame.
  // Strictly speaking this is unnecessary to do every frame, it'll only be
  // needed if the KF gets added to MapMaker. Do it anyway.
  // Export pose to current keyframe:
  mCurrentKF.se3CfromW = mse3CamFromWorld;
#ifdef _RECORD
  poolAllFrame.push_back(frameIndex++, mse3CamFromWorld,) //@hack by camaparijet for output all matrix
#endif

  // Record successful measurements. Use the KeyFrame-Measurement struct for this.
  mCurrentKF.mMeasurements.clear();
  for(vector<TrackerData*>::iterator it = vIterationSet.begin();
      it!= vIterationSet.end();
      it++)
  {
    if(!(*it)->bFound)
      continue;
    Measurement m;
    m.v2RootPos = (*it)->v2Found;
    m.v2ImplanePos = mCamera.UnProject(m.v2RootPos);
    m.m2CamDerivs = mCamera.GetProjectionDerivs();
    m.nLevel = (*it)->nSearchLevel;
    m.bSubPix = (*it)->bDidSubPix;
    mCurrentKF.mMeasurements[& ((*it)->Point)] = m;
  }

  // Finally, find the mean scene depth from tracked features
  {
    double dSum = 0;
    double dSumSq = 0;
    int nNum = 0;

    for(vector<TrackerData*>::iterator it = vIterationSet.begin();
        it!= vIterationSet.end(); it++)
    {
      if((*it)->bFound)
      {
        double z = (*it)->v3Cam[2];
        dSum+= z;
        dSumSq+= z*z;
        nNum++;
      }
    }

    if(nNum > 20)
    {
      mCurrentKF.dSceneDepthMean = dSum/nNum;
      mCurrentKF.dSceneDepthSigma = sqrt((dSumSq / nNum) - (mCurrentKF.dSceneDepthMean) * (mCurrentKF.dSceneDepthMean));
    }
  }
}

/**
 * Find points in the image. Uses the PatchFiner struct stored in TrackerData
 * @param vTD tracker data
 * @param nRange search range
 * @param nSubPixIts number of sub-pixel iterations required
 * @return number of points found
 */
int Tracker::SearchForPoints(vector<TrackerData*> &vTD, int nRange, int nSubPixIts)
{
  int nFound = 0;
  for(unsigned int i=0; i<vTD.size(); i++)   // for each point..
  {
    // First, attempt a search at pixel locations which are FAST corners.
    // (PatchFinder::FindPatchCoarse)
    TrackerData &TD = *vTD[i];
    PatchFinder &Finder = TD.Finder;
    Finder.MakeTemplateCoarseCont(TD.Point);
    if(Finder.TemplateBad()) {
      TD.bInImage = TD.bPotentiallyVisible = TD.bFound = false;
      continue;
    }

    manMeasAttempted[Finder.GetLevel()]++;  // Stats for tracking quality assessmenta

    bool bFound = Finder.FindPatchCoarse(ir(TD.v2Image), mCurrentKF, nRange);
    TD.bSearched = true;
    if(!bFound)
    {
      TD.bFound = false;
      continue;
    }

    TD.bFound = true;
    TD.dSqrtInvNoise = (1.0 / Finder.GetLevelScale());

    nFound++;
    manMeasFound[Finder.GetLevel()]++;

    // Found the patch in coarse search - are Sub-pixel iterations wanted too?
    if(nSubPixIts > 0)
    {
      TD.bDidSubPix = true;
      Finder.MakeSubPixTemplate();
      bool bSubPixConverges=Finder.IterateSubPixToConvergence(mCurrentKF, nSubPixIts);
      if(!bSubPixConverges)
      { // If subpix doesn't converge, the patch location is probably very dubious!
        TD.bFound = false;
        nFound--;
        manMeasFound[Finder.GetLevel()]--;
        continue;
      }
      TD.v2Found = Finder.GetSubPixPos();
    }
    else
    {
      TD.v2Found = Finder.GetCoarsePosAsVector();
      TD.bDidSubPix = false;
    }
  }
  return nFound;
};


/**
 * Calculate a pose update 6-vector from a bunch of image measurements.
 * User-selectable M-Estimator.
 * Normally this robustly estimates a sigma-squared for all the measurements
 * to reduce outlier influence, but this can be overridden if
 * dOverrideSigma is positive. Also, bMarkOutliers set to true
 * records any instances of a point being marked an outlier measurement
 * by the Tukey MEstimator.
 * @param vTD tracker data
 * @param dOverrideSigma
 * @param bMarkOutliers
 * @return
 */
Vector<6> Tracker::CalcPoseUpdate(vector<TrackerData*>& vTD, double dOverrideSigma, bool bMarkOutliers)
{
  // Which M-estimator are we using?
  int nEstimator = 0;
  static gvar3<string> gvsEstimator("TrackerMEstimator", "Tukey", SILENT);
  if(*gvsEstimator == "Tukey") {
    nEstimator = 0;
  } else if(*gvsEstimator == "Cauchy") {
    nEstimator = 1;
  } else if(*gvsEstimator == "Huber") {
    nEstimator = 2;
  } else {
    cout << "Invalid TrackerMEstimator, choices are Tukey, Cauchy, Huber" << endl;
    nEstimator = 0;
    *gvsEstimator = "Tukey";
  };

  // Find the covariance-scaled reprojection error for each measurement.
  // Also, store the square of these quantities for M-Estimator sigma squared estimation.
  vector<double> vdErrorSquared;
  for(unsigned int f=0; f<vTD.size(); f++)
  {
    TrackerData &TD = *vTD[f];
    if(!TD.bFound)
      continue;
    TD.v2Error_CovScaled = TD.dSqrtInvNoise* (TD.v2Found - TD.v2Image);
    vdErrorSquared.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
  };

  // No valid measurements? Return null update.
  if(vdErrorSquared.size() == 0)
    return makeVector( 0,0,0,0,0,0);

  // What is the distribution of errors?
  double dSigmaSquared;
  if(dOverrideSigma > 0) {
    dSigmaSquared = dOverrideSigma; // Bit of a waste having stored the vector of square errors in this case!
  } else {
    if (nEstimator == 0)
      dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
    else if(nEstimator == 1)
      dSigmaSquared = Cauchy::FindSigmaSquared(vdErrorSquared);
    else
      dSigmaSquared = Huber::FindSigmaSquared(vdErrorSquared);
  }

  // The TooN WLSCholesky class handles reweighted least squares.
  // It just needs errors and jacobians.
  WLS<6> wls;
  wls.add_prior(100.0); // Stabilising prior
  for(unsigned int f=0; f<vTD.size(); f++)
  {
    TrackerData &TD = *vTD[f];
    if(!TD.bFound)
      continue;
    Vector<2> &v2 = TD.v2Error_CovScaled;
    double dErrorSq = v2 * v2;
    double dWeight;

    if(nEstimator == 0)
      dWeight= Tukey::Weight(dErrorSq, dSigmaSquared);
    else if(nEstimator == 1)
      dWeight= Cauchy::Weight(dErrorSq, dSigmaSquared);
    else
      dWeight= Huber::Weight(dErrorSq, dSigmaSquared);

    // Inlier/outlier accounting, only really works for cut-off estimators such as Tukey.
    if(dWeight == 0.0)
    {
      if(bMarkOutliers)
        TD.Point.nMEstimatorOutlierCount++;
      continue;
    }
    else
    {
      if(bMarkOutliers)
        TD.Point.nMEstimatorInlierCount++;

      Matrix<2,6> &m26Jac = TD.m26Jacobian;
      wls.add_mJ(v2[0], TD.dSqrtInvNoise * m26Jac[0], dWeight); // These two lines are currently
      wls.add_mJ(v2[1], TD.dSqrtInvNoise * m26Jac[1], dWeight); // the slowest bit of poseits
    }
  }

  wls.compute();
  return wls.get_mu();
}

/**
 * Just add the current velocity to the current pose.
 * N.b. this doesn't actually use time in any way, i.e. it assumes
 * a one-frame-per-second camera. Skipped frames etc
 * are not handled properly here.
 */
void Tracker::ApplyMotionModel()
{
  mse3StartPos = mse3CamFromWorld;
  Vector<6> v6Velocity = mv6CameraVelocity;
  if(mbUseSBIInit)
  {
    v6Velocity.slice<3,3>() = mv6SBIRot.slice<3,3>();
    v6Velocity[0] = 0.0;
    v6Velocity[1] = 0.0;
  }
  mse3CamFromWorld = SE3<>::exp(v6Velocity) * mse3StartPos;
};

/**
 * The motion model is entirely the tracker's, and is kept as a decaying
 * constant velocity model.
 */
void Tracker::UpdateMotionModel()
{
  SE3<> se3NewFromOld = mse3CamFromWorld * mse3StartPos.inverse();
  Vector<6> v6Motion = SE3<>::ln(se3NewFromOld);
  Vector<6> v6OldVel = mv6CameraVelocity;

  mv6CameraVelocity = 0.9 * (0.5 * v6Motion + 0.5* v6OldVel);
  mdVelocityMagnitude = sqrt(mv6CameraVelocity * mv6CameraVelocity);

  // Also make an estimate of this which has been scaled by the mean scene depth.
  // This is used to decide if we should use a coarse tracking stage.
  // We can tolerate more translational vel when far away from scene!
  Vector<6> v6 = mv6CameraVelocity;
  v6.slice<0,3>() *= 1.0 / mCurrentKF.dSceneDepthMean;
  mdMSDScaledVelocityMagnitude = sqrt(v6*v6);
}


/**
 * Time to add a new keyframe? The MapMaker handles most of this.
 */
void Tracker::AddNewKeyFrame()
{
  mMapMaker.AddKeyFrame(mCurrentKF);
  mnLastKeyFrameDropped = mnFrame;
}

//
/**
 * Some heuristics to decide if tracking is any good, for this frame.
 * This influences decisions to add key-frames, and eventually
 * causes the tracker to attempt relocalisation.
 */
void Tracker::AssessTrackingQuality()
{
  int nTotalAttempted = 0;
  int nTotalFound = 0;
  int nLargeAttempted = 0;
  int nLargeFound = 0;

  for(int i=0; i<LEVELS; i++)
  {
    nTotalAttempted += manMeasAttempted[i];
    nTotalFound += manMeasFound[i];
    if(i>=2) nLargeAttempted += manMeasAttempted[i];
    if(i>=2) nLargeFound += manMeasFound[i];
  }

  if(nTotalFound == 0 || nTotalAttempted == 0)
  {
    mTrackingQuality = BAD;
  }
  else
  {
    double dTotalFracFound = (double) nTotalFound / nTotalAttempted;
    double dLargeFracFound;
    if(nLargeAttempted > 10)
      dLargeFracFound = (double) nLargeFound / nLargeAttempted;
    else
      dLargeFracFound = dTotalFracFound;

    // Original
    //static gvar3<double> gvdQualityGood("Tracker.TrackingQualityGood", 0.3, SILENT);
    //static gvar3<double> gvdQualityLost("Tracker.TrackingQualityLost", 0.13, SILENT);

    // I think camparijet changed the default values -- dhenell
    static gvar3<double> gvdQualityGood("Tracker.TrackingQualityGood", 0.003, SILENT);
    static gvar3<double> gvdQualityLost("Tracker.TrackingQualityLost", 0.001, SILENT);

    /**
     *@hack for adding key frame condition*/
    if(dTotalFracFound > *gvdQualityGood)
      mTrackingQuality = GOOD;
    else if(dLargeFracFound < *gvdQualityLost)
      mTrackingQuality = BAD;
    else
      mTrackingQuality = DODGY;
    /*if(nTotalFound > 30)
      mTrackingQuality = GOOD;
    else
      mTrackingQuality = BAD;*/
    //@hack endof evaluation
  }

  if(mTrackingQuality == DODGY)
  {
    // Further heuristics to see if it's actually bad, not just dodgy...
    // If the camera pose estimate has run miles away, it's probably bad.
    if(mMapMaker.IsDistanceToNearestKeyFrameExcessive(mCurrentKF))
      mTrackingQuality = BAD;
  }

  if(mTrackingQuality==BAD)
    mnLostFrames++;
  else
    mnLostFrames = 0;
}


/**
 * Return the user infor message
 * @return message string
 */
string Tracker::GetMessageForUser() const
{
  return mMessageForUser.str();
}


/**
 * Calculate the rotation of the small blurry image descriptor
 */
void Tracker::CalcSBIRotation()
{
  mpSBILastFrame->MakeJacs();
  pair<SE2<>, double> result_pair;
  result_pair = mpSBIThisFrame->IteratePosRelToTarget(*mpSBILastFrame, 6);
  SE3<> se3Adjust = SmallBlurryImage::SE3fromSE2(result_pair.first, mCamera);
  mv6SBIRot = se3Adjust.ln();
}


/**
 * Switch the map to the specified map.
 * @param map the map to switch to
 */
bool Tracker::SwitchMap(Map *map)
{
  if( map == NULL ) {
    return false;
  }

  if( mpMap == map)  {
    return true;
  }

  //set variables
  SE3<> se3Best = mRelocaliser.BestPose();
  mse3CamFromWorld = mse3StartPos = se3Best;
  mv6CameraVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = true;

  //set new map
  mpMap = map;

  return true;
}


/**
 * Re initialize the map.
 */
void Tracker::SetNewMap(Map * map)
{
  if( mpMap == map)
  {
    cerr << "*** WARNING Tracker::SetNewMap() map is the same. Aborting ***" << endl;
    return;
  }

  ResetCommon();

  mpMap = map;
}




ImageRef TrackerData::irImageSize;  // Static member of TrackerData lives here


}
