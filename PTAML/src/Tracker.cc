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
#include "PerformanceMonitor.h"

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

using namespace CVD;
using namespace std;
using namespace GVars3;

namespace PTAMM {

/**
 * The constructor mostly sets up interal reference variables to the other classes..
 * @param irVideoSize video image size
 * @param c camera model
 * @param maps list of maps
 * @param m current map
 * @param mm map maker
 */
Tracker::Tracker(const ImageRef &irVideoSize, const ATANCamera &c, Map *m,
                 MapMaker *mm, Relocaliser *pRelocaliser, PerformanceMonitor *pPerfMon)
  : mpMap(m)
  , mpMapMaker(mm)
  , mCamera(c)
  , mpRelocaliser(pRelocaliser)
  , mpPerfMon(pPerfMon)
  , mirSize(irVideoSize)
{
  TrackerData::irImageSize = mirSize;

  mpSBILastFrame = NULL;
  mpSBIThisFrame = NULL;

  ResetCommon();
}

/**
 * Common reset code for Reset() and ResetAll()
 */
void Tracker::ResetCommon()
{
  mbDidCoarse = false;
  mTrackingQuality = GOOD;
  mnLostFrames = 0;
  mdMSDScaledVelocityMagnitude = 0;
  mCamera.SetImageSize(mirSize);
  mnLastKeyFrameDropped = -20;
  mnFrame=0;
  mv6CameraVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = false;
}

/**
 * Resets the tracker, wipes the map.
 * This is the main Reset-handler-entry-point of the program!
 * Other classes' resets propagate from here.
 * It's always called in the Tracker's thread, often as a GUI command.
 */
void Tracker::Reset()
{
  if (mpMap->bEditLocked) {
    cerr << "MAP LOCKED: Cannot reset map " << mpMap->MapID() << "." << endl;
    return;
  }

  ResetCommon();
  mpMapMaker->Reset();
}

bool Tracker::HasGoodCoverage()
{
  Matrix<4,4,int> bins = Zeros;
  double dHori = 4.0 / (mirSize.x + 1);
  double dVert = 4.0 / (mirSize.y + 1);

  for (auto it = mvIterationSet.begin(); it != mvIterationSet.end(); ++it) {
    const Vector<2>& v2Point = (*it)->v2Image;

    int row = v2Point[1] * dVert;
    row = max(0, min(3, row));

    int col = v2Point[0] * dHori;
    col = max(0, min(3, row));

    ++bins[row][col];
  }

  int nEmpty = 0;

  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 4; ++j) {
      // Don't count inner cells
      if ((i != 1 && i != 2) || (j != 1 && j != 2)) {
        if (bins[i][j] < 5) {
          nEmpty++;
        }
      }
    }
  }

  return nEmpty < 3;
}

double gDist = 0;

bool Tracker::ShouldAddNewKeyFrame()
{
  if (mTrackingQuality != GOOD || mpMap->QueueSize() > 0 ||
      mnFrame - mnLastKeyFrameDropped <= 5)
  {
    return false;
  }

  double dDist = DistanceToClosestKeyFrame();

  gDist = dDist;

  static gvar3<int> gvdMaxKFDistWiggleMult("MapMaker.MaxKFDistWiggleMult", 2.00, SILENT);

  bool bFarAwayFromOldKeyFrames = dDist > *gvdMaxKFDistWiggleMult * mpMap->GetWiggleScaleDepthNormalized();

  return  bFarAwayFromOldKeyFrames || (!HasGoodCoverage() && dDist > 0.08);
}

void Tracker::UpdateStatsMessage()
{
  mMessageForUser << "Tracking Map, quality ";
  if(mTrackingQuality == GOOD)  mMessageForUser << "good.";
  if(mTrackingQuality == DODGY) mMessageForUser << "poor.";
  if(mTrackingQuality == BAD)   mMessageForUser << "bad.";

  mMessageForUser << " Found:";
  for(int i=0; i<LEVELS; i++) {
    mMessageForUser << " " << manMeasFound[i] << "/" << manMeasAttempted[i];
  }

  mMessageForUser << " Map " << mpMap->MapID() << ": "
                  << mpMap->GetMapPoints().size() << "P, " << mpMap->GetKeyFrames().size() << "KF " << gDist;
}

// TrackFrame is called by System.cc with each incoming video frame.
// It figures out what state the tracker is in, and calls appropriate internal tracking
// functions. bDraw tells the tracker wether it should output any GL graphics
// or not (it should not draw, for example, when AR stuff is being shown.)
void Tracker::ProcessFrame(KeyFrame &keyFrame, bool bRunTracker)
{
  mMessageForUser.str("");   // Wipe the user message clean

  mpCurrentKF = &keyFrame;

  // Update the small images for the rotation estimator
  static gvar3<double> gvdSBIBlur("Tracker.RotationEstimatorBlur", 0.75, SILENT);
  static gvar3<int> gvnUseSBI("Tracker.UseRotationEstimator", 1, SILENT);
  mbUseSBIInit = *gvnUseSBI;

  mpPerfMon->StartTimer("sbi_init");

  if(!mpSBIThisFrame) {
    mpSBIThisFrame = new SmallBlurryImage(*mpCurrentKF, *gvdSBIBlur);
    mpSBILastFrame = new SmallBlurryImage(*mpCurrentKF, *gvdSBIBlur);
  } else {
    delete mpSBILastFrame;
    mpSBILastFrame = mpSBIThisFrame;
    mpSBIThisFrame = new SmallBlurryImage(*mpCurrentKF, *gvdSBIBlur);
  }

  mpPerfMon->StopTimer("sbi_init");

  // From now on we only use the keyframe struct!
  mnFrame++;

  if (bRunTracker) {

    // Decide what to do - if there is a map, try to track the map ...
    if (!IsLost()) { // .. but only if we're not lost!

      mpPerfMon->StartTimer("sbi");
      if(mbUseSBIInit) {
        CalcSBIRotation();
      }
      mpPerfMon->StopTimer("sbi");

      ApplyMotionModel();       // 1.

      mpPerfMon->StartTimer("track");
      TrackMap();               // 2. These three lines do the main tracking work.
      mpPerfMon->StopTimer("track");

      UpdateMotionModel();      // 3.

      AssessTrackingQuality();  //  Check if we're lost or if tracking is poor.

      // Provide some feedback for the user:
      UpdateStatsMessage();

      // Heuristics to check if a key-frame should be added to the map:
      if (ShouldAddNewKeyFrame()) {
        mMessageForUser << " Adding key-frame.";
        AddNewKeyFrame();
      }

    } else { // Tracking is lost

      // cout << "Lost tracking..." << endl;
      mMessageForUser << "** Attempting recovery **.";

      if (AttemptRecovery()) {
        TrackMap();
        AssessTrackingQuality();
      }
    }
  }
}

void Tracker::GetDrawData(TrackerDrawData &drawData)
{
  drawData.bDidCoarse = mbDidCoarse;
  drawData.se3CamFromWorld = mse3CamFromWorld;
  mpCurrentKF->aLevels[0].GetAllFeatures(drawData.vCorners);

  drawData.vMapPoints.clear();

  for (auto it = mvIterationSet.begin(); it != mvIterationSet.end(); ++it) {
    drawData.vMapPoints.emplace_back((*it)->nSearchLevel, (*it)->v2Image);
  }
}

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
  //cout << "AttemptRecovery..." << endl;

  bool bRelocGood = mpRelocaliser->AttemptRecovery(*mpMap, *mpCurrentKF);
  if(!bRelocGood) {
    return false;
  }

  SE3<> se3Best = mpRelocaliser->BestPose();
  mse3CamFromWorld = mse3StartPos = se3Best;
  mv6CameraVelocity = Zeros;
  mbJustRecoveredSoUseCoarse = true;

  return true;
}

void Tracker::FindPVS(vector<TrackerData*> avPVS[])
{
  for (int i = 0; i < LEVELS; ++i) {
    avPVS[i].reserve(500);
  }

  // For all points in the map..
  const std::vector<MapPoint*>& mapPts = mpMap->GetMapPoints();

  ATANCamera camera = mCamera;
//  #pragma omp parallel for firstprivate(camera)
  for (size_t i=0; i < mapPts.size(); ++i) {
    MapPoint &p = *mapPts[i];
    // Ensure that this map point has an associated TrackerData struct.
    if(!p.pTData) {
      p.pTData = new TrackerData(&p);
    }

    TrackerData &TData = *p.pTData;

    // Project according to current view, and if it's not in the image, skip.
    TData.Project(mse3CamFromWorld, camera);
    if(!TData.bInImage) {
      continue;
    }

    // Calculate camera projection derivatives of this point.
    TData.GetDerivsUnsafe(camera);

    // And check what the PatchFinder (included in TrackerData) makes of the mappoint in this view..
    TData.nSearchLevel = TData.Finder.CalcSearchLevelAndWarpMatrix(TData.Point, mse3CamFromWorld, TData.m2CamDerivs);
    if(TData.nSearchLevel == -1) {
      continue;   // a negative search pyramid level indicates an inappropriate warp for this view, so skip.
    }

    // Otherwise, this point is suitable to be searched in the current image! Add to the PVS.
    TData.bSearched = false;
    TData.bFound = false;

    avPVS[TData.nSearchLevel].push_back(&TData);
  }

  // Next: A large degree of faffing about and deciding which points are going to be measured!
  // First, randomly shuffle the individual levels of the PVS.
  for (int i = 0; i < LEVELS; i++) {
    random_shuffle(avPVS[i].begin(), avPVS[i].end());
  }
}

void Tracker::TrackCoarse(vector<TrackerData*> avPVS[])
{
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
    vector<TrackerData*> vNextToSearch;

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

    for (auto it = vNextToSearch.begin(); it != vNextToSearch.end(); ++it) {
      if ((*it)->bFound) {
        mvIterationSet.push_back(*it);  // Copy over into the to-be-optimised list.
      }
    }

    if(nFound >= *gvnCoarseMin)  // Were enough found to do any meaningful optimisation?
    {
      mbDidCoarse = true;
      for(int iter = 0; iter < 10; ++iter) // If so: do ten Gauss-Newton pose updates iterations.
      {
        if (iter != 0) { // Re-project the points on all but the first iteration.
          for (size_t i = 0; i < mvIterationSet.size(); ++i) {
            mvIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
          }
        }
        for (size_t i=0; i < mvIterationSet.size(); ++i) {
          mvIterationSet[i]->CalcJacobian();
        }
        double dOverrideSigma = 0.0;
        // Hack: force the MEstimator to be pretty brutal
        // with outliers beyond the fifth iteration.
        if(iter > 5) {
          dOverrideSigma = 1.0;
        }

        // Calculate and apply the pose update...
        Vector<6> v6Update = CalcPoseUpdate(mvIterationSet, dOverrideSigma);
        mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
      }
    }
  }
}

void Tracker::TrackFine(vector<TrackerData*> avPVS[])
{
  // Pixel search range for the fine stage.
  // Can use a tighter search if the coarse stage was already done.
  int nFineRange = mbDidCoarse ? 5 : 10;

  // What patches shall we use this time? The high-level ones are quite important,
  // so do all of these, with sub-pixel refinement.
  {
    int l = LEVELS - 1;

    if (mbDidCoarse) {
      for (size_t i = 0; i < avPVS[l].size(); ++i) {
        avPVS[l][i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
      }
    }

    SearchForPoints(avPVS[l], nFineRange, 8);

    for (size_t i = 0; i < avPVS[l].size(); ++i) {
      if (avPVS[l][i]->bFound) {
        mvIterationSet.push_back(avPVS[l][i]);  // Again, plonk all searched points onto the (maybe already populate) vIterationSet.
      }
    }
  }

  // All the others levels: Initially, put all remaining potentially visible patches onto vNextToSearch.
  vector<TrackerData*> vNextToSearch;
  for (int l = LEVELS - 2; l >= 0; --l) {
    for (size_t i = 0; i < avPVS[l].size(); ++i) {
      vNextToSearch.push_back(avPVS[l][i]);
    }
  }

  // But we haven't got CPU to track _all_ patches in the map - arbitrarily limit
  // ourselves to 1000, and choose these randomly.
  static gvar3<int> gvnMaxPatchesPerFrame("Tracker.MaxPatchesPerFrame", 1000, SILENT);

  int nFinePatchesToUse = *gvnMaxPatchesPerFrame - static_cast<int>(mvIterationSet.size());
  if((int) vNextToSearch.size() > nFinePatchesToUse)
  {
    //random_shuffle(vNextToSearch.begin(), vNextToSearch.end());
    vNextToSearch.resize(nFinePatchesToUse); // Chop!
  }

  // If we did a coarse tracking stage: re-project and find derivs of fine points
  if(mbDidCoarse) {
    for (size_t i = 0; i < vNextToSearch.size(); ++i) {
      vNextToSearch[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
    }
  }

  // Find fine points in image:
  SearchForPoints(vNextToSearch, nFineRange, 0);

  // And attach them all to the end of the optimisation-set.
  for (size_t i = 0; i < vNextToSearch.size(); ++i) {\
    if (vNextToSearch[i]->bFound) {
      mvIterationSet.push_back(vNextToSearch[i]);
    }
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
        for(unsigned int i=0; i<mvIterationSet.size(); i++) {
          mvIterationSet[i]->ProjectAndDerivs(mse3CamFromWorld, mCamera);
        }
      }
      else
      {
        for(unsigned int i=0; i<mvIterationSet.size(); i++) {
          mvIterationSet[i]->LinearUpdate(v6LastUpdate);
        }
      }
    }

    if(bNonLinearIteration) {
      for(unsigned int i=0; i<mvIterationSet.size(); i++) {
        mvIterationSet[i]->CalcJacobian();
      }
    }

    // Again, an M-Estimator hack beyond the fifth iteration.
    double dOverrideSigma = 0.0;
    if(iter > 5) {
      dOverrideSigma = 16.0;
    }

    // Calculate and update pose; also store update vector for linear iteration updates.
    Vector<6> v6Update = CalcPoseUpdate(mvIterationSet, dOverrideSigma, iter==9);
    mse3CamFromWorld = SE3<>::exp(v6Update) * mse3CamFromWorld;
    v6LastUpdate = v6Update;
  }
}

void Tracker::UpdateCurrentKeyframeWithNewTrackingData()
{
  // Update the current keyframe with info on what was found in the frame.
  // Strictly speaking this is unnecessary to do every frame, it'll only be
  // needed if the KF gets added to MapMaker. Do it anyway.
  // Export pose to current keyframe:
  mpCurrentKF->se3CfromW = mse3CamFromWorld;
  // Record successful measurements. Use the KeyFrame-Measurement struct for this.
  mpCurrentKF->mMeasurements.clear();

  // Variables used for calculation a new mean depth
  double dSum = 0;
  double dSumSq = 0;
  int nNum = 0;

  for (auto it = mvIterationSet.begin(); it != mvIterationSet.end(); ++it) {
    // Add new measurement
    Measurement m;
    m.v2RootPos = (*it)->v2Found;
    m.v2ImplanePos = mCamera.UnProject(m.v2RootPos);
    m.m2CamDerivs = mCamera.GetProjectionDerivs();
    m.nLevel = (*it)->nSearchLevel;
    m.bSubPix = (*it)->bDidSubPix;
    mpCurrentKF->mMeasurements[& ((*it)->Point)] = m;

    // Calc mean depth
    double z = (*it)->v3Cam[2];
    dSum += z;
    dSumSq += z*z;
    nNum++;
  }

  if(nNum > 20) {
    mpCurrentKF->dSceneDepthMean = dSum / nNum;
    mpCurrentKF->dSceneDepthSigma = sqrt((dSumSq / nNum) - mpCurrentKF->dSceneDepthMean * mpCurrentKF->dSceneDepthMean);
  }
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
  for (int i = 0; i < LEVELS; ++i) {
    manMeasAttempted[i] = manMeasFound[i] = 0;
  }

  // Clear out old iterations set
  mvIterationSet.clear();

  // The Potentially-Visible-Set (PVS) is split into pyramid levels.
  vector<TrackerData*> avPVS[LEVELS];

  mpPerfMon->StartTimer("pvs");
  FindPVS(avPVS);
  mpPerfMon->StopTimer("pvs");

  // Start with a coarse tracking stage using features in the higher pyramid levels
  mpPerfMon->StartTimer("track_coarse");
  TrackCoarse(avPVS);
  mpPerfMon->StopTimer("track_coarse");

  // So, at this stage, we may or may not have done a coarse tracking stage.
  // Now do the fine tracking stage. This needs many more points!
  mpPerfMon->StartTimer("track_fine");
  TrackFine(avPVS);
  mpPerfMon->StopTimer("track_fine");

  UpdateCurrentKeyframeWithNewTrackingData();
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

  //#pragma omp parallel for schedule(dynamic,5)
  for (size_t i = 0; i < vTD.size(); ++i) {  // for each point..
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

    bool bFound = Finder.FindPatchCoarse(ir(TD.v2Image), *mpCurrentKF, nRange);
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
      bool bSubPixConverges=Finder.IterateSubPixToConvergence(*mpCurrentKF, nSubPixIts);
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
}


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
  for(unsigned int f=0; f<vTD.size(); f++) {
    TrackerData &TD = *vTD[f];
    TD.v2Error_CovScaled = TD.dSqrtInvNoise* (TD.v2Found - TD.v2Image);
    vdErrorSquared.push_back(TD.v2Error_CovScaled * TD.v2Error_CovScaled);
  }

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
}

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
  v6.slice<0,3>() *= 1.0 / mpCurrentKF->dSceneDepthMean;
  mdMSDScaledVelocityMagnitude = sqrt(v6*v6);
}

/**
 * Time to add a new keyframe? The MapMaker handles most of this.
 */
void Tracker::AddNewKeyFrame()
{
  mpMapMaker->AddKeyFrame(*mpCurrentKF);
  mnLastKeyFrameDropped = mnFrame;
}

double Tracker::DistanceToClosestKeyFrame()
{
  KeyFrame *pClosest = mpMap->ClosestKeyFrame(*mpCurrentKF);

  if (pClosest == NULL) {
    return std::numeric_limits<double>::max();
  }

  double dDist = mpMap->KeyFrameLinearDist(*mpCurrentKF, *pClosest);
  dDist *= (1.0 / mpCurrentKF->dSceneDepthMean);
  return dDist;
}

// Is the tracker's camera pose in cloud-cuckoo land?
bool Tracker::IsDistanceToNearestKeyFrameExcessive()
{
  return mpMap->DistToNearestKeyFrame(*mpCurrentKF) > mpMap->GetWiggleScale() * 10.0;
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

    static gvar3<double> gvdQualityGood("Tracker.TrackingQualityGood", 0.3, SILENT);
    static gvar3<double> gvdQualityLost("Tracker.TrackingQualityLost", 0.13, SILENT);

    /**
     *@hack for adding key frame condition*/
    if(dTotalFracFound > *gvdQualityGood)
      mTrackingQuality = GOOD;
    else if(dLargeFracFound < *gvdQualityLost)
      mTrackingQuality = BAD;
    else
      mTrackingQuality = DODGY;

    if (nTotalFound < 30) {
      mTrackingQuality = BAD;
    }

    //@hack endof evaluation
  }

  if(mTrackingQuality == DODGY)
  {
    // Further heuristics to see if it's actually bad, not just dodgy...
    // If the camera pose estimate has run miles away, it's probably bad.
    if(IsDistanceToNearestKeyFrameExcessive())
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


ImageRef TrackerData::irImageSize;  // Static member of TrackerData lives here


}
