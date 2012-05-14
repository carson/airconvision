// Copyright 2009 Isis Innovation Limited
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LevelHelpers.h"
#include "HomographyInit.h"
#include "PatchFinder.h"
#include "SmallBlurryImage.h"

#include <TooN/SVD.h>
#include <TooN/SymEigen.h>
#include <gvars3/instances.h>

#include <algorithm>
#include <chrono>

using namespace std;
using namespace TooN;
using namespace CVD;
using namespace GVars3;

namespace PTAMM {

BundleAdjustmentJob::BundleAdjustmentJob()
  : mbEmptyJob(true)
{
}

BundleAdjustmentJob::BundleAdjustmentJob(Map *pMap,
    const std::set<KeyFrame*>& sAdjustSet, const std::set<KeyFrame*>& sFixedSet,
    const std::set<MapPoint*>& sMapPoints, bool bRecent)
  : mpMap(pMap)
  , mbEmptyJob(false)
  , mbRecent(bRecent)
  , mbBundleAbortRequested(false)
{
  // Add the keyframes' poses to the bundle adjuster. Two parts: first nonfixed, then fixed.
  for(set<KeyFrame*>::const_iterator it = sAdjustSet.begin(); it!= sAdjustSet.end(); ++it)
  {
    int nBundleID = mBundle.AddCamera((*it)->se3CfromW, (*it)->bFixed);
    mView_BundleID[*it] = nBundleID;
    mBundleID_View[nBundleID] = *it;
  }

  for(set<KeyFrame*>::const_iterator it = sFixedSet.begin(); it!= sFixedSet.end(); ++it)
  {
    int nBundleID = mBundle.AddCamera((*it)->se3CfromW, true);
    mView_BundleID[*it] = nBundleID;
    mBundleID_View[nBundleID] = *it;
  }

  // Add the points' 3D position
  for(set<MapPoint*>::const_iterator it = sMapPoints.begin(); it!=sMapPoints.end(); ++it)
  {
    int nBundleID = mBundle.AddPoint((*it)->v3WorldPos);
    mPoint_BundleID[*it] = nBundleID;
    mBundleID_Point[nBundleID] = *it;
  }

  // Add the relevant point-in-keyframe measurements
  for(unsigned int i=0; i< mpMap->vpKeyFrames.size(); i++)
  {
    if(mView_BundleID.count(mpMap->vpKeyFrames[i]) == 0)
      continue;

    int nKF_BundleID = mView_BundleID[mpMap->vpKeyFrames[i]];

    for(meas_it it= mpMap->vpKeyFrames[i]->mMeasurements.begin();
        it!= mpMap->vpKeyFrames[i]->mMeasurements.end();
        ++it)
    {
      if(mPoint_BundleID.count(it->first) == 0)
        continue;

      int nPoint_BundleID = mPoint_BundleID[it->first];

      mBundle.AddMeas(nKF_BundleID, nPoint_BundleID, it->second.v2ImplanePos, LevelScale(it->second.nLevel) * LevelScale(it->second.nLevel), it->second.m2CamDerivs);
    }
  }
}

bool BundleAdjustmentJob::Run()
{
  // There is nothing to do bundle adjustment on, so just succeed
  if (mbEmptyJob) {
    return true;
  }

  // Run the bundle adjuster. This returns the number of successful iterations
  int nAccepted = mBundle.Compute(&mbBundleAbortRequested);

  if(nAccepted < 0)
  {
    // Crap: - LM Ran into a serious problem!
    // This is probably because the initial stereo was messed up.
    // Get rid of this map and start again!
    cout << "!! MapMaker: Cholesky failure in bundle adjust. " << endl
         << "   The map is probably corrupt: Ditching the map. " << endl;
    //mbResetRequested = true; // Just handle this outside of this class
    return false;
  }

  // Bundle adjustment did some updates, apply these to the map
  if(nAccepted > 0)
  {
    for(map<MapPoint*,int>::iterator itr = mPoint_BundleID.begin();
        itr != mPoint_BundleID.end(); ++itr)
    {
      itr->first->v3WorldPos = mBundle.GetPoint(itr->second);
    }

    for(map<KeyFrame*,int>::iterator itr = mView_BundleID.begin();
        itr != mView_BundleID.end(); itr++)
    {
      itr->first->se3CfromW = mBundle.GetCamera(itr->second);
    }

    if(mbRecent) {
      mpMap->bBundleConverged_Recent = false;
    }

    mpMap->bBundleConverged_Full = false;
  }

  if(mBundle.Converged()) {
    mpMap->bBundleConverged_Recent = true;
    if(!mbRecent) {
      mpMap->bBundleConverged_Full = true;
    }
  }

  // Handle outlier measurements:
  vector<pair<int,int> > vOutliers_PC_pair = mBundle.GetOutlierMeasurements();
  for(size_t i=0; i<vOutliers_PC_pair.size(); ++i)
  {
    MapPoint *pp = mBundleID_Point[vOutliers_PC_pair[i].first];
    KeyFrame *pk = mBundleID_View[vOutliers_PC_pair[i].second];
    Measurement &m = pk->mMeasurements[pp];
    if(pp->pMMData->GoodMeasCount() <= 2 || m.Source == Measurement::SRC_ROOT) {  // Is the original source kf considered an outlier? That's bad.
      pp->bBad = true;
    }
    else
    {
      // Do we retry it? Depends where it came from!!
      if(m.Source == Measurement::SRC_TRACKER || m.Source == Measurement::SRC_EPIPOLAR) {
        mpMap->vFailureQueue.push_back(pair<KeyFrame*,MapPoint*>(pk,pp));
      } else {
        pp->pMMData->sNeverRetryKFs.insert(pk);
      }

      pk->mMeasurements.erase(pp);
      pp->pMMData->sMeasurementKFs.erase(pk);
    }
  }

  return true;
}


/**
 * Constructor. Calls reset and sets the map ID
 */
Map::Map()
  : N(10000),
    nTex(0)
{
  static int nMapCounter = 0;
  mnMapNum = nMapCounter++;
  Reset();

  mdWiggleScale = GV3::get<double>("MapMaker.WiggleScale", 0.1, SILENT);
}

/**
 * Destructor
 */
Map::~Map()
{
  Reset();

}

/**
 * Reset the map
 */
void Map::Reset()
{
  //clear map points
  for(size_t i = 0; i < vpPoints.size(); i++) {
    delete vpPoints.at(i);
  }
  vpPoints.clear();

  //clear trash points
  EmptyTrash();

  //clear keyframes
  for(size_t i = 0; i < vpKeyFrames.size(); i++) {
    delete vpKeyFrames.at(i);
  }
  vpKeyFrames.clear();

  bGood = false;         //no longer good
  bEditLocked = false;   //make editable
  sSaveFile = "";        //not associated with a file on disk

  //clear queued keyframes.
  for(size_t i = 0; i < vpKeyFrameQueue.size(); i++) {
    delete vpKeyFrameQueue.at(i);
  }
  vpKeyFrameQueue.clear();

  //clear the failure queue
  vFailureQueue.clear();

  //clear queued new observations.
  while(!qNewQueue.empty())
  {
//     delete qNewQueue.front();
    qNewQueue.pop_front();
  }

  bBundleConverged_Full = true;
  bBundleConverged_Recent = true;

  // mnMapNum is not reset as this is constant for a map.
}

// Finds 3d coords of point in reference frame B from two z=1 plane projections
Vector<3> Map::ReprojectPoint(const SE3<>& se3AfromB, const Vector<2> &v2A, const Vector<2> &v2B) const
{
  Matrix<3,4> PDash;
  PDash.slice<0,0,3,3>() = se3AfromB.get_rotation().get_matrix();
  PDash.slice<0,3,3,1>() = se3AfromB.get_translation().as_col();

  Matrix<4> A;
  A[0][0] = -1.0; A[0][1] =  0.0; A[0][2] = v2B[0]; A[0][3] = 0.0;
  A[1][0] =  0.0; A[1][1] = -1.0; A[1][2] = v2B[1]; A[1][3] = 0.0;
  A[2] = v2A[0] * PDash[2] - PDash[0];
  A[3] = v2A[1] * PDash[2] - PDash[1];

  SVD<4,4> svd(A);
  Vector<4> v4Smallest = svd.get_VT()[3];
  if(v4Smallest[3] == 0.0)
    v4Smallest[3] = 0.00001;
  return project(v4Smallest);
}

// InitFromStereo() generates the initial match from two keyframes
// and a vector of image correspondences. Uses the
bool Map::InitFromStereo(KeyFrame &kF,
                         KeyFrame &kS,
                         vector<pair<ImageRef, ImageRef> > &vTrailMatches,
                         SE3<> &se3TrackerPose)
{
  ATANCamera &camera = kF.Camera;

  vector<HomographyMatch> vMatches;
  for(size_t i=0; i<vTrailMatches.size(); ++i)
  {
    HomographyMatch m;
    m.v2CamPlaneFirst = camera.UnProject(vTrailMatches[i].first);
    m.v2CamPlaneSecond = camera.UnProject(vTrailMatches[i].second);
    m.m2PixelProjectionJac = camera.GetProjectionDerivs();
    vMatches.push_back(m);
  }

  SE3<> se3;
  HomographyInit homographyInit;
  if(!homographyInit.Compute(vMatches, 5.0, se3))
  {
    cout << "  Could not init from stereo pair, try again." << endl;
    return false;
  }

  // Check that the initialiser estimated a non-zero baseline
  double dTransMagn = sqrt(se3.get_translation() * se3.get_translation());
  if(dTransMagn == 0)
  {
    cout << "  Estimated zero baseline from stereo pair, try again." << endl;
    return false;
  }
  // change the scale of the map so the second camera is wiggleScale away from the first
  se3.get_translation() *= mdWiggleScale/dTransMagn;


  KeyFrame *pkFirst = new KeyFrame(kF);
  KeyFrame *pkSecond = new KeyFrame(kS);

  pkFirst->bFixed = true;
  pkFirst->se3CfromW = SE3<>();

  pkSecond->bFixed = false;
  pkSecond->se3CfromW = se3;

  // Construct map from the stereo matches.
  PatchFinder finder;

  for(size_t i=0; i<vMatches.size(); ++i)
  {
    MapPoint *p = new MapPoint();

    // Patch source stuff:
    p->pPatchSourceKF = pkFirst;
    p->nSourceLevel = 0;
    p->v3Normal_NC = makeVector( 0,0,-1);
    p->irCenter = vTrailMatches[i].first;
    p->v3Center_NC = unproject(camera.UnProject(p->irCenter));
    p->v3OneDownFromCenter_NC = unproject(camera.UnProject(p->irCenter + ImageRef(0,1)));
    p->v3OneRightFromCenter_NC = unproject(camera.UnProject(p->irCenter + ImageRef(1,0)));
    normalize(p->v3Center_NC);
    normalize(p->v3OneDownFromCenter_NC);
    normalize(p->v3OneRightFromCenter_NC);
    p->RefreshPixelVectors();

    // Do sub-pixel alignment on the second image
    finder.MakeTemplateCoarseNoWarp(*p);
    finder.MakeSubPixTemplate();
    finder.SetSubPixPos(vec(vTrailMatches[i].second));
    bool bGood = finder.IterateSubPixToConvergence(*pkSecond,10);
    if(!bGood)
    {
      delete p; continue;
    }

    // Triangulate point:
    Vector<2> v2SecondPos = finder.GetSubPixPos();
    p->v3WorldPos = ReprojectPoint(se3, camera.UnProject(v2SecondPos), vMatches[i].v2CamPlaneFirst);
    if(p->v3WorldPos[2] < 0.0)
    {
      delete p; continue;
    }

    // Not behind map? Good, then add to map.
    p->pMMData = new MapMakerData();
    vpPoints.push_back(p);

    // Construct first two measurements and insert into relevant DBs:
    Measurement mFirst;
    mFirst.nLevel = 0;
    mFirst.Source = Measurement::SRC_ROOT;
    mFirst.v2RootPos = vec(vTrailMatches[i].first);
    mFirst.v2ImplanePos = camera.UnProject(mFirst.v2RootPos);
    mFirst.m2CamDerivs = camera.GetProjectionDerivs();
    mFirst.bSubPix = true;
    pkFirst->mMeasurements[p] = mFirst;
    p->pMMData->sMeasurementKFs.insert(pkFirst);

    Measurement mSecond;
    mSecond.nLevel = 0;
    mSecond.Source = Measurement::SRC_TRAIL;
    mSecond.v2RootPos = finder.GetSubPixPos();
    mSecond.v2ImplanePos = camera.UnProject(mSecond.v2RootPos);
    mSecond.m2CamDerivs = camera.GetProjectionDerivs();
    mSecond.bSubPix = true;
    pkSecond->mMeasurements[p] = mSecond;
    p->pMMData->sMeasurementKFs.insert(pkSecond);
  }

  MakeTextureFromKF(*pkFirst);//@hack
  MakeTextureFromKF(*pkSecond);//@hack

  vpKeyFrames.push_back(pkFirst);
  vpKeyFrames.push_back(pkSecond);

  pkFirst->MakeKeyFrame_Rest();
  pkSecond->MakeKeyFrame_Rest();


  for(int i=0; i<5; i++)
    BundleAdjustAll();

  // Estimate the feature depth distribution in the first two key-frames
  // (Needed for epipolar search)
  pkFirst->RefreshSceneDepth();
  pkSecond->RefreshSceneDepth();
  mdWiggleScaleDepthNormalized = mdWiggleScale / pkFirst->dSceneDepthMean;


  AddSomeMapPoints(0);
  AddSomeMapPoints(3);
  AddSomeMapPoints(1);
  AddSomeMapPoints(2);

  bBundleConverged_Full = false;
  bBundleConverged_Recent = false;

  std::chrono::time_point<std::chrono::system_clock> startTime = std::chrono::system_clock::now();

  while(!bBundleConverged_Full)
  {
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    if (now - startTime > std::chrono::seconds(5)) {
      std::cout << "InitFromStereo timed out" << std::endl;
      return false;
    }

    auto job = BundleAdjustAll();
    if (!job.Run()) {
      return false;
    }

    /*
    if(mbResetRequested || mbReInitRequested || mbSwitchRequested)
      return false;
      */
  }

  // Rotate and translate the map so the dominant plane is at z=0:
  ApplyGlobalTransformation(CalcPlaneAligner());
  bGood = true;

  se3TrackerPose = pkSecond->se3CfromW;

  cout << "se3CfromW : " << pkFirst->se3CfromW.ln();
  cout << "  MapMaker: made initial map with " << vpPoints.size() << " points." << endl;

  return true;
}


/**
 * Tries to make a new map point out of a single candidate point
 * by searching for that point in another keyframe, and triangulating
 * if a match is found.
 */
bool Map::AddPointEpipolar(KeyFrame &kSrc,
                           KeyFrame &kTarget,
                           int nLevel,
                           int nCandidate)
{
//   static Image<Vector<2> > imUnProj;
//   static bool bMadeCache = false;
//   if(!bMadeCache)
//     {
//       imUnProj.resize(kSrc.aLevels[0].im.size());
//       ImageRef ir;
//       do imUnProj[ir] = kSrc.Camera.UnProject(ir);
//       while(ir.next(imUnProj.size()));
//       bMadeCache = true;
//     }

  int nLevelScale = LevelScale(nLevel);
  Candidate &candidate = kSrc.aLevels[nLevel].vCandidates[nCandidate];
  ImageRef irLevelPos = candidate.irLevelPos;
  Vector<2> v2RootPos = LevelZeroPos(irLevelPos, nLevel);

  Vector<3> v3Ray_SC = unproject(kSrc.Camera.UnProject(v2RootPos));
  normalize(v3Ray_SC);
  Vector<3> v3LineDirn_TC = kTarget.se3CfromW.get_rotation() * (kSrc.se3CfromW.get_rotation().inverse() * v3Ray_SC);

  // Restrict epipolar search to a relatively narrow depth range
  // to increase reliability
  double dMean = kSrc.dSceneDepthMean;
  double dSigma = kSrc.dSceneDepthSigma;
  double dStartDepth = max(mdWiggleScale, dMean - dSigma);
  double dEndDepth = min(40 * mdWiggleScale, dMean + dSigma);

  Vector<3> v3CamCenter_TC = kTarget.se3CfromW * kSrc.se3CfromW.inverse().get_translation(); // The camera end
  Vector<3> v3RayStart_TC = v3CamCenter_TC + dStartDepth * v3LineDirn_TC;                               // the far-away end
  Vector<3> v3RayEnd_TC = v3CamCenter_TC + dEndDepth * v3LineDirn_TC;                               // the far-away end


  if(v3RayEnd_TC[2] <= v3RayStart_TC[2])  // it's highly unlikely that we'll manage to get anything out if we're facing backwards wrt the other camera's view-ray
    return false;
  if(v3RayEnd_TC[2] <= 0.0 )  return false;
  if(v3RayStart_TC[2] <= 0.0)
    v3RayStart_TC += v3LineDirn_TC * (0.001 - v3RayStart_TC[2] / v3LineDirn_TC[2]);

  Vector<2> v2A = project(v3RayStart_TC);
  Vector<2> v2B = project(v3RayEnd_TC);
  Vector<2> v2AlongProjectedLine = v2A-v2B;

  if( (v2AlongProjectedLine * v2AlongProjectedLine) < 0.00000001)
  {
    cout << "v2AlongProjectedLine too small." << endl;
    return false;
  }

  normalize(v2AlongProjectedLine);
  Vector<2> v2Normal;
  v2Normal[0] = v2AlongProjectedLine[1];
  v2Normal[1] = -v2AlongProjectedLine[0];

  double dNormDist = v2A * v2Normal;
  if(fabs(dNormDist) > kTarget.Camera.LargestRadiusInImage() )
    return false;

  double dMinLen = min(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) - 0.05;
  double dMaxLen = max(v2AlongProjectedLine * v2A, v2AlongProjectedLine * v2B) + 0.05;
  if(dMinLen < -2.0)  dMinLen = -2.0;
  if(dMaxLen < -2.0)  dMaxLen = -2.0;
  if(dMinLen > 2.0)   dMinLen = 2.0;
  if(dMaxLen > 2.0)   dMaxLen = 2.0;

  // Find current-frame corners which might match this
  PatchFinder Finder;
  Finder.MakeTemplateCoarseNoWarp(kSrc, nLevel, irLevelPos);
  if(Finder.TemplateBad())  return false;

  vector<Vector<2> > &vv2Corners = kTarget.aLevels[nLevel].vImplaneCorners;
  vector<ImageRef> &vIR = kTarget.aLevels[nLevel].vCorners;
  if(!kTarget.aLevels[nLevel].bImplaneCornersCached)
  {
    for(unsigned int i=0; i<vIR.size(); i++)   // over all corners in target img..
      vv2Corners.push_back(kTarget.Camera.UnProject(ir(LevelZeroPos(vIR[i], nLevel))));
    kTarget.aLevels[nLevel].bImplaneCornersCached = true;
  }

  int nBest = -1;
  int nBestZMSSD = Finder.GetMaxSSD() + 1;
  double dMaxDistDiff = kTarget.Camera.OnePixelDist() * (4.0 + 1.0 * nLevelScale);
  double dMaxDistSq = dMaxDistDiff * dMaxDistDiff;

  for(unsigned int i=0; i<vv2Corners.size(); i++)   // over all corners in target img..
  {
    const Vector<2>& v2Im = vv2Corners[i];
    double dDistDiff = dNormDist - v2Im * v2Normal;
    if( (dDistDiff * dDistDiff) > dMaxDistSq)       continue; // skip if not along epi line
    if( (v2Im * v2AlongProjectedLine) < dMinLen)    continue; // skip if not far enough along line
    if( (v2Im * v2AlongProjectedLine) > dMaxLen)    continue; // or too far
    int nZMSSD = Finder.ZMSSDAtPoint(kTarget.aLevels[nLevel].im, vIR[i]);
    if(nZMSSD < nBestZMSSD)
    {
      nBest = i;
      nBestZMSSD = nZMSSD;
    }
  }

  if(nBest == -1)   return false;   // Nothing found.

  //  Found a likely candidate along epipolar ray
  Finder.MakeSubPixTemplate();
  Finder.SetSubPixPos(LevelZeroPos(vIR[nBest], nLevel));
  bool bSubPixConverges = Finder.IterateSubPixToConvergence(kTarget,10);
  if(!bSubPixConverges)
    return false;

  // Now triangulate the 3d point...
  Vector<3> v3New;
  v3New = kTarget.se3CfromW.inverse() *
    ReprojectPoint(kSrc.se3CfromW * kTarget.se3CfromW.inverse(),
                   kSrc.Camera.UnProject(v2RootPos),
                   kTarget.Camera.UnProject(Finder.GetSubPixPos()));

  MapPoint *pNew = new MapPoint;
  pNew->v3WorldPos = v3New;
  pNew->pMMData = new MapMakerData();

  // Patch source stuff:
  pNew->pPatchSourceKF = &kSrc;
  pNew->nSourceLevel = nLevel;
  pNew->v3Normal_NC = makeVector( 0,0,-1);
  pNew->irCenter = irLevelPos;
  pNew->v3Center_NC = unproject(kSrc.Camera.UnProject(v2RootPos));
  pNew->v3OneRightFromCenter_NC = unproject(kSrc.Camera.UnProject(v2RootPos + vec(ImageRef(nLevelScale,0))));
  pNew->v3OneDownFromCenter_NC  = unproject(kSrc.Camera.UnProject(v2RootPos + vec(ImageRef(0,nLevelScale))));

  normalize(pNew->v3Center_NC);
  normalize(pNew->v3OneDownFromCenter_NC);
  normalize(pNew->v3OneRightFromCenter_NC);

  pNew->RefreshPixelVectors();

  /**
   *@hack by camparijet
   * added color for painting
   */
  pNew->pColor = kSrc.im_cl[pNew->irCenter];
  /*@hack end*/

  vpPoints.push_back(pNew);
  qNewQueue.push_back(pNew);

  Measurement m;
  m.Source = Measurement::SRC_ROOT;
  m.v2RootPos = v2RootPos;
  m.v2ImplanePos = kSrc.Camera.UnProject(m.v2RootPos);
  m.m2CamDerivs = kSrc.Camera.GetProjectionDerivs();
  m.nLevel = nLevel;
  kSrc.mMeasurements[pNew] = m;

  m.Source = Measurement::SRC_EPIPOLAR;
  m.v2RootPos = Finder.GetSubPixPos();
  m.v2ImplanePos = kTarget.Camera.UnProject(m.v2RootPos);
  m.m2CamDerivs = kTarget.Camera.GetProjectionDerivs();
  kTarget.mMeasurements[pNew] = m;

  pNew->pMMData->sMeasurementKFs.insert(&kSrc);
  pNew->pMMData->sMeasurementKFs.insert(&kTarget);

  return true;
}


// Mapmaker's try-to-find-a-point-in-a-keyframe code. This is used to update
// data association if a bad measurement was detected, or if a point
// was never searched for in a keyframe in the first place. This operates
// much like the tracker! So most of the code looks just like in
// TrackerData.h.
bool Map::ReFind_Common(KeyFrame &k, MapPoint &p)
{
  // abort if either a measurement is already in the map, or we've
  // decided that this point-kf combo is beyond redemption
  if(p.pMMData->sMeasurementKFs.count(&k)
     || p.pMMData->sNeverRetryKFs.count(&k))
    return false;

  static PatchFinder Finder;
  Vector<3> v3Cam = k.se3CfromW*p.v3WorldPos;
  if(v3Cam[2] < 0.001)
  {
    p.pMMData->sNeverRetryKFs.insert(&k);
    return false;
  }
  Vector<2> v2ImPlane = project(v3Cam);
  if( (v2ImPlane* v2ImPlane) > (k.Camera.LargestRadiusInImage() * k.Camera.LargestRadiusInImage()) )
  {
    p.pMMData->sNeverRetryKFs.insert(&k);
    return false;
  }

  Vector<2> v2Image = k.Camera.Project(v2ImPlane);
  if(k.Camera.Invalid())
  {
    p.pMMData->sNeverRetryKFs.insert(&k);
    return false;
  }

  const ImageRef& irImageSize = k.aLevels[0].im.size();
  if(v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize[0] || v2Image[1] > irImageSize[1])
  {
    p.pMMData->sNeverRetryKFs.insert(&k);
    return false;
  }

  Matrix<2> m2CamDerivs = k.Camera.GetProjectionDerivs();
  Finder.MakeTemplateCoarse(p, k.se3CfromW, m2CamDerivs);

  if(Finder.TemplateBad())
  {
    p.pMMData->sNeverRetryKFs.insert(&k);
    return false;
  }

  bool bFound = Finder.FindPatchCoarse(ir(v2Image), k, 4);  // Very tight search radius!
  if(!bFound)
  {
    p.pMMData->sNeverRetryKFs.insert(&k);
    return false;
  }

  // If we found something, generate a measurement struct and put it in the map
  Measurement m;
  m.nLevel = Finder.GetLevel();
  m.Source = Measurement::SRC_REFIND;

  if(Finder.GetLevel() > 0)
  {
    Finder.MakeSubPixTemplate();
    Finder.IterateSubPixToConvergence(k,8);
    m.v2RootPos = Finder.GetSubPixPos();
    m.bSubPix = true;
  }
  else
  {
    m.v2RootPos = Finder.GetCoarsePosAsVector();
    m.bSubPix = false;
  }

  m.v2ImplanePos = k.Camera.UnProject(m.v2RootPos);
  m.m2CamDerivs = k.Camera.GetProjectionDerivs();

  if(k.mMeasurements.count(&p))
  {
    assert(0); // This should never happen, we checked for this at the start.
  }

  k.mMeasurements[&p] = m;
  p.pMMData->sMeasurementKFs.insert(&k);

  return true;
}

// A general data-association update for a single keyframe
// Do this on a new key-frame when it's passed in by the tracker
int Map::ReFindInSingleKeyFrame(KeyFrame &k)
{
  int nFoundNow = 0;
  for(size_t i=0; i<vpPoints.size(); ++i)
    if(ReFind_Common(k,*vpPoints[i]))
      nFoundNow++;

  return nFoundNow;
}

// When new map points are generated, they're only created from a stereo pair
// this tries to make additional measurements in other KFs which they might
// be in.
void Map::ReFindNewlyMade()
{
  if(qNewQueue.empty())
    return;

  int nFound = 0;
  int nBad = 0;
  while(!qNewQueue.empty() && vpKeyFrameQueue.empty())
    {
      MapPoint* pNew = qNewQueue.front();
      qNewQueue.pop_front();
      if(pNew->bBad)
      {
        nBad++;
        continue;
      }
      for(size_t i=0; i<vpKeyFrames.size(); ++i) {
        if(ReFind_Common(*vpKeyFrames[i], *pNew))
          nFound++;
      }
    }
}

// Dud measurements get a second chance.
void Map::ReFindFromFailureQueue()
{
  if(vFailureQueue.empty())
    return;

  sort(vFailureQueue.begin(), vFailureQueue.end());
  vector<pair<KeyFrame*, MapPoint*> >::iterator it;
  int nFound=0;
  for(it = vFailureQueue.begin(); it!=vFailureQueue.end(); ++it) {
    if(ReFind_Common(*it->first, *it->second))
      nFound++;
  }

  vFailureQueue.erase(vFailureQueue.begin(), it);
}

void Map::QueueKeyFrame(const KeyFrame &k)
{
  KeyFrame *pK = new KeyFrame(k);
  if(pK->pSBI != NULL)
  {
    delete pK->pSBI;
    pK->pSBI = NULL; // Mapmaker uses a different SBI than the tracker, so will re-gen its own
  }

  MakeTextureFromKF(*pK);//@hack
  vpKeyFrameQueue.push_back(pK);
}

/**
 * Mapmaker's code to handle incoming keyframes.
 */
void Map::AddKeyFrameFromTopOfQueue()
{
  if(vpKeyFrameQueue.empty())
    return;

  KeyFrame *pK = vpKeyFrameQueue[0];
  vpKeyFrameQueue.erase(vpKeyFrameQueue.begin());
  pK->MakeKeyFrame_Rest();
  vpKeyFrames.push_back(pK);

  // Any measurements? Update the relevant point's measurement counter status map
  for(meas_it it = pK->mMeasurements.begin();
      it!=pK->mMeasurements.end();
      ++it)
  {
    it->first->pMMData->sMeasurementKFs.insert(pK);
    it->second.Source = Measurement::SRC_TRACKER;
  }

  // And maybe we missed some - this now adds to the map itself, too.
  ReFindInSingleKeyFrame(*pK);

  //AddSomeMapPoints(3);       // .. and add more map points by epipolar search.
  AddSomeMapPoints(0);
  AddSomeMapPoints(1);
  AddSomeMapPoints(2);

  bBundleConverged_Full = false;
  bBundleConverged_Recent = false;
}

// Adds map points by epipolar search to the last-added key-frame, at a single
// specified pyramid level. Does epipolar search in the target keyframe as closest by
// the ClosestKeyFrame function.
void Map::AddSomeMapPoints(int nLevel)
{
  KeyFrame &kSrc = *vpKeyFrames[vpKeyFrames.size() - 1]; // The new keyframe
  KeyFrame &kTarget = *ClosestKeyFrame(kSrc);
  Level &l = kSrc.aLevels[nLevel];

  kSrc.ThinCandidates(nLevel);

  for(size_t i = 0; i<l.vCandidates.size(); ++i)
    AddPointEpipolar(kSrc, kTarget, nLevel, i);
};


// Perform bundle adjustment on all keyframes, all map points
BundleAdjustmentJob Map::BundleAdjustAll()
{
  // construct the sets of kfs/points to be adjusted:
  // in this case, all of them
  set<KeyFrame*> sAdj;
  set<KeyFrame*> sFixed;
  for(size_t i=0; i < vpKeyFrames.size(); ++i) {
    if(vpKeyFrames[i]->bFixed)
      sFixed.insert(vpKeyFrames[i]);
    else
      sAdj.insert(vpKeyFrames[i]);
  }

  set<MapPoint*> sMapPoints;
  for(size_t i=0; i < vpPoints.size(); ++i)
    sMapPoints.insert(vpPoints[i]);

  return BundleAdjustmentJob(this, sAdj, sFixed, sMapPoints, false);
}

// Peform a local bundle adjustment which only adjusts
// recently added key-frames
BundleAdjustmentJob Map::BundleAdjustRecent()
{
  if(vpKeyFrames.size() < 8)
  { // Ignore this unless map is big enough
    bBundleConverged_Recent = true;
    return BundleAdjustmentJob();
  }

  // First, make a list of the keyframes we want adjusted in the adjuster.
  // This will be the last keyframe inserted, and its four nearest neighbors
  set<KeyFrame*> sAdjustSet;
  KeyFrame *pkfNewest = vpKeyFrames.back();
  sAdjustSet.insert(pkfNewest);

  vector<KeyFrame*> vClosest = NClosestKeyFrames(*pkfNewest, 4); //@hack default is 4

  for(int i=0; i<4; i++) {
    if(vClosest[i]->bFixed == false)
      sAdjustSet.insert(vClosest[i]);
  }

  // Now we find the set of features which they contain.
  set<MapPoint*> sMapPoints;
  for(set<KeyFrame*>::iterator iter = sAdjustSet.begin();
      iter != sAdjustSet.end();
      ++iter)
  {
    map<MapPoint*,Measurement> &mKFMeas = (*iter)->mMeasurements;
    for(meas_it jiter = mKFMeas.begin(); jiter!= mKFMeas.end(); ++jiter)
      sMapPoints.insert(jiter->first);
  }

  // Finally, add all keyframes which measure above points as fixed keyframes
  set<KeyFrame*> sFixedSet;
  for(vector<KeyFrame*>::iterator it = vpKeyFrames.begin(); it != vpKeyFrames.end(); ++it)
  {
    if(sAdjustSet.count(*it))
      continue;

    bool bInclude = false;

    for(meas_it jiter = (*it)->mMeasurements.begin(); jiter!= (*it)->mMeasurements.end(); ++jiter) {
      if(sMapPoints.count(jiter->first)) {
        bInclude = true;
        break;
      }
    }

    if(bInclude) {
      sFixedSet.insert(*it);
    }
  }

  return BundleAdjustmentJob(this, sAdjustSet, sFixedSet, sMapPoints, true);
}

// Find a dominant plane in the map, find an SE3<> to put it as the z=0 plane
SE3<> Map::CalcPlaneAligner()
{
  unsigned int nPoints = vpPoints.size();
  if(nPoints < 10)
  {
    cout << "  MapMaker: CalcPlane: too few points to calc plane." << endl;
    return SE3<>();
  }

  int nRansacs = GV2.GetInt("MapMaker.PlaneAlignerRansacs", 100, HIDDEN|SILENT);
  Vector<3> v3BestMean;
  Vector<3> v3BestNormal;
  double dBestDistSquared = 9999999999999999.9;

  for(int i=0; i<nRansacs; i++)
  {
    int nA = rand()%nPoints;
    int nB = nA;
    int nC = nA;
    while(nB == nA)
      nB = rand()%nPoints;
    while(nC == nA || nC==nB)
      nC = rand()%nPoints;

    Vector<3> v3Mean = 0.33333333 * (vpPoints[nA]->v3WorldPos +
                                     vpPoints[nB]->v3WorldPos +
                                     vpPoints[nC]->v3WorldPos);

    Vector<3> v3CA = vpPoints[nC]->v3WorldPos  - vpPoints[nA]->v3WorldPos;
    Vector<3> v3BA = vpPoints[nB]->v3WorldPos  - vpPoints[nA]->v3WorldPos;
    Vector<3> v3Normal = v3CA ^ v3BA;
    if( (v3Normal * v3Normal) == 0 )
      continue;
    normalize(v3Normal);

    double dSumError = 0.0;
    for(unsigned int i=0; i<nPoints; i++)
    {
      Vector<3> v3Diff = vpPoints[i]->v3WorldPos - v3Mean;
      double dDistSq = v3Diff * v3Diff;
      if(dDistSq == 0.0)
        continue;
      double dNormDist = fabs(v3Diff * v3Normal);

      if(dNormDist > 0.05)
        dNormDist = 0.05;
      dSumError += dNormDist;
    }

    if(dSumError < dBestDistSquared)
    {
      dBestDistSquared = dSumError;
      v3BestMean = v3Mean;
      v3BestNormal = v3Normal;
    }
  }

  // Done the ransacs, now collect the supposed inlier set
  vector<Vector<3> > vv3Inliers;
  for(unsigned int i=0; i<nPoints; i++)
  {
    Vector<3> v3Diff = vpPoints[i]->v3WorldPos - v3BestMean;
    double dDistSq = v3Diff * v3Diff;
    if(dDistSq == 0.0)
      continue;
    double dNormDist = fabs(v3Diff * v3BestNormal);
    if(dNormDist < 0.05)
      vv3Inliers.push_back(vpPoints[i]->v3WorldPos);
  }

  // With these inliers, calculate mean and cov
  Vector<3> v3MeanOfInliers = Zeros;
  for(unsigned int i=0; i<vv3Inliers.size(); i++)
    v3MeanOfInliers+=vv3Inliers[i];
  v3MeanOfInliers *= (1.0 / vv3Inliers.size());

  Matrix<3> m3Cov = Zeros;
  for(unsigned int i=0; i<vv3Inliers.size(); i++)
  {
    Vector<3> v3Diff = vv3Inliers[i] - v3MeanOfInliers;
    m3Cov += v3Diff.as_col() * v3Diff.as_row();
  }

  // Find the principal component with the minimal variance: this is the plane normal
  SymEigen<3> sym(m3Cov);
  Vector<3> v3Normal = sym.get_evectors()[0];

  // Use the version of the normal which points towards the cam center
  if(v3Normal[2] > 0)
    v3Normal *= -1.0;

  Matrix<3> m3Rot = Identity;
  m3Rot[2] = v3Normal;
  m3Rot[0] = m3Rot[0] - (v3Normal * (m3Rot[0] * v3Normal));
  normalize(m3Rot[0]);
  m3Rot[1] = m3Rot[2] ^ m3Rot[0];

  SE3<> se3Aligner;
  se3Aligner.get_rotation() = m3Rot;
  Vector<3> v3RMean = se3Aligner * v3MeanOfInliers;
  se3Aligner.get_translation() = -v3RMean;

  return se3Aligner;
}

double Map::KeyFrameLinearDist(const KeyFrame &k1, const KeyFrame &k2) const
{
  Vector<3> v3KF1_CamPos = k1.se3CfromW.inverse().get_translation();
  Vector<3> v3KF2_CamPos = k2.se3CfromW.inverse().get_translation();
  Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;
  return sqrt(v3Diff * v3Diff); // Dist
}

vector<KeyFrame*> Map::NClosestKeyFrames(const KeyFrame &k, unsigned int N) const
{
  vector<pair<double, KeyFrame* > > vKFandScores;
  for(size_t i=0; i < vpKeyFrames.size(); ++i)
  {
    if(vpKeyFrames[i] == &k)
      continue;
    double dDist = KeyFrameLinearDist(k, *vpKeyFrames[i]);
    vKFandScores.push_back(make_pair(dDist, vpKeyFrames[i]));
  }

  N = min<unsigned>(N, vKFandScores.size());
  partial_sort(vKFandScores.begin(), vKFandScores.begin() + N, vKFandScores.end());

  vector<KeyFrame*> vResult;
  for(unsigned int i=0; i<N; i++)
    vResult.push_back(vKFandScores[i].second);

  return vResult;
}

KeyFrame* Map::ClosestKeyFrame(const KeyFrame &k)
{
  double dClosestDist = 9999999999.9;
  int nClosest = -1;
  for(size_t i=0; i < vpKeyFrames.size(); ++i)
  {
    if(vpKeyFrames[i] == &k)
      continue;

    double dDist = KeyFrameLinearDist(k, *vpKeyFrames[i]);
    if(dDist < dClosestDist)
    {
      dClosestDist = dDist;
      nClosest = i;
    }
  }

  assert(nClosest != -1);

  return vpKeyFrames[nClosest];
}

double Map::DistToNearestKeyFrame(const KeyFrame &kCurrent)
{
  KeyFrame *pClosest = ClosestKeyFrame(kCurrent);
  return KeyFrameLinearDist(kCurrent, *pClosest); // Dist
}


/**
 * HandleBadPoints() Does some heuristic checks on all points in the map to see if
 * they should be flagged as bad, based on tracker feedback.
 */
void Map::HandleBadPoints()
{
  // Did the tracker see this point as an outlier more often than as an inlier?
  for (size_t i = 0; i < vpPoints.size(); ++i)
  {
    MapPoint *p = vpPoints[i];
    if(p->nMEstimatorOutlierCount > 20 && p->nMEstimatorOutlierCount > p->nMEstimatorInlierCount) {
      p->bBad = true;
    }
  }

  // All points marked as bad will be erased - erase all records of them
  // from keyframes in which they might have been measured.
  for (size_t i = 0; i < vpPoints.size(); ++i)
  {
    if(vpPoints[i]->bBad) {
      MapPoint *p = vpPoints[i];
      for(size_t j = 0; j < vpKeyFrames.size(); ++j)
      {
        KeyFrame &k = *vpKeyFrames[j];
        if(k.mMeasurements.count(p)) {
          k.mMeasurements.erase(p);
        }
      }
    }
  }

  // Move bad points to the trash list.
  MoveBadPointsToTrash();
}

/**
 * Move any points marked as bad to the trash
 */
void Map::MoveBadPointsToTrash()
{
  int nBad = 0;
  for(int i = vpPoints.size()-1; i>=0; i--)
    {
      if(vpPoints[i]->bBad)
        {
          vpPointsTrash.push_back(vpPoints[i]);
          vpPoints.erase(vpPoints.begin() + i);
          nBad++;
        }
    };
}


/**
 * Delete of the points in the trash
 */
void Map::EmptyTrash()
{
  for(unsigned int i=0; i<vpPointsTrash.size(); i++)
    delete vpPointsTrash[i];
  vpPointsTrash.clear();
}


// Rotates/translates the whole map and all keyframes
void Map::ApplyGlobalTransformation(const SE3<>& se3NewFromOld)
{
  SE3<> se3OlfFromNew = se3NewFromOld.inverse();
  for(size_t i=0; i < vpKeyFrames.size(); ++i) {
    vpKeyFrames[i]->se3CfromW = vpKeyFrames[i]->se3CfromW * se3OlfFromNew;
  }

  for(size_t i=0; i < vpPoints.size(); ++i) {
    vpPoints[i]->v3WorldPos = se3NewFromOld * vpPoints[i]->v3WorldPos;
    vpPoints[i]->RefreshPixelVectors();
  }
}

// Applies a global scale factor to the map
void Map::ApplyGlobalScale(double dScale)
{
  for(size_t i=0; i < vpKeyFrames.size(); ++i) {
    vpKeyFrames[i]->se3CfromW.get_translation() *= dScale;
  }

  for(size_t i=0; i < vpPoints.size(); ++i) {
    vpPoints[i]->v3WorldPos *= dScale;
    vpPoints[i]->v3PixelRight_W *= dScale;
    vpPoints[i]->v3PixelDown_W *= dScale;
    vpPoints[i]->RefreshPixelVectors();
  }
}

void Map::InitTexture()
{
  nTex = 0;
  glGenTextures(N,&texName[0]);//@hack by camparijet for adding keyframe
}

void Map::MakeTextureFromKF(KeyFrame &k)
{
  glBindTexture(GL_TEXTURE_2D, texName[nTex]);
  glTexImage2D(k.im_cl);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  k.tIndex = nTex;
  //cout << "Added Keyframe : " << nTex << endl;
  nTex++;
}

void Map::ReleaseTexture()
{
  glDeleteTextures(N,&texName[0]);
}

}

