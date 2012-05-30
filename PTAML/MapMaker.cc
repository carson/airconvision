// Copyright 2008 Isis Innovation Limited
#include "MapMaker.h"
#include "MapPoint.h"
#include "PatchFinder.h"
#include "SmallMatrixOpts.h"
#include "HomographyInit.h"
#include "SmallBlurryImage.h"

#include <cvd/vector_image_ref.h>
#include <cvd/vision.h>
#include <cvd/image_interpolate.h>

#include <TooN/SVD.h>
#include <TooN/SymEigen.h>

#include <gvars3/instances.h>
#include <fstream>
#include <algorithm>
#include <chrono>

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;


/**
 * Constructor sets up internal reference variable to Map.
 * Most of the intialisation is done by Reset()..
 * @param maps list of maps
 * @param m current map
 */
MapMaker::MapMaker(std::vector<Map*> &maps, Map* m)
  : mvpMaps(maps),
    mpMap(m),
    mbAbortRequested(false),
    mbResetRequested(false),
    mbResetDone(true),
    mbReInitRequested(false),
    mbReInitDone(false),
    mbSwitchRequested(false),
    mbSwitchDone(false),
    mbStereoInitDone(false)
{
  mdMaxKFDistWiggleMult = GV3::get<double>("MapMaker.MaxKFDistWiggleMult", 1.0, SILENT);

  mpMap->mapLockManager.Register( this );

  Reset();

  start(); // This CVD::thread func starts the map-maker thread with function run()
}

/**
 * Destructor
 */
MapMaker::~MapMaker()
{
  mbAbortRequested = true;
  stop(); // makes shouldStop() return true
  cout << "Waiting for mapmaker to die.." << endl;
  join();
  cout << " .. mapmaker has died." << endl;
  mpMap->mapLockManager.UnRegister( this );
}

/**
 * Reinitialize the map maker for a new map
 */
void MapMaker::ReInit()
{
  if(mpMap == mpNewMap)
  {
    cerr << "*** WARNING: MapMaker::ReInit() changing to same map! ***" << endl;
  }
  else
  {
    mpMap->mapLockManager.UnRegister( this );
    mpMap = mpNewMap;
    mpMap->mapLockManager.Register( this );
    Reset();
  }

  mbReInitRequested = false;
  mbReInitDone = true;
}

/**
 * Reset a map.
 */
void MapMaker::Reset(Map * map)
{
  if(map == NULL)
  {
    cerr << "*** ERROR: Trying to reset a null map ***" << endl;
    exit(1);
  }

  // This is only called from within the mapmaker thread...
  map->Reset();

  mbResetDone = true;
  mbResetRequested = false;
}

/**
 * Switch maps
 */
void MapMaker::SwitchMap()
{
  //change
  mpMap->mapLockManager.UnRegister( this );
  mpMap = mpSwitchMap;
  mpMap->mapLockManager.Register( this );
  //load current state?

  //set bools
  mbSwitchRequested = false;
  mbSwitchDone = true;
}

// CHECK_RESET is a handy macro which makes the mapmaker thread stop
// what it's doing and reset, if required.
#define CHECK_RESET if(mbResetRequested) {Reset(); continue;};

#define CHECK_REINIT if(mbReInitRequested) {ReInit(); continue;}
#define CHECK_SWITCH if(mbSwitchRequested) { SwitchMap(); continue; }
#define CHECK_UNLOCK mpMap->mapLockManager.CheckLockAndWait( this, 0 );

#define CKECK_ABORTS CHECK_RESET CHECK_REINIT CHECK_SWITCH CHECK_UNLOCK

#define DEBUG_MAP_MAKER(x) x

/**
 * Run the map maker thread
 */
void MapMaker::run()
{
#ifdef WIN32
  // For some reason, I get tracker thread starvation on Win32 when
  // adding key-frames. Perhaps this will help:
  SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_LOWEST);
  ///@TODO Try setting this to THREAD_PRIORITY_HIGHEST to see how it effects your performance.
#endif

  while(!shouldStop())  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
  {
    CKECK_ABORTS;
    sleep(5); // Sleep not really necessary, especially if mapmaker is busy
    CKECK_ABORTS;

    // Handle any GUI commands encountered..
    while(!mvQueuedCommands.empty())
    {
      GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
      mvQueuedCommands.erase(mvQueuedCommands.begin());
    }

    // Perform all queued actions

    {
      std::lock_guard<std::mutex> lock(m);
      while (!mvQueuedActions.empty()) {
        CKECK_ABORTS;
        mvQueuedActions.front()();
        mvQueuedActions.pop();
      }
    }

    if(!mpMap->IsGood() || mpMap->bEditLocked )  // Nothing to do if there is no map yet! or is locked
      continue;


    // From here on, mapmaker does various map-maintenance jobs in a certain priority
    // Hierarchy. For example, if there's a new key-frame to be added (QueueSize() is >0)
    // then that takes high priority.

    CKECK_ABORTS;
    // Should we run local bundle adjustment?
    if(!mpMap->RecentBundleAdjustConverged() && mpMap->QueueSize() == 0) {
      mbAbortRequested = false;
      DEBUG_MAP_MAKER(cout << "START: Recent bundle adjustment" << endl);
      if (!mpMap->RecentBundleAdjust(&mbAbortRequested)) {
        mbResetRequested = true;
      }
      DEBUG_MAP_MAKER(cout << "  END: Recent bundle adjustment: " << mbAbortRequested << endl);
    }

    CKECK_ABORTS;
    // Are there any newly-made map points which need more measurements from older key-frames?
    if(mpMap->RecentBundleAdjustConverged() && mpMap->QueueSize() == 0) {
      DEBUG_MAP_MAKER(cout << "START: Refining newly made map points" << endl);
      mpMap->ReFindNewlyMade();
      DEBUG_MAP_MAKER(cout << "  END: Refining newly made map points" << endl);
    }

    CKECK_ABORTS;
    // Run global bundle adjustment?
    if(mpMap->RecentBundleAdjustConverged() && !mpMap->FullBundleAdjustConverged() && mpMap->QueueSize() == 0) {

    // I added this rule to avoid starting a full bundle adjust if there were waiting KFs,
    // otherwise the camera loses tracking with fast camera movements -- dhenell
    //if (mpMap->QueueSize() == 0) {
      mbAbortRequested = false;
      DEBUG_MAP_MAKER(cout << "START: Full bundle adjustment" << endl);
      if (!mpMap->FullBundleAdjust(&mbAbortRequested)) {
        mbResetRequested = true;
      }
      DEBUG_MAP_MAKER(cout << "  END: Full bundle adjustment: " << mbAbortRequested << endl);
    }

    CKECK_ABORTS;
    // Very low priorty: re-find measurements marked as outliers
    if(mpMap->RecentBundleAdjustConverged() && mpMap->FullBundleAdjustConverged() &&
        rand()%20 == 0 && mpMap->QueueSize() == 0)
    {
      DEBUG_MAP_MAKER(cout << "START: Refining failed map points" << endl);
      mpMap->ReFindFromFailureQueue();
      DEBUG_MAP_MAKER(cout << "  END: Refining failed map points" << endl);
    }

    CKECK_ABORTS;
    DEBUG_MAP_MAKER(cout << "START: Handling bad points" << endl);
    mpMap->HandleBadPoints();
    DEBUG_MAP_MAKER(cout << "  END: Handling bad points" << endl);

    CKECK_ABORTS;
    // Any new key-frames to be added?
    if(mpMap->QueueSize() > 0) { // this is unnecessary since AddKeyFrameFromTopOfQueue does this check anyway
      DEBUG_MAP_MAKER(cout << "START: Adding key frame" << endl);
      mpMap->AddKeyFrameFromTopOfQueue(); // Integrate into map data struct, and process
      DEBUG_MAP_MAKER(cout << "  END: Adding key frame" << endl);
    }
  }
}

/**
 * Tracker calls this to demand a reset
 */
void MapMaker::RequestReset()
{
  mbResetDone = false;
  mbResetRequested = true;
  mbAbortRequested = true;
}

/**
 * check if the reset has been done
 */
bool MapMaker::ResetDone()
{
  return mbResetDone;
}

/**
 * System calls this to demand a reinitialization for
 * a new map to be inserted
 * @param map the new map
 */
void MapMaker::RequestReInit(Map * map)
{
  mpNewMap = map;
  mbReInitDone = false;
  mbReInitRequested = true;
}

/**
 * Check if the reinitialization has been done
 * @return reinitialization done?
 */
bool MapMaker::ReInitDone()
{
  return mbReInitDone;
}

/**
 * System calls this to request a switch the supplied map
 * @param map map to switch to
 */
bool MapMaker::RequestSwitch(Map * map)
{
  if( map == NULL ) {
    return false;
  }

  mpSwitchMap = map;
  mbSwitchDone = false;
  mbSwitchRequested = true;

  return true;
}

/**
 * Check if the switch has been done
 * @return switch done
 */
bool MapMaker::SwitchDone()
{
  return mbSwitchDone;
}

void MapMaker::RequestRealignment()
{
  if(!mpMap->IsGood() || mpMap->bEditLocked ) { // Nothing to do if there is no map yet! or is locked
    return;
  }

  std::lock_guard<std::mutex> lock(m);

  mvQueuedActions.push([mpMap] () {
    SE3<> se3GroundAlignment = mpMap->CalcPlaneAligner(false);

    // Don't align in the XY-plane!
    se3GroundAlignment.get_translation()[0] = 0;
    se3GroundAlignment.get_translation()[1] = 0;

    mpMap->ApplyGlobalTransformation(se3GroundAlignment);
  });
}

void MapMaker::RequestMapTransformation(const SE3<> &se3NewFromOld)
{
  if(!mpMap->IsGood() || mpMap->bEditLocked ) { // Nothing to do if there is no map yet! or is locked
    return;
  }

  std::lock_guard<std::mutex> lock(m);

  mvQueuedActions.push([se3NewFromOld, mpMap] () {
    mpMap->ApplyGlobalTransformation(se3NewFromOld);
  });
}

void MapMaker::RequestMapScaling(double dScale)
{
  if(!mpMap->IsGood() || mpMap->bEditLocked ) { // Nothing to do if there is no map yet! or is locked
    return;
  }

  std::lock_guard<std::mutex> lock(m);

  mvQueuedActions.push([dScale, mpMap] () {
    mpMap->ApplyGlobalScale(dScale);
  });
}

void MapMaker::RequestCallback(std::function<void()> fn)
{
  std::lock_guard<std::mutex> lock(m);

  mvQueuedActions.push([=] () { fn(); });
}

void MapMaker::InitFromStereo(KeyFrame &kFirst, KeyFrame &kSecond,
                              std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
                              SE3<> &se3CameraPos)
{
  std::lock_guard<std::mutex> lock(m);

  mbStereoInitDone = false;
  mbAbortRequested = false;
  mvQueuedActions.push([kFirst, kSecond, vMatches, se3CameraPos, mpMap, &mbAbortRequested, &mbStereoInitDone] () mutable {
    mpMap->InitFromStereo(kFirst, kSecond, vMatches, se3CameraPos, &mbAbortRequested);
    mbStereoInitDone = true;
  });
}

/**
 * The tracker entry point for adding a new keyframe;
 * the tracker thread doesn't want to hang about, so
 * just dumps it on the top of the mapmaker's queue to
 * be dealt with later, and return.
 * @param k the new keyframe
 */
void MapMaker::AddKeyFrame(const KeyFrame &k)
{
  if( mpMap->bEditLocked ) {
    return;
  }

  mpMap->QueueKeyFrame(k);

  // Tell the mapmaker to stop doing low-priority stuff and concentrate on this KF first.
  mbAbortRequested = true;
}

/*Hack camparijet*/
//@todo change NeedNewKeyFrame condition
bool MapMaker::NeedNewKeyFrame(const KeyFrame &kCurrent)
{
  KeyFrame *pClosest = mpMap->ClosestKeyFrame(kCurrent);
  double dDist = mpMap->KeyFrameLinearDist(kCurrent, *pClosest);
  dDist *= (1.0 / kCurrent.dSceneDepthMean);

  return dDist > mdMaxKFDistWiggleMult * mpMap->GetWiggleScaleDepthNormalized();
}

// Is the tracker's camera pose in cloud-cuckoo land?
bool MapMaker::IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent)
{
  return mpMap->DistToNearestKeyFrame(kCurrent) > mpMap->GetWiggleScale() * 10.0;
}


void MapMaker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  Command c;
  c.sCommand = sCommand;
  c.sParams = sParams;
  ((MapMaker*) ptr)->mvQueuedCommands.push_back(c);
}

void MapMaker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
  cout << "! MapMaker::GUICommandHandler: unhandled command "<< sCommand << endl;
}

}
