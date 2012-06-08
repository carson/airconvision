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
#include <condition_variable>
#include <functional>
#include <thread>

// CHECK_RESET is a handy macro which makes the mapmaker thread stop
// what it's doing and reset, if required.
#define CHECK_UNLOCK mpMap->mapLockManager.CheckLockAndWait( this, 0 );
#define CKECK_ABORTS

#if 0
#   define DEBUG_MAP_MAKER(x) x
#else
#   define DEBUG_MAP_MAKER(x)
#endif

namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;


bool ActionDispatcher::Empty() const
{
  std::lock_guard<std::mutex> lock(m);
  return mvQueuedActions.empty();
}

void ActionDispatcher::RunQueuedActions()
{
  bool bHasActions = false;
  {
    std::lock_guard<std::mutex> lock(m);
    bHasActions = !mvQueuedActions.empty();
  }

  while (bHasActions) {

    Action fn;

    {
      std::lock_guard<std::mutex> lock(m);
      fn = std::move(mvQueuedActions.front());
      mvQueuedActions.pop();
      bHasActions = !mvQueuedActions.empty();
    }

    fn();
  }
}

void ActionDispatcher::PushAction(const Action &a)
{
  std::lock_guard<std::mutex> lock(m);
  mvQueuedActions.push(a);
}

void ActionDispatcher::PushActionAndWait(const Action &a)
{
  std::condition_variable cond;
  bool bDone = false;

  {
    std::lock_guard<std::mutex> lock(m);
    mvQueuedActions.push([&a, &cond, &bDone] () {
        a();
        bDone = true;
        cond.notify_one();
    });
  }

  std::unique_lock<std::mutex> lock(m);
  while(!bDone) {
    cond.wait(lock);
  }
}


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
    mbStereoInitDone(false)
{
  mpMap->mapLockManager.Register( this );

  mpMap->Reset();
}

/**
 * Destructor
 */
MapMaker::~MapMaker()
{
  mbAbortRequested = true;
  mpMap->mapLockManager.UnRegister( this );
}

/**
 * Run the map maker thread
 */
void MapMaker::operator()()
{
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Perform all queued actions
    mDispatcher.RunQueuedActions();

    // Nothing to do if there is no map yet! or is locked
    if (!mpMap->IsGood() || mpMap->bEditLocked) {
      continue;
    }

    // From here on, mapmaker does various map-maintenance jobs in a certain priority
    // Hierarchy. For example, if there's a new key-frame to be added (QueueSize() is >0)
    // then that takes high priority.

    // Should we run local bundle adjustment?
    if(!mpMap->RecentBundleAdjustConverged() && mpMap->QueueSize() == 0) {
      mbAbortRequested = false;
      DEBUG_MAP_MAKER(cout << "START: Recent bundle adjustment" << endl);
      if (!mpMap->RecentBundleAdjust(&mbAbortRequested)) {
        ResetImpl();
      }
      DEBUG_MAP_MAKER(cout << "  END: Recent bundle adjustment: " << mbAbortRequested << endl);
    }

    if (mbAbortRequested || !mDispatcher.Empty()) continue;

    // Are there any newly-made map points which need more measurements from older key-frames?
    if(mpMap->RecentBundleAdjustConverged() && mpMap->QueueSize() == 0) {
      DEBUG_MAP_MAKER(cout << "START: Refining newly made map points" << endl);
      mpMap->ReFindNewlyMade();
      DEBUG_MAP_MAKER(cout << "  END: Refining newly made map points" << endl);
    }

    if (mbAbortRequested || !mDispatcher.Empty()) continue;

    // Run global bundle adjustment?
    if(mpMap->RecentBundleAdjustConverged() && !mpMap->FullBundleAdjustConverged() && mpMap->QueueSize() == 0) {

    // I added this rule to avoid starting a full bundle adjust if there were waiting KFs,
    // otherwise the camera loses tracking with fast camera movements -- dhenell
    //if (mpMap->QueueSize() == 0) {
      mbAbortRequested = false;
      DEBUG_MAP_MAKER(cout << "START: Full bundle adjustment" << endl);
      if (!mpMap->FullBundleAdjust(&mbAbortRequested)) {
        ResetImpl();
      }
      DEBUG_MAP_MAKER(cout << "  END: Full bundle adjustment: " << mbAbortRequested << endl);
    }

    if (mbAbortRequested || !mDispatcher.Empty()) continue;

    // Very low priorty: re-find measurements marked as outliers
    if(mpMap->RecentBundleAdjustConverged() && mpMap->FullBundleAdjustConverged() &&
        rand()%20 == 0 && mpMap->QueueSize() == 0)
    {
      DEBUG_MAP_MAKER(cout << "START: Refining failed map points" << endl);
      mpMap->ReFindFromFailureQueue();
      DEBUG_MAP_MAKER(cout << "  END: Refining failed map points" << endl);
    }

    if (mbAbortRequested || !mDispatcher.Empty()) continue;

    DEBUG_MAP_MAKER(cout << "START: Handling bad points" << endl);
    mpMap->HandleBadPoints();
    DEBUG_MAP_MAKER(cout << "  END: Handling bad points" << endl);
  }
}


/**
 * Reinitialize the map maker for a new map
 */
void MapMaker::ReInitImpl()
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
}

/**
 * Reset a map.
 */
void MapMaker::ResetImpl2(Map * map)
{
  if (map == NULL) {
    throw std::runtime_error("Trying to reset a null map");
  }

  // This is only called from within the mapmaker thread...
  map->Reset();
}

/**
 * Switch maps
 */
void MapMaker::SwitchMapImpl()
{
  //change
  mpMap->mapLockManager.UnRegister( this );
  mpMap = mpSwitchMap;
  mpMap->mapLockManager.Register( this );
  //load current state?
}


/**
 * Tracker calls this to demand a reset
 */
void MapMaker::Reset()
{
  mbAbortRequested = true;
  mDispatcher.PushActionAndWait(std::bind(&MapMaker::ResetImpl, this));
}

/**
 * System calls this to demand a reinitialization for
 * a new map to be inserted
 * @param map the new map
 */
void MapMaker::ReInit(Map * map)
{
  mpNewMap = map;

  mbAbortRequested = true;
  mDispatcher.PushActionAndWait(std::bind(&MapMaker::ReInitImpl, this));
}

/**
 * System calls this to request a switch the supplied map
 * @param map map to switch to
 */
bool MapMaker::Switch(Map * map)
{
  if( map == NULL ) {
    return false;
  }

  mpSwitchMap = map;
  mpNewMap = map;

  mbAbortRequested = true;
  mDispatcher.PushActionAndWait(std::bind(&MapMaker::SwitchMapImpl, this));

  return true;
}

void MapMaker::RequestRealignment()
{
  if(!mpMap->IsGood() || mpMap->bEditLocked ) { // Nothing to do if there is no map yet! or is locked
    return;
  }

  mDispatcher.PushAction([mpMap] () {
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

  mDispatcher.PushAction([se3NewFromOld, mpMap] () {
    mpMap->ApplyGlobalTransformation(se3NewFromOld);
  });
}

void MapMaker::RequestMapScaling(double dScale)
{
  if(!mpMap->IsGood() || mpMap->bEditLocked ) { // Nothing to do if there is no map yet! or is locked
    return;
  }

  mDispatcher.PushAction([dScale, mpMap] () {
    mpMap->ApplyGlobalScale(dScale);
  });
}

void MapMaker::InitFromStereo(KeyFrame &kFirst, KeyFrame &kSecond,
                              std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
                              SE3<> &se3CameraPos)
{
  mbStereoInitDone = false;
  mbAbortRequested = true;
  mDispatcher.PushAction([kFirst, kSecond, vMatches, se3CameraPos, mpMap, &mbAbortRequested, &mbStereoInitDone] () mutable {
    mbAbortRequested = false;
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

  mbAbortRequested = true;
  mDispatcher.PushAction([mpMap] () {
    if (mpMap->IsGood() && !mpMap->bEditLocked) {
      mpMap->AddKeyFrameFromTopOfQueue();
    }
  });
}

}
