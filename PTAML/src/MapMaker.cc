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
MapMaker::MapMaker(Map* m)
  : mbDone(false)
  , mpMap(m)
  , mbAbortRequested(false)
  , mbStereoInitDone(false)
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
  while (!mbDone) {
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
        mpMap->Reset();
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
        mpMap->Reset();
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
 * Tracker calls this to demand a reset
 */
void MapMaker::Reset(bool async)
{
  mbAbortRequested = true;

  auto action = [this] () { mpMap->Reset(); };

  if (async) {
    mDispatcher.PushAction(action);
  } else {
    mDispatcher.PushActionAndWait(action);
  }
}

void MapMaker::RealignGroundPlane(bool async)
{
  if(!mpMap->IsGood() || mpMap->bEditLocked ) { // Nothing to do if there is no map yet! or is locked
    return;
  }

  auto action = [this] () {
    SE3<> se3GroundAlignment = mpMap->CalcPlaneAligner(false);

    // Don't align in the XY-plane!
    se3GroundAlignment.get_translation()[0] = 0;
    se3GroundAlignment.get_translation()[1] = 0;

    mpMap->ApplyGlobalTransformation(se3GroundAlignment);
  };

  if (async) {
    mDispatcher.PushAction(action);
  } else {
    mDispatcher.PushActionAndWait(action);
  }
}

void MapMaker::TransformMapPoints(const SE3<> &se3NewFromOld, bool async)
{
  if(!mpMap->IsGood() || mpMap->bEditLocked ) { // Nothing to do if there is no map yet! or is locked
    return;
  }

  auto action = [se3NewFromOld, this] () {
    mpMap->ApplyGlobalTransformation(se3NewFromOld);
  };

  if (async) {
    mDispatcher.PushAction(action);
  } else {
    mDispatcher.PushActionAndWait(action);
  }
}

void MapMaker::ScaleMapPoints(double dScale, bool async)
{
  if(!mpMap->IsGood() || mpMap->bEditLocked ) { // Nothing to do if there is no map yet! or is locked
    return;
  }

  auto action = [dScale, this] () {
    mpMap->ApplyGlobalScale(dScale);
  };

  if (async) {
    mDispatcher.PushAction(action);
  } else {
    mDispatcher.PushActionAndWait(action);
  }
}

void MapMaker::InitFromStereo(KeyFrame &kFirst, KeyFrame &kSecond,
                              std::vector<std::pair<CVD::ImageRef, CVD::ImageRef> > &vMatches,
                              SE3<> *se3CameraPos, bool async)
{
  mbStereoInitDone = false;
  mbAbortRequested = true;

  auto action = [kFirst, kSecond, vMatches, se3CameraPos, this] () mutable {
    mbAbortRequested = false;
    mpMap->InitFromStereo(kFirst, kSecond, vMatches, se3CameraPos, &mbAbortRequested);
    mbStereoInitDone = true;
  };

  if (async) {
    mDispatcher.PushAction(action);
  } else {
    mDispatcher.PushActionAndWait(action);
  }
}

void MapMaker::InitFromStereo(KeyFrame &kFirst, KeyFrame &kSecond,
                              const SE3<> &se3SecondCameraPos)
{
  mbStereoInitDone = false;
  mbAbortRequested = true;
  mDispatcher.PushAction([kFirst, kSecond, se3SecondCameraPos, this] () mutable {
    mbAbortRequested = false;
    mpMap->InitFromStereo(kFirst, kSecond, se3SecondCameraPos, &mbAbortRequested);
    mbStereoInitDone = true;
  });
}

void MapMaker::InitFromKnownPlane(KeyFrame &kKeyFrame, const TooN::Vector<4> &v4GroundPlane, SE3<> &se3TrackerPose)
{
  mbAbortRequested = true;
  mDispatcher.PushActionAndWait([kKeyFrame, v4GroundPlane, this, &se3TrackerPose] () {
    mpMap->InitFromKnownPlane(kKeyFrame, v4GroundPlane, se3TrackerPose);
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
  if (mpMap->bEditLocked) {
    return;
  }

  mpMap->QueueKeyFrame(k);

  mbAbortRequested = true;
  mDispatcher.PushAction([this] () {
    if (mpMap->IsGood() && !mpMap->bEditLocked) {
      mpMap->AddKeyFrameFromTopOfQueue();
    }
  });
}

}
