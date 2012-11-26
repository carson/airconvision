#include "TargetController.h"

#include <iostream>

namespace PTAMM {

using namespace TooN;
using namespace std::chrono;

typedef duration<double> RealSeconds;

void TargetController::Update(const SE3<> &se3Pose, bool bHasTracking, const TimePoint& t)
{
  SE3<> se3PoseCorrection;
  se3PoseCorrection.get_rotation() = SO3<>(makeVector(0, 1, 0), makeVector(1, 0, 0));
  SE3<> se3FixedPose = se3Pose; // * se3PoseCorrection;

  // Check if this is the first update
  if (mLastUpdate.time_since_epoch() == Clock::duration::zero()) {
    mv3PrevPosInWorld = se3FixedPose.inverse().get_translation();
    mStartTime = mLastUpdate = t;

    std::cout << "first tick!" << std::endl;
    return;
  }

  double dt = duration_cast<RealSeconds>(t - mLastUpdate).count(); // delta time in seconds
  mLastUpdate = t;

  assert(dt > 0.0001);
  if (dt < 0.0001) {
    return;
  }






  if (bHasTracking) {
    // Calculate the yaw-only rotation transformation
    SO3<> so3Rotation = se3FixedPose.get_rotation();
    Vector<3> v3Up = makeVector(0, 0, 1);
    Vector<3> v3RotatedUp = so3Rotation * v3Up;
    Vector<3> v3Diff = v3RotatedUp + v3Up;
    if (v3Diff * v3Diff < 0.0001) {
      // The up vector is 180 deg rotated, which causes the SO3(a,b) constructor to fail
      SO3<> so3StraightenUp = SO3<>::exp(makeVector(M_PI,0,0)); // Very untested...!
      so3Rotation = so3StraightenUp * so3Rotation;
    } else {
      SO3<> so3StraightenUp(v3RotatedUp, -v3Up);
      so3Rotation = so3StraightenUp * so3Rotation;
    }

    // Offset calculation
    Vector<3> v3PosInWorld = se3FixedPose.inverse().get_translation();
    Vector<3> v3OffsetInWorld = mv3TargetPosInWorld - v3PosInWorld;
    mv3Offset = so3Rotation * v3OffsetInWorld;
    if (mReset) mOffsetFilter.Reset();
    mOffsetFilter.Update(mv3Offset);

    // Velocity calculation
    if (mReset) {
      mv3Velocity = makeVector(0, 0, 0);
      mVelocityFilter.Reset();
    }
    else {
      Vector<3> v3DeltaPos = v3PosInWorld - mv3PrevPosInWorld;
      mv3Velocity = so3Rotation * v3DeltaPos * (1.0 / dt);
    }
    mVelocityFilter.Update(mv3Velocity);
    mv3PrevPosInWorld = v3PosInWorld;



    // Setting thrust for mode transitions
    if ((mConfig == (ENGAGED | TAKEOFF | TRACKING)) && mv3Offset[2] > 0.2) {
      mConfig = ENGAGED | TRACKING;
    }
    else if ((mConfig & 0x3) != mConfigRqst) {
      if (!(mConfig & (ENGAGED | TAKEOFF)) && (mConfigRqst == ENGAGED)) {
        mControl[4] = (double) mHoverGas / 4.;
      }
      else {
        mControl[4] = 0;
      }
      mConfig = mConfigRqst | TRACKING;
    }

    // Control law
    if (mConfig & ENGAGED) {
      if (mConfig & TAKEOFF) {
        mControl[4] += 30. * dt;
        if (mControl[4] > 150.) mControl[4] = 150.;
      }
      else {
        mControl[4] = 40; // Just kill it for now
      }
    }
  } // end of if(bHasTracking)
  else {
    mControl[0] = 0.;
    mControl[1] = 0.;
    mControl[2] = 0.;
    mControl[3] = 0.;
  }
}

void TargetController::SetTarget(const TooN::SE3<> &se3PoseInWorld)
{
  mv3TargetPosInWorld = se3PoseInWorld.get_translation();
  mOffsetFilter.Reset();
}

double TargetController::GetTime() const
{
  return duration_cast<RealSeconds>(mLastUpdate - mStartTime).count();
}

}

