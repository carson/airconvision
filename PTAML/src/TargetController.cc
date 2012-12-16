#include "TargetController.h"

#include <iostream>

namespace PTAMM {

using namespace TooN;
using namespace std;
using namespace std::chrono;

typedef duration<double> RealSeconds;

void TargetController::Update(const SE3<> &se3Pose, bool bHasTracking, const TimePoint& t)
{
  // Check if this is the first update
  if (mLastUpdate.time_since_epoch() == Clock::duration::zero()) {
    mStartTime = mLastUpdate = t;
    // TODO: Put this in the constructor?
    // Initialize private members
    mv3TargetPosInWorld = Zeros;
    mv3PrevPosInWorld = Zeros;
    mv3Offset = Zeros;
    mv3Velocity = Zeros;
    memset(mControlInt, 0, sizeof(double)*3);
    memset(mControl, 0, sizeof(double)*4);
    mControl[4] = 127.;
    mConfig = 0; mConfigRqst = 0;
    mHoverGas = 127;
    mReset = true;

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
    SO3<> so3Rotation = se3Pose.get_rotation();
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
    Vector<3> v3PosInWorld = se3Pose.inverse().get_translation();
    Vector<3> v3OffsetInWorld = mv3TargetPosInWorld - v3PosInWorld;
    mv3Offset = so3Rotation * v3OffsetInWorld;
    if (mReset) mOffsetFilter.Reset();
    mOffsetFilter.Update(mv3Offset);
    Vector<3> v3OffsetFiltered = mOffsetFilter.GetValue();

    // Velocity calculation
    if (mReset) {
      // TODO: Find out of this causes a memory leak
      mv3Velocity = makeVector(0, 0, 0);
      mVelocityFilter.Reset();
    }
    else {
      Vector<3> v3DeltaPos = v3PosInWorld - mv3PrevPosInWorld;
      mv3Velocity = so3Rotation * v3DeltaPos * (1.0 / dt);
    }
    mVelocityFilter.Update(mv3Velocity);
    Vector<3> v3VelocityFiltered = mVelocityFilter.GetValue();


    // Setting thrust for mode transitions
    if ((mConfig == (ENGAGED | TAKEOFF | TRACKING)) && v3PosInWorld[2] > HTAKEOFF) {
      cout << ", TAKEOFF!!!";
      // The camera has exceeded an altitude of HTAKEOFF meters in takeoff mode
      mConfig = ENGAGED | TRACKING;
      mControl[4] -= 10.;
    }
    else if (((mConfig & (ENGAGED | TAKEOFF)) != mConfigRqst)
          && !((mConfigRqst & TAKEOFF) && ~(mConfig & TAKEOFF)
              && ((mConfig & ENGAGED) || (v3PosInWorld[2] > HTAKEOFF)))) {
      // Requested config is different from current config
      // (and the request isn't to go to takeoff mode from waypoint-acquire-and-hold mode or
      // altitude greater than HTAKEOFF meters)
      if (!(mConfig & (ENGAGED | TAKEOFF)) && (mConfigRqst == ENGAGED)) {
        // Requested waypoint-acquire-and-hold mode (while not in takeoff mode)
        mControl[4] = (double) mHoverGas;
        cout << "HoverGas set to: " << mControl[4] << endl;
      }
      else if (!(mConfig & ENGAGED) && (mConfigRqst == TAKEOFF)) {
        // Requsted takeoff mode
        mControl[4] = 30.;
      }
      mConfig = mConfigRqst | TRACKING;
      // cout << "****" << endl;
      // cout << "  v3PosInWorld:     " << v3PosInWorld << endl;
      // cout << "  v3OffsetInWorld:  " << v3OffsetInWorld << endl;
      // cout << "  mv3Offset:        " << mv3Offset << endl;
      // cout << "  v3OffsetFiltered: " << v3OffsetFiltered << endl;
      // cout << "****" << endl;
    }
    else {
      mConfig |= TRACKING;
    }

    // Control law
    if (mConfig & ENGAGED) {
      // Control is engaged
      if (mConfig & TAKEOFF) {
        // Control is engaged in takeoff mode
        mControl[0] = 0.;
        mControl[1] = 0.;
        mControl[2] = 0.;
        mControl[3] = 0.;
        // Ramp up the throttle
        mControl[4] += 20. * dt;
        if (mControl[4] > 140.) mControl[4] = 140.;
      }
      else {
        // Control is engaged in waypoint-acquire-and-hold mode

        // Pitch conrol law
        mControl[0] = min(max(
            -20. * v3OffsetFiltered[1] + 150. * v3VelocityFiltered[1],
            -30.), 30.);
            // -AUTHORITY), AUTHORITY);
        mControlInt[0] += 0.05 * mControl[0] * dt;
        mControlInt[0] = min(max(mControlInt[0], -20.), 20.); // anti-wind-up
        mControl[0] = min(max(mControl[0] + mControlInt[0], -AUTHORITY), AUTHORITY);

        // Roll control law
        mControl[1] = min(max(
            -20. * v3OffsetFiltered[0] + 150. * v3VelocityFiltered[0],
            -30.), 30.);
            // -AUTHORITY), AUTHORITY);
        mControlInt[1] += 0.05 * mControl[1] * dt;
        mControlInt[1] = min(max(mControlInt[1], -20.), 20.); // anti-wind-up
        mControl[1] = min(max(mControl[1] + mControlInt[1], -AUTHORITY), AUTHORITY);

        // Yaw control law (TBD)

        // Throttle control law
        mControl[3] = min(max(
            13.33 * (2.05 - v3PosInWorld[2]) + 48. * v3VelocityFiltered[2],
            -20.), 20.);
            // -AUTHORITY), AUTHORITY);
        mControl[4] += 0.2 * mControl[3] * dt;
        mControl[4] = min(max(mControl[4], 30.), 192.); // anti-wind-up
      }
    }
    else {
      // Control is not engaged
      mControl[0] = 0.;
      mControl[1] = 0.;
      mControl[2] = 0.;
      mControl[3] = 0.;
      // Reset the inegrators
      mControlInt[0] = 0.;
      mControlInt[1] = 0.;
      mControlInt[2] = 0.;
    }

    mv3PrevPosInWorld = v3PosInWorld;
    mReset = false;
  } // end of if(bHasTracking)
  else {
    // Tracking was lost
    mControl[0] = 0.;
    mControl[1] = 0.;
    mControl[2] = 0.;
    mControl[3] = 0.;
    if (mConfig & TAKEOFF) mControl[4] = 30.;
    mConfig &= ~TRACKING;
    mReset = true;
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

