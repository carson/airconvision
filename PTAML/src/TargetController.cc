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
    mOffsetFilter.Reset(); mOffsetFilter.Update(mv3Offset);
    mVelocityFilter.Reset(); mVelocityFilter.Update(mv3Velocity);
    memset(mControl, 0, sizeof(double)*4);
    memset(mOffsetInt, 0, sizeof(double)*3);
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

    Matrix<3> m33PEarthToPCam = se3Pose.get_rotation().get_matrix();
    const Matrix<3> m33EarthToPEarth = Data(0,1,0,1,0,0,0,0,-1);
    const Matrix<3> m33PCamToBody = Data(0,-1,0,1,0,0,0,0,1);
    Matrix<3> m33EarthToBody = m33PCamToBody * m33PEarthToPCam * m33EarthToPEarth;

    mv3EulerAngles[0] = atan2(m33EarthToBody(1,2), m33EarthToBody(2,2));  // Phi
    mv3EulerAngles[1] = -asin(m33EarthToBody(0,2));  // Theta
    mv3EulerAngles[2] = atan2(m33EarthToBody(0,1), m33EarthToBody(0,0));  // Psi

    // Conversion to heading frame
    double cPsi = cos(mv3EulerAngles[2]);
    double sPsi = sin(mv3EulerAngles[2]);
    Matrix<3> m33PEarthToHeading = Data(sPsi,cPsi,0,cPsi,-sPsi,0,0,0,-1);

    mv3PosInWorld = se3Pose.inverse().get_translation();

    // Velocity calculation in heading frame
    if (mReset) {
      // TODO: Find out of this causes a memory leak
      mv3Velocity = Zeros;
      mVelocityFilter.Reset();
    }
    else {
      Vector<3> v3DeltaPos = mv3PosInWorld - mv3PrevPosInWorld;
      mv3Velocity = m33PEarthToHeading * v3DeltaPos * (1.0 / dt);
    }
    mVelocityFilter.Update(mv3Velocity);
    Vector<3> v3VelocityFiltered = mVelocityFilter.GetValue();

    mv3TargetPosInWorld[2] = 1.;

    // Positional error (vector to target) in heading frame
    mv3Offset = m33PEarthToHeading * (mv3TargetPosInWorld - mv3PosInWorld);
    if (mReset) mOffsetFilter.Reset();
    mOffsetFilter.Update(mv3Offset);
    Vector<3> v3OffsetFiltered = mOffsetFilter.GetValue();




    // Setting thrust for mode transitions
    if ((mConfig == (ENGAGED | TAKEOFF | TRACKING)) && mv3PosInWorld[2] > HTAKEOFF) {
      cout << ", TAKEOFF!!!";
      // The camera has exceeded an altitude of HTAKEOFF meters in takeoff mode
      mConfig = ENGAGED | TRACKING;
    }
    else if (((mConfig & (ENGAGED | TAKEOFF)) != mConfigRqst)
          && !((mConfigRqst & TAKEOFF) && ~(mConfig & TAKEOFF)
              && ((mConfig & ENGAGED) || (mv3PosInWorld[2] > HTAKEOFF)))) {
      // Requested config is different from current config
      // (and the request isn't to go to takeoff mode from waypoint-acquire-and-hold mode or
      // altitude greater than HTAKEOFF meters)
      if (!(mConfig & ENGAGED) && (mConfigRqst == TAKEOFF)) {
        // Requsted takeoff mode
        mControl[4] = -30.;  // Set thrust integral to idle (-30 N)
      }
      mConfig = mConfigRqst | TRACKING;
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
        mControl[4] += 10. * dt;
        if (mControl[4] > 10.) mControl[4] = 10.;
      }
      else {
        // Control is engaged in waypoint-acquire-and-hold mode


        // Roll control law
        mControl[0] = min(max(
            -30. * min(max(v3OffsetFiltered[1], -1., 1.)
            - 24.8484 * v3VelocityFiltered[1],
            -50.), 50.);

        // Pitch control law
        mControl[1] = min(max(
            -30. * min(max(v3OffsetFiltered[0], -1., 1.)
            - 24.8484 * v3VelocityFiltered[0],
            -50.), 50.);


        // Yaw control law
        // TODO: actually do yaw control
        mControl[2] = 0;

        // Thrust control law
        mControl[3] = min(max(
            -8.616. * min(max(v3OffsetFiltered[2], -1., 1.)
            - 6.096 * v3VelocityFiltered[2],
            -15.), 15.);
        mControl[4] += 0.2 * mControl[3] * dt;
        mControl[4] = min(max(mControl[4], -10.), 10.);
      }
    }
    else {
      // Control is not engaged
      mControl[0] = 0.;
      mControl[1] = 0.;
      mControl[2] = 0.;
      mControl[3] = 0.;
      // Reset the integrators
      mOffsetInt[0] = 0.;
      mOffsetInt[1] = 0.;
      mOffsetInt[2] = 0.;
    }

    mv3PrevPosInWorld = mv3PosInWorld;
    mReset = false;
  } // end of if(bHasTracking)
  else {
    // Tracking was lost
    mControl[0] = 0.;
    mControl[1] = 0.;
    mControl[2] = 0.;
    mControl[3] = 0.;
    // Kill a takeoff attempt
    if (mConfig & TAKEOFF) mControl[4] = 7.5;

    mConfig &= ~TRACKING;
    mReset = true;
  }
}

void TargetController::SetTarget(TooN::Vector<3> v3PosInWorld)
{
  mv3TargetPosInWorld = v3PosInWorld;
  mOffsetFilter.Reset();
}

double TargetController::GetTime() const
{
  return duration_cast<RealSeconds>(mLastUpdate - mStartTime).count();
}

}

