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
    mv3TargetPosInWorld[2] = 1.;
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

    mConfig |= TRACKING;

    // Earth frame - x: "North", y: "East", z: down
    // Body frame - x: out the nose, y: out the right wing, z: out the belly
    // PEarth (PTAM Earth) frame - x: "East", y: "North", z: up
    // PCamera (PTAM Camera) frame - z: out the lens, y: down the sensor, x: right-hand rule
    // Heading frame - x: along heading, z: down, y: right-hand rule

    Matrix<3> m33PEarthToPCam = se3Pose.get_rotation().get_matrix();
    const Matrix<3> m33EarthToPEarth = Data(0.,1.,0.,1.,0.,0.,0.,0.,-1.);
    const Matrix<3> m33PCamToBody = Data(0.,-1.,0.,1.,0.,0.,0.,0.,1.);
    Matrix<3> m33EarthToBody = m33PCamToBody * m33PEarthToPCam * m33EarthToPEarth;

    if (mReset) {
      mPhiRL.Reset();
      mThetaRL.Reset();
      mAltitudeRL.Reset();
    }
    mPhiRL.Update(atan2(m33EarthToBody(1,2), m33EarthToBody(2,2)), M_PI, dt);  // Phi (roll angle)
    mThetaRL.Update(-asin(m33EarthToBody(0,2)), M_PI, dt);  // Theta (pitch angle)
    mv3EulerAngles[0] = mPhiRL.GetValue();
    mv3EulerAngles[1] = mThetaRL.GetValue();
    mv3EulerAngles[2] = atan2(m33EarthToBody(0,1), m33EarthToBody(0,0));  // Psi (heading angle)

    double cPsi = cos(mv3EulerAngles[2]);
    double sPsi = sin(mv3EulerAngles[2]);
    Matrix<3> m33PEarthToHeading = Data(sPsi,cPsi,0.,cPsi,-sPsi,0.,0.,0.,-1.);

    mv3PosInWorld = se3Pose.inverse().get_translation();
    mAltitudeRL.Update(mv3PosInWorld[2], 2.5, dt);
    mv3PosInWorld[2] = mAltitudeRL.GetValue();

    if (mHoldCurrentLocation) {
      mv3TargetPosInWorld[0] = mv3PosInWorld[0];
      mv3TargetPosInWorld[1] = mv3PosInWorld[1];
      mOffsetFilter.Reset();
      mHoldCurrentLocation = false;
    }

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

    // Positional error (vector to target) in heading frame
    mv3Offset = m33PEarthToHeading * (mv3TargetPosInWorld - mv3PosInWorld);
    if (mReset) mOffsetFilter.Reset();
    mOffsetFilter.Update(mv3Offset);
    Vector<3> v3OffsetFiltered = mOffsetFilter.GetValue();



    // Monitor for takeoff
    if ((mConfig == (ENGAGED | TAKEOFF | TRACKING)) && mv3PosInWorld[2] > HTAKEOFF) {
      // The camera has exceeded an altitude of HTAKEOFF meters in takeoff mode
      cout << ", TAKEOFF!!!";
      mConfig = ENGAGED | TRACKING;
      mv3TargetPosInWorld[0] = mv3PosInWorld[0];
      mv3TargetPosInWorld[1] = mv3PosInWorld[1];
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
            30. * min(max(v3OffsetFiltered[1], -1.), 1.)
            - 24.8484 * v3VelocityFiltered[1],
            -50.), 50.);

        // Pitch control law
        mControl[1] = min(max(
            30. * min(max(v3OffsetFiltered[0], -1.), 1.)
            - 24.8484 * v3VelocityFiltered[0],
            -50.), 50.);


        // Yaw control law
        // TODO: actually do yaw control
        mControl[2] = 0;

        // Thrust control law
        mControl[3] = min(max(
            -8.616 * min(max(v3OffsetFiltered[2], -1.), 1.)
            + 6.096 * v3VelocityFiltered[2],
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

    mConfig &= ~TRACKING;
    mReset = true;
  }
}

void TargetController::RequestConfig(uint8_t nRequest)
{
  uint8_t nEngaged = mConfig & (ENGAGED | TAKEOFF);

  if (nEngaged != nRequest) {
    cout << " >> Config " << (int)nRequest << " requested from config "
        << (int)nEngaged << endl;
    // Requested mode is different from engaged mode
    if (!nEngaged) {
      // Currently not engaged in any mode
      if (nRequest == ENGAGED) {
        // Control engagement requested
        mControl[4] = 0.;  // Clear the hover thrust
        mConfig |= ENGAGED;
      } else if (nRequest == TAKEOFF && -mv3PosInWorld[2] < HTAKEOFF) {
        // Takeoff mode requested
        mControl[4] = -30.;  // Set hover thrust to idle
        mConfig |= TAKEOFF;
      }
      // NOTE: Intentionally ignoring case where nRequest == (ENGAGED | TAKEOFF)
    } else if (nRequest == (ENGAGED | TAKEOFF)) {
      if (nEngaged == TAKEOFF && -mv3PosInWorld[2] < HTAKEOFF)
        mConfig |= ENGAGED;
      // NOTE: Intentionally ignoring case where nEngaged == ENGAGED
    } else {
      mConfig &= !(ENGAGED | TAKEOFF);
    }
  }
}

void TargetController::SetTarget(TooN::Vector<3> v3PosInWorld)
{
  mv3TargetPosInWorld = v3PosInWorld;
  mOffsetFilter.Reset();
}

void TargetController::SetTargetLocation(TooN::Vector<2> v2LocInWorld) {
  mv3TargetPosInWorld[0] = v2LocInWorld[0];
  mv3TargetPosInWorld[1] = v2LocInWorld[1];
  mOffsetFilter.Reset();
}

double TargetController::GetTime() const
{
  return duration_cast<RealSeconds>(mLastUpdate - mStartTime).count();
}

}

