#ifndef __POSITION_HOLD_H
#define __POSITION_HOLD_H

#include "Filters.h"
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <chrono>

namespace PTAMM {

// mode specifiers
enum {
  ENGAGED    = 1<<0,
  TAKEOFF    = 1<<1,
  TRACKING   = 1<<2,
  ON_GROUND  = 1<<3,
  EXPERIMENT = 1<<4,
};

// experiment mode specifiers
enum {
  EXP_OFF,
  EXP_TAKEOFF,
  EXP_WAYPOINT,
  EXP_LANDING,
};

// height at which the airplane is considered to have taken off (meters)
#define HTAKEOFF 0.02

class TargetController {
  public:
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::time_point<Clock> TimePoint;

    void Update(const TooN::SE3<> &se3Pose, bool bHasTracking, const TimePoint& t = Clock::now());

    void SetTarget(TooN::Vector<3> v3PosInWorld);
    void SetTargetLocation(TooN::Vector<2> v2LocInWorld);
    void HoldCurrentLocation(void) { mHoldCurrentLocation = true; }
    void SetTargetAltitude(double h);
    void RequestConfig(uint8_t nRequest);

    // To access private data
    const TooN::Vector<3>& GetPosInWorld() const { return mv3PosInWorld; }
    const TooN::Vector<3>& GetEulerAngles() const { return mv3EulerAngles; }
    const TooN::Vector<3>& GetTargetOffset() const { return mv3Offset; }
    const TooN::Vector<3>& GetVelocity() const { return mv3Velocity; }
    const TooN::Vector<3>& GetTarget() const { return mv3TargetPosInWorld; }
    double GetTargetAltitude() { return mv3TargetPosInWorld[2]; }
    const double(& GetControl() const)[5] { return mControl; }
    uint8_t GetConfig() const { return mConfig; }
    uint8_t GetExperimentMode() const { return mExperimentMode; }
    TimePoint GetTime() const;


  private:
    double mControl[5];
    // mControl[0]: roll command
    // mControl[1]: pitch command
    // mControl[2]: yaw command
    // mControl[3]: transient thrust command (delta from hover)
    // mControl[4]: thrust for hover (integral, delta from thrust stick)

    double mOffsetInt[2];
    // mOffsetInt[0]: x offset integral
    // mOffsetInt[1]: y offset integral

    TimePoint mStartTime;
    TimePoint mLastUpdate;
    TooN::Vector<3> mv3PosInWorld;
    TooN::Vector<3> mv3TargetPosInWorld;
    TooN::Vector<3> mv3PrevPosInWorld;
    TooN::Vector<3> mv3Offset;
    TooN::Vector<3> mv3Velocity;
    TooN::Vector<3> mv3EulerAngles;

    uint8_t mConfig, mConfigRqst, mExperimentMode;
    double mAltitudeOffset;

    bool mReset;
    bool mHoldCurrentLocation;

    MovingAverage<TooN::Vector<3>, 4> mVelocityFilter;
    MovingAverage<TooN::Vector<3>, 4> mPositionFilter;
    RateLimit<double> mAltitudeRL;
    RateLimit<double> mPhiRL;
    RateLimit<double> mThetaRL;
};

}

#endif

