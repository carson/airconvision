#ifndef __POSITION_HOLD_H
#define __POSITION_HOLD_H

#include "Filters.h"
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <chrono>

namespace PTAMM {

// mode specifiers
#define ENGAGED  0x01
#define TAKEOFF  0x02
#define TRACKING 0x04

// height at which the airplane is considered to have taken off (meters)
#define HTAKEOFF 0.2

class TargetController {
  public:
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::time_point<Clock> TimePoint;

    void Update(const TooN::SE3<> &se3Pose, bool bHasTracking, const TimePoint& t = Clock::now());

    void SetTarget(TooN::Vector<3> v3PosInWorld);
    void SetTargetLocation(TooN::Vector<2> v2LocInWorld);
    void HoldCurrentLocation(void) { mHoldCurrentLocation = true; }
    void SetTargetAltitude(double altitude) { mv3TargetPosInWorld[2] = -altitude; }

    void RequestConfig(uint8_t nRequest);
    uint8_t GetConfig() const { return mConfig; }
    const double* GetControl() const { return mControl; }
    double GetTime() const;

    const TooN::Vector<3>& GetPosInWorld() const { return mv3PosInWorld; }
    const TooN::Vector<3>& GetEulerAngles() const { return mv3EulerAngles; }

    const TooN::Vector<3>& GetTargetOffset() const { return mOffsetFilter.GetValue(); }
    const TooN::Vector<3>& GetVelocity() const { return mVelocityFilter.GetValue(); }

    double GetTargetAltitude() { return mv3TargetPosInWorld[2]; }


  private:
    double mControl[5];
    // mControl[0]: roll command
    // mControl[1]: pitch command
    // mControl[2]: yaw command
    // mControl[3]: transient thrust command (delta from hover)
    // mControl[4]: thrust for hover (integral, delta from thrust stick)

    double mOffsetInt[3];
    // mOffsetInt[0]: x integral
    // mOffsetInt[1]: y integral
    // mOffsetInt[2]: z integral

    TimePoint mStartTime;
    TimePoint mLastUpdate;
    TooN::Vector<3> mv3PosInWorld;
    TooN::Vector<3> mv3TargetPosInWorld;
    TooN::Vector<3> mv3PrevPosInWorld;
    TooN::Vector<3> mv3Offset;
    TooN::Vector<3> mv3Velocity;
    TooN::Vector<3> mv3EulerAngles;

    uint8_t mConfig, mConfigRqst;
    int mHoverGas;

    bool mReset;
    bool mHoldCurrentLocation;

    MovingAverageFilter<TooN::Vector<3>, 4> mOffsetFilter;
    MovingAverageFilter<TooN::Vector<3>, 4> mVelocityFilter;
};

}

#endif

