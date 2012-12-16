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

// set external control authority to half that of manual control (i.e. can be overridden)
#define AUTHORITY 64.

// height at which the airplane is considered to have taken off (meters)
#define HTAKEOFF 0.2

class TargetController {
  public:
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::time_point<Clock> TimePoint;

    void Update(const TooN::SE3<> &se3Pose, bool bHasTracking, const TimePoint& t = Clock::now());

    void SetTarget(const TooN::SE3<> &se3PoseInWorld);

    void RequestConfig(uint8_t c, int h) { mConfigRqst = c; mHoverGas = h; };
    uint8_t GetConfig() const { return mConfig; }

    const double* GetControl() const { return mControl; }

    double GetTime() const;

    const TooN::Vector<3>& GetTargetOffset() const { return mv3Offset; }
    const TooN::Vector<3>& GetVelocity() const { return mv3Velocity; }

    const TooN::Vector<3>& GetTargetOffsetFiltered() const { return mOffsetFilter.GetValue(); }
    const TooN::Vector<3>& GetVelocityFiltered() const { return mVelocityFilter.GetValue(); }

  private:
    double mControl[5];
    // mControl[0]: pitch command
    // mControl[1]: roll command
    // mControl[2]: yaw command
    // mControl[3]: throttle command (delta from hover)
    // mControl[4]: throttle for hover (integral)

    double mControlInt[3];
    // mControlInt[0]: pitch integral
    // mControlInt[1]: roll integral
    // mControlInt[2]: yaw integral

    TimePoint mStartTime;
    TimePoint mLastUpdate;
    TooN::Vector<3> mv3TargetPosInWorld;
    TooN::Vector<3> mv3PrevPosInWorld;
    TooN::Vector<3> mv3Offset;
    TooN::Vector<3> mv3Velocity;

    uint8_t mConfig, mConfigRqst;
    int mHoverGas;

    bool mReset;

    MovingAverageFilter<TooN::Vector<3>, 6> mOffsetFilter;
    MovingAverageFilter<TooN::Vector<3>, 6> mVelocityFilter;
};

}

#endif

