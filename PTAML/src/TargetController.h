#ifndef __POSITION_HOLD_H
#define __POSITION_HOLD_H

#include "Filters.h"
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <chrono>

namespace PTAMM {

class TargetController {
  public:
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::time_point<Clock> TimePoint;

    void Update(const TooN::SE3<> &se3Pose, bool bHasTracking, const TimePoint& t = Clock::now());

    void SetTarget(const TooN::SE3<> &se3PoseInWorld);

    void RequestConfig(uint8_t ConfigRqst) { mConfigRqst = ConfigRqst; };

    double GetTime() const;

    const TooN::Vector<3>& GetTargetOffset() const { return mv3Offset; }
    const TooN::Vector<3>& GetVelocity() const { return mv3Velocity; }

    const TooN::Vector<3>& GetTargetOffsetFiltered() const { return mOffsetFilter.GetValue(); }
    const TooN::Vector<3>& GetVelocityFiltered() const { return mVelocityFilter.GetValue(); }

    const double* GetControl() const { return mControl; }
    const uint8_t GetConfig() const { return mConfig; }

  private:
    TimePoint mStartTime;
    TimePoint mLastUpdate;
    TooN::Vector<3> mv3TargetPosInWorld;
    TooN::Vector<3> mv3PrevPosInWorld;
    TooN::Vector<3> mv3Offset;
    TooN::Vector<3> mv3Velocity;

    double mControl[5];
    uint8_t mConfig, mConfigRqst;

    MovingAverageFilter<TooN::Vector<3>, 6> mOffsetFilter;
    MovingAverageFilter<TooN::Vector<3>, 6> mVelocityFilter;
};

}

#endif

