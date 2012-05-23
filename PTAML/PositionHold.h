#ifndef __POSITION_HOLD_H
#define __POSITION_HOLD_H

#include "Filters.h"
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <chrono>

namespace PTAMM {

using std::chrono::time_point;
using std::chrono::high_resolution_clock;

class PositionHold {
  public:

    typedef high_resolution_clock Clock;
    typedef time_point<Clock> TimePoint;

    void Init(const TooN::SE3<> &se3Pose, const TimePoint& t = Clock::now());
    void Update(const TooN::SE3<> &se3Pose, const TimePoint& t = Clock::now());

    double GetTime() const;
    const TooN::Vector<3>& GetTargetOffset() const { return mv3Offset; }
    const TooN::Vector<3>& GetVelocity() const { return mv3Velocity; }

    const TooN::Vector<3>& GetTargetOffsetFiltered() const { return mOffsetFilter.GetValue(); }
    const TooN::Vector<3>& GetVelocityFiltered() const { return mVelocityFilter.GetValue(); }

  private:
    TimePoint mStartTime;
    TimePoint mLastUpdate;
    TooN::Vector<3> mv3TargetPosInWorld;
    TooN::Vector<3> mv3PrevPosInWorld;
    TooN::Vector<3> mv3Offset;
    TooN::Vector<3> mv3Velocity;

    MovingAverageFilter<TooN::Vector<3>, 6> mOffsetFilter;
    MovingAverageFilter<TooN::Vector<3>, 6> mVelocityFilter;
};

}

#endif
