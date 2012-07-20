#ifndef __PATH_CONTROLLER_H
#define __PATH_CONTROLLER_H

#include <TooN/se3.h>
#include <chrono>

namespace PTAMM {

class PathController {
  public:
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::time_point<Clock> TimePoint;

    PathController() {}

    void Update(const TooN::SE3<> &se3Pose, const TimePoint& t = Clock::now()) {}

    void AddWaypoint(const TooN::SE3<> &se3PoseInWorld) {}
    void ClearWaypoints() {}

    void Start() {}
};

}

#endif
