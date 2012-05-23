#include "PositionHold.h"

namespace PTAMM {

using namespace TooN;
using namespace std::chrono;

typedef duration<double, std::ratio<1>> fseconds;

void PositionHold::Init(const SE3<> &se3Pose, const TimePoint& t)
{
  mv3PrevPosInWorld = mv3TargetPosInWorld = se3Pose.inverse().get_translation();
  mStartTime = mLastUpdate = t;
}

void PositionHold::Update(const SE3<> &se3Pose, const TimePoint& t)
{
  double dt = duration_cast<fseconds>(t - mLastUpdate).count(); // delta time in seconds
  mLastUpdate = t;

  Vector<3> v3PosInWorld = se3Pose.inverse().get_translation();
  Vector<3> v3OffsetInWorld = mv3TargetPosInWorld - v3PosInWorld;
  mv3Offset = se3Pose.get_rotation() * v3OffsetInWorld;
  mOffsetFilter.Update(mv3Offset);

  Vector<3> v3DeltaPos = v3PosInWorld - mv3PrevPosInWorld;
  mv3Velocity = se3Pose.get_rotation() * v3DeltaPos * (1.0 / dt);
  mv3PrevPosInWorld = v3PosInWorld;
  mVelocityFilter.Update(mv3Velocity);
}

double PositionHold::GetTime() const
{
  return duration_cast<fseconds>(mLastUpdate - mStartTime).count();
}



}
