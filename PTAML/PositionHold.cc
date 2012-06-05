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

  if (dt < 0.00001) {
    return;
  }

  mLastUpdate = t;

  // Calculate the yaw-only rotation transformation
  SO3<> so3Rotation = se3Pose.get_rotation();
  Vector<3> v3Up = makeVector(0, 0, 1);
  SO3<> so3StraightenUp(so3Rotation * v3Up, -v3Up);
  so3Rotation = so3StraightenUp * so3Rotation;

  // Offset calculation
  Vector<3> v3PosInWorld = se3Pose.inverse().get_translation();
  Vector<3> v3OffsetInWorld = mv3TargetPosInWorld - v3PosInWorld;
  mv3Offset = so3Rotation * v3OffsetInWorld;
  mOffsetFilter.Update(mv3Offset);

  // Velocity calculation
  Vector<3> v3DeltaPos = v3PosInWorld - mv3PrevPosInWorld;
  mv3Velocity = so3Rotation * v3DeltaPos * (1.0 / dt);
  mv3PrevPosInWorld = v3PosInWorld;
  mVelocityFilter.Update(mv3Velocity);
}

double PositionHold::GetTime() const
{
  return duration_cast<fseconds>(mLastUpdate - mStartTime).count();
}



}
