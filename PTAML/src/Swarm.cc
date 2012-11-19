#include "Swarm.h"

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "rs232.h"

#include "MathUtils.h"
#include "Timing.h"

#include <gvars3/gvars3.h>

#include <iostream>

using namespace GVars3;
using namespace TooN;
using namespace std;

namespace PTAMM {

SwarmLab::SwarmLab()
  : mbDone(false)
  , mbOpen(false)
  , mtpPoseTime()
  , mbPoseUpdated(false)
{
  // Read COM port settings
  mnComPortId = GV3::get<int>("SwarmLab.ComPortId", 17, SILENT); // 17 is /dev/ttyUSB1
  int nBaudrate = GV3::get<int>("SwarmLab.ComPortBaudrate", "38400", SILENT);

  if (OpenComport(mnComPortId, nBaudrate) == 0) {
    mbOpen = true;
  }

  if (!mbOpen) {
    cerr << "Failed to open COM port #" << mnComPortId << " @ " << nBaudrate << endl;
  }
}

void SwarmLab::operator()()
{
  RateLimiter rateLimiter;
  while (!mbDone && mbOpen) {
    {
      std::unique_lock<std::mutex> lock(mMutex);
      if (mbPoseUpdated) {
        SendPosePacket();
        mbPoseUpdated = false;
      }
    }

    rateLimiter.Limit(100.0); // Limit to 100 Hz
  }
}

void SwarmLab::SendPosePacket()
{
  // Build the packet
  struct PoseInfoPacket_t {
    uint8_t Header;
    uint64_t Time; // In microseconds since epoch
    float Position[3];
    float Yaw;
    float Pitch;
    float Roll;
    uint16_t Checksum;
  };

  SE3<> se3WorldPose = mse3CurrentPose.inverse();
  Vector<3> v3Pos = se3WorldPose.get_translation();
  Vector<3> v3YawPitchRoll = So3ToYawPitchRoll(se3WorldPose.get_rotation());

  auto timestamp = std::chrono::duration_cast<
      std::chrono::microseconds>(mtpPoseTime.time_since_epoch()).count();

  PoseInfoPacket_t packet;
  packet.Header = 0xE5;
  packet.Time = timestamp;
  packet.Position[0] = v3Pos[0];
  packet.Position[1] = v3Pos[1];
  packet.Position[2] = v3Pos[2];
  packet.Yaw = v3YawPitchRoll[0];
  packet.Pitch = v3YawPitchRoll[1];
  packet.Roll = v3YawPitchRoll[2];
  packet.Checksum = 0;

  packet.Checksum = Checksum(reinterpret_cast<uint8_t*>(&packet),
                             sizeof(packet));

  SendBuffer(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
  //cout << packet.Time << " " << v3Pos << "  " << v3YawPitchRoll << "  cs: " << packet.Checksum << endl;
}

uint16_t SwarmLab::Checksum(const uint8_t* data, size_t length) const
{
  uint16_t cs = 0;

  for (size_t i = 0; i < length; ++i) {
    cs += data[i];
  }

  return cs;
}

void SwarmLab::SendBuffer(uint8_t* data, int length)
{
  while (length > 0) {
    int sent = SendBuf(mnComPortId, data, length);
    length -= sent;
    data += sent;
  }
}

void SwarmLab::UpdatePose(const TooN::SE3<> &se3Pose, bool bHasTracking,
                          const HiResTimePoint &tpTime)
{
  if (bHasTracking) {
    std::unique_lock<std::mutex> lock(mMutex);
    mse3CurrentPose = se3Pose;
    mtpPoseTime = tpTime;
    mbPoseUpdated = true;
  }
}

}
