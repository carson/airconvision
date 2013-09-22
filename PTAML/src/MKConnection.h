#ifndef __MKCONNECTION_H
#define __MKCONNECTION_H

#include "MKProtocol.h"

#include <cstdarg>
#include <functional>

#include <TooN/TooN.h>

namespace PTAMM {

struct PTAMToMK_t {
  float roll_cmd;  // rad/s^2
  float pitch_cmd;  // rad/s^2
  float yaw_cmd;  // rad/s^2
  int16_t transient_thrust;  // Q9 N
  int16_t hover_thrust;  // Q9 N
  float euler_angles[3];  // rad
  uint8_t status;  // 0x1 = Engaged, 0x2 = Takeoff Mode, 0x4 = Tracking
} __attribute__((packed));

struct MKToPTAM_t {
  float single[3];
  int16_t int16[6];
  uint8_t request;  // 0x1 = Engaged, 0x2 = Takeoff Mode
  uint8_t altitude;  // Q6 m [0, 4]
  uint8_t count;
} __attribute__((packed));

struct MKData_t {
  int16_t int16[9];
} __attribute__((packed));

struct MKDebug_t {
  uint8_t status[2];
  int16_t int16[32];
} __attribute__((packed));

typedef struct {
  int32_t Longitude;  // in 1E-7 deg
  int32_t Latitude; // in 1E-7 deg
  int32_t Altitude; // in mm
  uint8_t Status;// validity of data
} __attribute__((packed)) GPS_Pos_t;

typedef struct {
  uint16_t Distance;         // distance to target in dm
  int16_t Bearing;          // course to target in deg
} __attribute__((packed)) GPS_PosDev_t;

typedef struct {
  uint8_t Version;           // version of the data structure
  GPS_Pos_t CurrentPosition;    // see ubx.h for details
  GPS_Pos_t TargetPosition;
  GPS_PosDev_t TargetPositionDeviation;
  GPS_Pos_t HomePosition;
  GPS_PosDev_t HomePositionDeviation;
  uint8_t  WaypointIndex;        // index of current waypoints running from 0 to WaypointNumber-1
  uint8_t  WaypointNumber;       // number of stored waypoints
  uint8_t  SatsInUse;          // number of satellites used for position solution
  int16_t Altimeter;          // hight according to air pressure
  int16_t Variometer;         // climb(+) and sink(-) rate
  uint16_t FlyingTime;         // in seconds
  uint8_t  UBat;           // Battery Voltage in 0.1 Volts
  uint16_t GroundSpeed;        // speed over ground in cm/s (2D)
  int16_t Heading;          // current flight direction in ° as angle to north
  int16_t CompassHeading;       // current compass value in °
  int8_t  AngleNick;          // current Nick angle in 1°
  int8_t  AngleRoll;          // current Rick angle in 1°
  uint8_t  RC_Quality;         // RC_Quality
  uint8_t  FCStatusFlags;        // Flags from FC
  uint8_t  NCFlags;          // Flags from NC
  uint8_t  Errorcode;          // 0 --> okay
  uint8_t  OperatingRadius;      // current operation radius around the Home Position in m
  int16_t TopSpeed;         // velocity in vertical direction in cm/s
  uint8_t  TargetHoldTime;       // time in s to stay at the given target, counts down to 0 if target has been reached
  uint8_t  FCStatusFlags2;       // StatusFlags2 (since version 5 added)
  int16_t SetpointAltitude;     // setpoint for altitude
  uint8_t  Gas;            // for future use
  uint16_t Current;          // actual current in 0.1A steps
  uint16_t UsedCapacity;       // used capacity in mAh
} __attribute__((packed)) MKNavi_t;

class MKConnection {
  public:
    typedef std::function<void()> PositionHoldCallback;
    typedef std::function<void(const MKToPTAM_t&)> MKToPTAMCallback;
    typedef std::function<void(const MKData_t&)> MKDataCallback;
    typedef std::function<void(const MKDebug_t&)> MKDebugCallback;
    typedef std::function<void(const MKNavi_t&)> MKNaviCallback;

    MKConnection() : mOpen(false) {}
    MKConnection(int comPortId, int baudrate);
    ~MKConnection();

    operator bool() const { return mOpen; }

    void ProcessIncoming();

    void SendPTAMToMK(const double *control, const TooN::Vector<3> &eulerAngles,
        uint8_t status);
    void SendNewTargetNotice();

    void RequestMKDebugInterval(uint8_t interval);
    void RequestMKNaviInterval(uint8_t interval);

    void SetMKToPTAMCallback(const MKToPTAMCallback &callback) {
      mMKToPTAMCallback = callback;
    }
    void SetMKDataCallback(const MKDataCallback &callback) {
      mMKDataCallback = callback;
    }
    void SetMKDebugCallback(const MKDebugCallback &callback) {
      mMKDebugCallback = callback;
    }
    void SetMKNaviCallback(const MKNaviCallback &callback) {
      mMKNaviCallback = callback;
    }
    void SetPositionHoldCallback(const PositionHoldCallback &callback) {
      mPositionHoldCallback = callback;
    }

  private:
    void HandleMKToPTAM(const SerialMsg_t& msg);
    void HandleMKData(const SerialMsg_t& msg);
    void HandleMKDebug(const SerialMsg_t& msg);
    void HandleMKNavi(const SerialMsg_t& msg);

    void SendData(uint8_t cmdID, uint8_t dataLength, ...);

    bool mOpen;
    int mComPortId;

    enum { TX_BUFFER_SIZE = 4096 };
    uint8_t mTxBufferData[TX_BUFFER_SIZE];
    enum { RX_BUFFER_SIZE = 4096 };
    uint8_t mRxBufferData[RX_BUFFER_SIZE];

    Buffer_t mRxBuffer;

    // Callbacks
    MKToPTAMCallback mMKToPTAMCallback;
    MKDataCallback mMKDataCallback;
    MKDebugCallback mMKDebugCallback;
    MKNaviCallback mMKNaviCallback;
    PositionHoldCallback mPositionHoldCallback;
};

}

#endif
