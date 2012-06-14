#ifndef __SCALE_MARKER_TRACKER_H
#define __SCALE_MARKER_TRACKER_H

#include "ATANCamera.h"

#include <TooN/se3.h>
#include <cvd/image.h>
#include <cvd/byte.h>

namespace PTAMM {

class ARToolkitTracker;

class ScaleMarkerTracker
{
  public:
    ScaleMarkerTracker(const ATANCamera &c, ARToolkitTracker& arTracker);

    bool DetermineScaleFromMarker(const CVD::Image<CVD::byte> &imFrame, const TooN::SE3<> & se3CamFromWorld,
                                  TooN::SE3<> &se3WorldFromNormWorld, double &dScale);

  private:
    ATANCamera mCamera;             // Projection model
    ARToolkitTracker& mARTracker;   // Tracker of markers
};

}

#endif
