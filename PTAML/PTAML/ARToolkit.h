#ifndef __ARTOOLKIT_H
#define __ARTOOLKIT_H

#include <cvd/image.h>
#include <cvd/byte.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <AR/ar.h>

#include <vector>

namespace PTAMM {

class ARToolkitTracker {
  public:
    ARToolkitTracker()
      : mPatternId(-1)
      , mTrackedMarker(0) {}

    bool Init(const CVD::ImageRef& imSize);
    bool Track(const CVD::Image<CVD::byte> &imFrame, bool bDraw = false);
    void GetMarkerTrans(TooN::SE3<>& markerTransform);
    void GetMarkerCorners(std::vector<TooN::Vector<2> >& corners);

  private:
    int mPatternId;
    ARMarkerInfo* mTrackedMarker;
};

}

#endif
