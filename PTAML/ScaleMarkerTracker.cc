#include "ScaleMarkerTracker.h"
#include "ARToolkit.h"
#include "Utils.h"

#include <TooN/TooN.h>

#include <vector>

using namespace std;
using namespace TooN;
using namespace CVD;
using namespace GVars3;

namespace PTAMM {

ScaleMarkerTracker::ScaleMarkerTracker(const ATANCamera &c, ARToolkitTracker& arTracker)
  : mCamera(c)
  , mARTracker(arTracker)
{
}

bool ScaleMarkerTracker::DetermineScaleFromMarker(const Image<CVD::byte> &imFrame,
                                                  const SE3<> & se3CamFromWorld,
                                                  SE3<> &se3WorldFromNormWorld,
                                                  double &dScale)
{
  if (mARTracker.Track(imFrame, false)) {
    // Get corner points of the marker in screen space
    std::vector<Vector<2> > imPts;
    mARTracker.GetMarkerCorners(imPts);

    // Project the corners onto the ground plane to get
    // the coordinates in the PTAM world space
    std::vector<Vector<3> > pointsOnPlane;
    for (auto it = imPts.begin(); it != imPts.end(); ++it) {
      Vector<3> pointOnPlane;
      if (PickPointOnGround(mCamera, se3CamFromWorld, *it, pointOnPlane)) {
        pointsOnPlane.push_back(pointOnPlane);
      }
    }

    // Abort if not all corners were found
    if (pointsOnPlane.size() != 4) {
      return false;
    }

    // Determine scale and origin
    Vector<3> origin = Zeros;
    double edgeLengthSum = 0;
    Vector<3> prevPoint = pointsOnPlane[3];
    for (auto it = pointsOnPlane.begin(); it != pointsOnPlane.end(); ++it) {
      origin += *it;
      double edgeLength = norm(*it - prevPoint);
      edgeLengthSum += edgeLength;
      prevPoint = *it;
    }

    // origin is now the sum of all the corner points, so the marker center is the average
    origin *= 0.25;

    Vector<3> xAxis = pointsOnPlane[1] + pointsOnPlane[2] - pointsOnPlane[0] - pointsOnPlane[3];
    Vector<3> yAxis = pointsOnPlane[0] + pointsOnPlane[1] - pointsOnPlane[2] - pointsOnPlane[3];
    Vector<3> zAxis = xAxis ^ yAxis;
    yAxis = zAxis ^ xAxis;

    normalize(xAxis);
    normalize(yAxis);
    normalize(zAxis);

    Matrix<3,3> rot;
    rot[0] = xAxis;
    rot[1] = yAxis;
    rot[2] = zAxis;

    // The misalignment
    se3WorldFromNormWorld = SE3<>(SO3<>(rot.T()), origin);

    // The scale
    static gvar3<double> gvdMarkerSize("Marker.Size", 0.08, SILENT); // Size of the marker in meters

    dScale = (4.0 * *gvdMarkerSize) / edgeLengthSum;

    return true;
  }

  return false;
}

}
