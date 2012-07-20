#ifndef __FRONTEND_RENDERER_H
#define __FRONTEND_RENDERER_H

#include "ATANCamera.h"
#include "Frontend.h"

#include <TooN/TooN.h>
#include <TooN/se3.h>

namespace PTAMM {

class FrontendRenderer {
  public:
    FrontendRenderer(const ATANCamera &camera, const FrontendDrawData &drawData)
      : mDrawData(drawData)
      , mCamera(camera)
      , mse3CamFromWorld(drawData.tracker.se3CamFromWorld)
    {
    }

    void Draw();

  private:
    TooN::Vector<2> ProjectPoint(const TooN::SE3<> &se3CamFromWorld,
    		                     const TooN::Vector<3> &v3Point);
    void DrawTrails(const std::vector<std::pair<CVD::ImageRef, CVD::ImageRef>> &vTrails,
                    const std::vector<CVD::ImageRef> &vDeadTrails);

    void DrawCorners(const std::vector<CVD::ImageRef> &vCorners);
    void DrawGrid(const TooN::SE3<> &se3CamFromWorld);
    void DrawMapPoints(const std::vector<std::pair<int, TooN::Vector<2> >> &vMapPoints);
    void DrawMarkerPose(const TooN::SE3<> &se3WorldFromNormWorld);

  private:
    const FrontendDrawData &mDrawData;
    ATANCamera mCamera;
    TooN::SE3<> mse3CamFromWorld;
};

}

#endif
