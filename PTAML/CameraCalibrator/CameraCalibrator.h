// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#ifndef __CAMERACALIBRATOR_H
#define __CAMERACALIBRATOR_H
#include "CalibImage.h"
#include "VideoSource.h"
#include <gvars3/gvars3.h>
#include <vector>
#include "GLWindow2.h"

namespace PTAMM {

class CameraCalibrator
{
public:
  CameraCalibrator(VideoSource* videoSource = NULL);
  void Run();
  
  void CalibrateFromImages(const std::vector<std::string> & filenames);

protected:
  void Reset();
  void HandleFrame(CVD::Image<CVD::byte> imFrame);
  static void MainLoopCallback(void* pvUserData);
  void MainLoopStep();

  GLWindow2 mGLWindow;
  VideoSource* mVideoSource;
  ATANCamera mCamera;
  bool mbDone;

  std::vector<CalibImage> mvCalibImgs;
  void OptimizeOneStep();
  
  bool mbGrabNextFrame;
  GVars3::gvar3<int> mgvnOptimizing;
  GVars3::gvar3<int> mgvnShowImage;
  GVars3::gvar3<int> mgvnDisableDistortion;
  double mdMeanPixelError;

  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
};

}

#endif
