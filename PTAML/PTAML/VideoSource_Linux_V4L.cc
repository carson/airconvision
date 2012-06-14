// Copyright 2008 Isis Innovation Limited
#include "VideoSource_Linux_V4L.h"
#include <cvd/colourspace_convert.h>
#include <gvars3/instances.h>

namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;

VideoSource_Linux_V4L::VideoSource_Linux_V4L()
{
  cout << "  VideoSource_Linux_V4L: Opening video source..." << endl;
  string QuickCamFile = GV3::get<string>("VideoSource.V4LDevice", "/dev/video0");
  ImageRef irSize = GV3::get<ImageRef>("VideoSource.Resolution", ImageRef(640, 480));
  int nFrameRate = GV3::get<int>("VideoSource.Framerate", 60);
  mVideoBuffer = new V4LBuffer<yuv422>(QuickCamFile, irSize, -1, false, nFrameRate);
  mirSize = mVideoBuffer->size();
  cout << "  V4L initialized." << endl;
}

void VideoSource_Linux_V4L::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
  VideoFrame<yuv422> *pVidFrame = mVideoBuffer->get_frame();
  convert_image(*pVidFrame, imBW);
  convert_image(*pVidFrame, imRGB);
  mVideoBuffer->put_frame(pVidFrame);
}

}

