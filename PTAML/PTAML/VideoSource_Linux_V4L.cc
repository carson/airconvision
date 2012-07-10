// Copyright 2008 Isis Innovation Limited
#include "VideoSource_Linux_V4L.h"
#include <cvd/colourspace_convert.h>
#include <gvars3/instances.h>

namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;

VideoSource_Linux_V4L::VideoSource_Linux_V4L(const std::string &sName)
{
  cout << "  VideoSource_Linux_V4L: Opening video source..." << endl;
  string sDevice = GV3::get<string>(sName + ".V4LDevice", "/dev/video0");
  ImageRef irSize = GV3::get<ImageRef>(sName + ".Resolution", ImageRef(640, 480));
  int nFrameRate = GV3::get<int>(sName + ".Framerate", 60);
  mVideoBuffer = new V4LBuffer<yuv422>(sDevice, irSize, -1, false, nFrameRate);
  mirSize = mVideoBuffer->size();
  cout << "  V4L initialized." << endl;
}

void VideoSource_Linux_V4L::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
  VideoFrame<yuv422> *pVidFrame = mVideoBuffer->get_frame();
  convert_image(*pVidFrame, imBW);
  convert_image(*pVidFrame, imRGB);
  mVideoBuffer->put_frame(pVidFrame);

  while (mVideoBuffer->pendingFrame()) {
    VideoFrame<yuv422> *pVidFrame = mVideoBuffer->get_frame();
    convert_image(*pVidFrame, imBW);
    convert_image(*pVidFrame, imRGB);
    mVideoBuffer->put_frame(pVidFrame);
  }
}

}

