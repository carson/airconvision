// Copyright 2008 Isis Innovation Limited
#include "VideoSource_Linux_V4L.h"
#include <cvd/colourspace_convert.h>
#include <gvars3/instances.h>
#include <flycapture/FlyCapture2.h>
#include <flycapture/FlyCapture2GUI.h>

#define V4LorFLYCAP 0 // 0:FLYCAP 1:V4L

#if V4LorFLYCAP
#else

#define VIDEO_W 640
#define VIDEO_H 510

static void PrintErrorAndExit(FlyCapture2::Error error)
{
	error.PrintErrorTrace();
	exit(0);
}
#endif

namespace PTAMM {

#if V4LorFLYCAP
#else
static FlyCapture2::Camera flu3cam;
#endif

using namespace CVD;
using namespace std;
using namespace GVars3;

VideoSource_Linux_V4L::VideoSource_Linux_V4L(const std::string &sName)
{
#if V4LorFLYCAP
  cout << "  VideoSource_Linux_V4L: Opening video source..." << endl;
  string sDevice = GV3::get<string>(sName + ".V4LDevice", "/dev/video0");
  ImageRef irSize = GV3::get<ImageRef>(sName + ".Resolution", ImageRef(640, 480));
  int nFrameRate = GV3::get<int>(sName + ".Framerate", 60);
  mVideoBuffer = new V4LBuffer<yuv422>(sDevice, irSize, -1, false, nFrameRate);
  mirSize = mVideoBuffer->size();
  cout << "  V4L initialized." << endl;
#else
	FlyCapture2::Error flu3error;
	FlyCapture2::BusManager flu3busMgr;
	FlyCapture2::PGRGuid flu3guid;
	
	flu3error=flu3busMgr.GetCameraFromIndex(0,&flu3guid);
	if(flu3error!=FlyCapture2::PGRERROR_OK) PrintErrorAndExit(flu3error);
	
	flu3error=flu3cam.Connect(&flu3guid);
	if(flu3error!=FlyCapture2::PGRERROR_OK) PrintErrorAndExit(flu3error);
	
	mirSize = ImageRef(VIDEO_W, VIDEO_H);
	
	flu3error=flu3cam.StartCapture();
	if(flu3error!=FlyCapture2::PGRERROR_OK) PrintErrorAndExit(flu3error);
#endif
}

VideoSource_Linux_V4L::~VideoSource_Linux_V4L()
{
#if V4LorFLYCAP
#else
	flu3cam.StopCapture();
	flu3cam.Disconnect();
#endif
}



void VideoSource_Linux_V4L::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{

#if V4LorFLYCAP
  VideoFrame<yuv422> *pVidFrame = mVideoBuffer->get_frame();
  convert_image(*pVidFrame, imBW);
  convert_image(*pVidFrame, imRGB);
  mVideoBuffer->put_frame(pVidFrame);

  /*
  while (mVideoBuffer->pendingFrame()) {
    VideoFrame<yuv422> *pVidFrame = mVideoBuffer->get_frame();
    convert_image(*pVidFrame, imBW);
    convert_image(*pVidFrame, imRGB);
    mVideoBuffer->put_frame(pVidFrame);
  }
  */
#else
	imRGB.resize(mirSize);
	imBW.resize(mirSize);

	static FlyCapture2::Image flu3Image;

	flu3cam.RetrieveBuffer(&flu3Image);

	FlyCapture2::Image tempImageRGB(VIDEO_H,VIDEO_W,VIDEO_W*3,(unsigned char*) imRGB.data(), VIDEO_H*VIDEO_W*3,FlyCapture2::PIXEL_FORMAT_RGB,FlyCapture2::RGGB);
	flu3Image.Convert(FlyCapture2::PIXEL_FORMAT_RGB,&tempImageRGB);
	
	FlyCapture2::Image tempImageBW(VIDEO_H,VIDEO_W,VIDEO_W,(unsigned char*) imBW.data(), VIDEO_H*VIDEO_W,FlyCapture2::PIXEL_FORMAT_MONO8,FlyCapture2::NONE);
	flu3Image.Convert(FlyCapture2::PIXEL_FORMAT_MONO8,&tempImageBW);
#endif
}

}

