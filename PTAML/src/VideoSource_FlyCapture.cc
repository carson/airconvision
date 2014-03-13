// Copyright 2008 Isis Innovation Limited
#include "VideoSource_FlyCapture.h"
#include <cvd/colourspace_convert.h>
#include <gvars3/instances.h>
#include <flycapture/FlyCapture2GUI.h>

// TODO: read the width and height from the camera
#define VIDEO_H 480
#define VIDEO_W 640

static void PrintErrorAndExit(FlyCapture2::Error error)
{
	error.PrintErrorTrace();
	exit(0);
}

namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;

VideoSource_FlyCapture::VideoSource_FlyCapture(const unsigned int iIndex,
	const unsigned int iMemoryChannel)
{
	FlyCapture2::Error flu3error;
	FlyCapture2::BusManager flu3busMgr;
	FlyCapture2::PGRGuid flu3guid;

	cout << "  VideoSource_FlyCapture: Opening PointGrey Camera " << iIndex
		<< " with memory channel " << iMemoryChannel << endl;

	flu3error =flu3busMgr.GetCameraFromIndex(iIndex, &flu3guid);
	if(flu3error != FlyCapture2::PGRERROR_OK) PrintErrorAndExit(flu3error);

	flu3error = mflu3cam.Connect(&flu3guid);
	if(flu3error != FlyCapture2::PGRERROR_OK) PrintErrorAndExit(flu3error);

	flu3error = mflu3cam.RestoreFromMemoryChannel(iMemoryChannel);
	if(flu3error != FlyCapture2::PGRERROR_OK) PrintErrorAndExit(flu3error);

	flu3error = mflu3cam.StartCapture();
	if(flu3error != FlyCapture2::PGRERROR_OK) PrintErrorAndExit(flu3error);

	mirSize = ImageRef(VIDEO_W, VIDEO_H);
}

VideoSource_FlyCapture::~VideoSource_FlyCapture()
{
	mflu3cam.StopCapture();
	mflu3cam.Disconnect();
}

void VideoSource_FlyCapture::GetAndFillFrameBWandRGB(Image<byte> &imBW,
	Image<Rgb<byte> > &imRGB)
{
	imRGB.resize(mirSize);
	imBW.resize(mirSize);

	mflu3cam.RetrieveBuffer(&mflu3Image);

	FlyCapture2::Image tempImageRGB(VIDEO_H, VIDEO_W, VIDEO_W * 3,
		(unsigned char*)imRGB.data(), VIDEO_H * VIDEO_W * 3,
		FlyCapture2::PIXEL_FORMAT_RGB, FlyCapture2::RGGB);
	mflu3Image.Convert(FlyCapture2::PIXEL_FORMAT_RGB, &tempImageRGB);

	FlyCapture2::Image tempImageBW(VIDEO_H, VIDEO_W, VIDEO_W,
		(unsigned char*)imBW.data(), VIDEO_H * VIDEO_W,
		FlyCapture2::PIXEL_FORMAT_MONO8, FlyCapture2::NONE);
	mflu3Image.Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &tempImageBW);
}

}

