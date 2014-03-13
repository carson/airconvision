// -*- c++ *--
// Copyright 2008 Isis Innovation Limited
//

#ifndef __VIDEO_SOURCE_FLYCAPTURE
#define __VIDEO_SOURCE_FLYCAPTURE

#include "VideoSource.h"
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <cvd/colourspaces.h>
#include <flycapture/FlyCapture2.h>

namespace PTAMM {

class VideoSource_FlyCapture : public VideoSource
{
  public:
    VideoSource_FlyCapture(const unsigned int iIndex,
      const unsigned int iMemoryChannel);
    ~VideoSource_FlyCapture();
    void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW,
      CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);
    const CVD::ImageRef& Size() const { return mirSize; }
  private:
    CVD::ImageRef mirSize;
    FlyCapture2::Camera mflu3cam;
    FlyCapture2::Image mflu3Image;
};

}

#endif
