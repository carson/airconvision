// -*- c++ *--
// Copyright 2008 Isis Innovation Limited
//

#ifndef __VIDEO_SOURCE_LINUX_V4L_H
#define __VIDEO_SOURCE_LINUX_V4L_H

#include "VideoSource.h"
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <cvd/Linux/v4lbuffer.h>
#include <cvd/colourspaces.h>

namespace PTAMM {

class VideoSource_Linux_V4L : public VideoSource
{
  public:
    VideoSource_Linux_V4L(const std::string &sName);
    void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);
    const CVD::ImageRef& Size() const { return mirSize; }
  private:
    CVD::V4LBuffer<CVD::yuv422>* mVideoBuffer;
    CVD::ImageRef mirSize;
};

}

#endif
