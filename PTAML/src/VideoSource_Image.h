// -*- c++ *--
// Copyright 2008 Isis Innovation Limited
//

#ifndef __VIDEO_SOURCE_IMAGE_H
#define __VIDEO_SOURCE_IMAGE_H

#include "VideoSource.h"
#include "Timing.h"

#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <cvd/colourspaces.h>

namespace PTAMM {

class VideoSource_Image : public VideoSource {
  public:
    VideoSource_Image(const std::string &sName);
    void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);
    const CVD::ImageRef& Size() const { return mirSize; }
  private:
    CVD::Image<CVD::byte> mimBW;
    CVD::Image<CVD::Rgb<CVD::byte> > mimRGB;
    CVD::ImageRef mirSize;
    int mnFrameRate;
    RateLimiter mRateLimiter;
};

}

#endif
