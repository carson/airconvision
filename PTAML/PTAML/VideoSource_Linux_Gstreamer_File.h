// -*- c++ *--
// Copyright 2008 Isis Innovation Limited
//

#ifndef __VIDEO_SOURCE_LINUX_GSTREAMER_FILE_H
#define __VIDEO_SOURCE_LINUX_GSTREAMER_FILE_H

#include "VideoSource.h"

#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>

#include <gst/gst.h>

#include <string>

namespace PTAMM {

class VideoSource_Linux_Gstreamer_File : public VideoSource
{
  public:
    VideoSource_Linux_Gstreamer_File(const std::string& videoSourceFile);
    ~VideoSource_Linux_Gstreamer_File();

    void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imBW, CVD::Image<CVD::Rgb<CVD::byte> > &imRGB);
    const CVD::ImageRef& Size() const { return mirSize; }
  private:
    GstElement *mRgbVideoSink;
    GstElement *mGrayVideoSink;
    GstElement *mSourcePipeline;
    double mFrameNumber;
    CVD::ImageRef mirSize;
};

}

#endif
