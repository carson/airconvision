#ifndef __FRAME_GRABBER_H
#define __FRAME_GRABBER_H

#include <cvd/image.h>
#include <cvd/rgb.h>

#include <string>

namespace PTAMM {

class VideoSource;

class FrameGrabber {
  public:
    FrameGrabber();

    void GrabNextFrame();

    bool IsUsingStereo() const { return mbUseStereo; }

    const CVD::ImageRef& GetFrameSize() const;

    const CVD::Image<CVD::Rgb<CVD::byte>>& GetFrameRGB1() const { return mimFrameRGB1; }
    const CVD::Image<CVD::byte>& GetFrameBW1() const { return mimFrameBW1; }

  private:
    VideoSource* CreateVideoSource(const std::string &sName) const;

  private:
    bool mbUseStereo;

    VideoSource *mpVideoSource1;                     // Camera 1
    VideoSource *mpVideoSource2;                     // Camera 2

    CVD::Image<CVD::Rgb<CVD::byte>> mimFrameRGB1;    // RGB frame from camera 1

    CVD::Image<CVD::byte> mimFrameBW1;               // BW frame from camera 1
    CVD::Image<CVD::byte> mimFrameBW2;               // BW frame from camera 2

    bool mbFreezeVideo;
};

}

#endif
