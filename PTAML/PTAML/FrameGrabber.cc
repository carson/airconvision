#include "FrameGrabber.h"
#include "Timing.h"
#include "VideoSource.h"
#include "VideoSource_Linux_V4L.h"
#include "VideoSource_Linux_Gstreamer_File.h"

#include <gvars3/gvars3.h>
#include <gvars3/instances.h>

using namespace std;
using namespace CVD;
using namespace GVars3;

namespace PTAMM {


FrameGrabber::FrameGrabber()
  : mpVideoSource1(NULL)
  , mpVideoSource2(NULL)
  , mbFreezeVideo(false)
{
  mbUseStereo = GV3::get<int>("UseStereo", 1, SILENT);

  // Initialize camera 1
  mpVideoSource1 = CreateVideoSource("VideoSource1");
  // Initialize buffers for camera 1
  ImageRef irVideoSize1 = mpVideoSource1->Size();
  mimFrameBW1.resize(irVideoSize1);
  mimFrameRGB1.resize(irVideoSize1);

  if (mbUseStereo) {
    // Initialize camera 2
    mpVideoSource2 = CreateVideoSource("VideoSource2");
    // Initialize buffers for camera 2
    ImageRef irVideoSize2 = mpVideoSource2->Size();
    mimFrameBW2.resize(irVideoSize2);
  }
}

VideoSource* FrameGrabber::CreateVideoSource(const std::string &sName) const
{
  string videoSourceFileName = GV3::get<string>(sName + ".FileInput", "");

  if (videoSourceFileName.empty()) {
    return new VideoSource_Linux_V4L(sName);
  } else {
    return new VideoSource_Linux_Gstreamer_File(videoSourceFileName);
  }
}

void FrameGrabber::GrabNextFrame()
{
  if (!mbFreezeVideo) {
    gVideoSourceTimer.Start();

    if (mbUseStereo) {
      mpVideoSource2->GetAndFillFrameBWandRGB(mimFrameBW2, mimFrameRGB1);
    }

    mpVideoSource1->GetAndFillFrameBWandRGB(mimFrameBW1, mimFrameRGB1);

    gVideoSourceTimer.Stop();
  }
}

const CVD::ImageRef& FrameGrabber::GetFrameSize() const
{
  return mpVideoSource1->Size();
}

}
