#include "VideoSource.h"
#include "VideoSource_Linux_V4L.h"
#include "VideoSource_Linux_Gstreamer_File.h"

#include <gvars3/gvars3.h>
#include <gvars3/instances.h>

using namespace std;
using namespace GVars3;

namespace PTAMM {

typedef VideoSource_Linux_V4L CameraVideoSource;
typedef VideoSource_Linux_Gstreamer_File FileVideoSource;

VideoSource* CreateVideoSource()
{
  string videoSourceFileName = GV3::get<string>("VideoInput", "");

  if (videoSourceFileName.empty()) {
    return new CameraVideoSource("VideoSource1");
  } else {
    return new FileVideoSource(videoSourceFileName);
  }
}

}
