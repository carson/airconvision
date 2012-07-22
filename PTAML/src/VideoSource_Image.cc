#include "VideoSource_Image.h"
#include <cvd/colourspace_convert.h>
#include <cvd/image_io.h>
#include <gvars3/instances.h>

namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;

VideoSource_Image::VideoSource_Image(const std::string &sName)
{
  string sImageFile = GV3::get<string>(sName + ".ImageFile", "frame.jpg");
  cout << "Loading image for " << sName << ": " << sImageFile << endl;

  img_load(mimRGB, sImageFile);
  mimBW.resize(mimRGB.size());
  convert_image(mimRGB, mimBW);

  mnFrameRate = GV3::get<int>(sName + ".Framerate", 60);
  mirSize = mimRGB.size();

  cout << "Loaded image. Size: " << mirSize << endl;
}

void VideoSource_Image::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<Rgb<byte> > &imRGB)
{
  imBW.copy_from(mimBW);
  imRGB.copy_from(mimRGB);
  mRateLimiter.Limit(mnFrameRate);
}

}
