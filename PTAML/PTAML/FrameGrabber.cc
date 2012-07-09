#include "FrameGrabber.h"
#include "Timing.h"
#include "VideoSource.h"
#include "VideoSource_Linux_V4L.h"
#include "VideoSource_Linux_Gstreamer_File.h"

#include <gvars3/gvars3.h>
#include <gvars3/instances.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <stdexcept>

using namespace std;
using namespace CVD;
using namespace GVars3;
using namespace TooN;

namespace PTAMM {

DisparityGenerator::DisparityGenerator()
  : bm(cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE)
  , algorithm(STEREO_BM)
  , numberOfDisparities(80)
  , SADWindowSize(10)
{
}

void
DisparityGenerator::Generate(const cv::Mat &img1, const cv::Mat &img2,
                             const cv::Rect &roi1, const cv::Rect &roi2,
                             const cv::Mat &Q, cv::Mat &disp)
{
  using namespace cv;

  if (algorithm == STEREO_BM) {
    bm.state->roi1 = roi1;
    bm.state->roi2 = roi2;
    bm.state->preFilterCap = 31;
    bm.state->SADWindowSize = SADWindowSize >= 5 ? SADWindowSize | 1 : 9;
    bm.state->minDisparity = 0;
    bm.state->numberOfDisparities = numberOfDisparities;
    bm.state->textureThreshold = 10;
    bm.state->uniquenessRatio = 10;
    bm.state->speckleWindowSize = 100;
    bm.state->speckleRange = 32;
    bm.state->disp12MaxDiff = 1;

    bm.state->trySmallerWindows = 1;

    bm(img1, img2, disp, CV_32F);
  } else if (algorithm == STEREO_VAR) {
    var.levels = 3;                       // ignored with USE_AUTO_PARAMS
    var.pyrScale = 0.5;                   // ignored with USE_AUTO_PARAMS
    var.nIt = 25;
    var.minDisp = -numberOfDisparities;
    var.maxDisp = 0;
    var.poly_n = 3;
    var.poly_sigma = 0.0;
    var.fi = 15.0f;
    var.lambda = 0.03f;
    var.penalization = var.PENALIZATION_TICHONOV; // ignored with USE_AUTO_PARAMS
    var.cycle = var.CYCLE_V;              // ignored with USE_AUTO_PARAMS
    var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;

    var(img1, img2, disp);
  } else if (algorithm == STEREO_SGBM || algorithm == STEREO_HH) {
    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

    int cn = img1.channels();

    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = numberOfDisparities;
    sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = 100;
    sgbm.speckleRange = 32;
    sgbm.disp12MaxDiff = -1; //sg_disp12MaxDiff;
    sgbm.fullDP = algorithm == STEREO_HH;

    sgbm(img1, img2, disp);
  }
}

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

    LoadCalibration();
  }
}

void FrameGrabber::ExtractPointCloud(const cv::Mat &_3dImage,
                                     std::vector<TooN::Vector<3> >& points) const
{
  const double max_z = 2000.0;
  points.clear();
  for (int i = 0; i < _3dImage.rows; ++i) {
    for (int j = 0; j < _3dImage.cols; ++j) {
      const cv::Vec3f &pt = _3dImage.at<cv::Vec3f>(i, j);
      if (fabs(pt[2]) < max_z) {
        points.push_back(makeVector(pt[0], pt[1], pt[2]));
      }
    }
  }
}

void FrameGrabber::ProcessStereoImages()
{
  ImageRef irVideoSize = mpVideoSource1->Size();
  cv::Mat img1(irVideoSize.y, irVideoSize.x, CV_8UC1,
               mimFrameBW1.data(), mimFrameBW1.row_stride());
  cv::Mat img2(irVideoSize.y, irVideoSize.x, CV_8UC1,
               mimFrameBW2.data(), mimFrameBW2.row_stride());

  cv::remap(img1, mImg1r, mMap11, mMap12, cv::INTER_LINEAR);
  cv::remap(img2, mImg2r, mMap21, mMap22, cv::INTER_LINEAR);

  mDispGenerator.numberOfDisparities = ((GetFrameSize().x/8) + 15) & -16;
  mDispGenerator.Generate(mImg1r, mImg2r, mRoi1, mRoi2, mQ, mDisp);

  cv::Mat xyz;
  cv::reprojectImageTo3D(mDisp, xyz, mQ);

  // Convert the Mat into a list of points (with some filtering)
  ExtractPointCloud(xyz, mPointCloud);

/*
  cv::Mat disp8;
  mDisp.convertTo(disp8, CV_8U, 255.0/mDispGenerator.numberOfDisparities);
  cv::imshow("disp", disp8);
  cv::waitKey(10);
  */
}

void FrameGrabber::LoadCalibration()
{
  // reading intrinsic parameters
  cv::FileStorage fs("Data/intrinsics.yml", CV_STORAGE_READ);
  if (!fs.isOpened()) {
    throw std::runtime_error("Failed to intrinsic.yml");
  }

  cv::Mat M1, D1, M2, D2;
  fs["M1"] >> M1;
  fs["D1"] >> D1;
  fs["M2"] >> M2;
  fs["D2"] >> D2;

  fs.open("Data/extrinsics.yml", CV_STORAGE_READ);
  if (!fs.isOpened()) {
    throw std::runtime_error("Failed to open extrinsinc.yml");
  }

  cv::Mat R, T, R1, P1, R2, P2;
  fs["R"] >> R;
  fs["T"] >> T;

  cv::Size img_size(640, 480);

  cv::stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, mQ,
                    cv::CALIB_ZERO_DISPARITY, -1, img_size, &mRoi1, &mRoi2);


  cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, mMap11, mMap12);
  cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, mMap21, mMap22);
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
