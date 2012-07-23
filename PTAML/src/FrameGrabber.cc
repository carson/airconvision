#include "FrameGrabber.h"
#include "Timing.h"
#include "PerformanceMonitor.h"
#include "VideoSource.h"
#include "VideoSource_Linux_V4L.h"
#include "VideoSource_Linux_Gstreamer_File.h"
#include "VideoSource_Image.h"

#include <gvars3/gvars3.h>
#include <gvars3/instances.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <stdexcept>
#include <chrono>

using namespace std;
using namespace CVD;
using namespace GVars3;
using namespace TooN;

namespace PTAMM {

DisparityGenerator::DisparityGenerator()
  : algorithm(STEREO_BM)
  , numberOfDisparities(80)
  , SADWindowSize(10)
  , bm(cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE)
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

void FrameData::Resize(const CVD::ImageRef &irImageSize)
{
  for (int i = 0; i < 2; ++i) {
    imFrameBW[i].resize(irImageSize);
    imFrameRGB[i].resize(irImageSize);
  }
}

FrameGrabber::FrameGrabber(PerformanceMonitor *pPerfMon)
  : mbDone(false)
  , mpPerfMon(pPerfMon)
  , mpVideoSource1(NULL)
  , mpVideoSource2(NULL)
  , mnFrameDataIndex(0)
  , mbHasNewFrame(false)
  , mbFreezeVideo(false)
{
  mbUseStereo = GV3::get<int>("UseStereo", 1, SILENT);

  // Initialize camera 1
  mpVideoSource1 = CreateVideoSource("VideoSource1");
  // Initialize buffers for camera 1
  ImageRef irVideoSize1 = mpVideoSource1->Size();
  maFrameData[0].Resize(irVideoSize1);
  maFrameData[1].Resize(irVideoSize1);
  mTempFrameData.Resize(irVideoSize1);

  if (mbUseStereo) {
    // Initialize camera 2
    mpVideoSource2 = CreateVideoSource("VideoSource2");
    // Just check so that the video sizes are equal!
    ImageRef irVideoSize2 = mpVideoSource2->Size();
    assert(irVideoSize1 == irVideoSize2);
    LoadCalibration();
  }
}

FrameGrabber::~FrameGrabber()
{
  delete mpVideoSource1;
  delete mpVideoSource2;
}

void FrameGrabber::operator ()()
{
  while (!mbDone) {
    FetchNextFrame();
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
        points.push_back(makeVector(-pt[0], -pt[1], -pt[2]));
      }
    }
  }
}

void FrameGrabber::ProcessStereoImages()
{
  FrameData& fd = maFrameData[mnFrameDataIndex];

  ImageRef irVideoSize = mpVideoSource1->Size();
  cv::Mat img1(irVideoSize.y, irVideoSize.x, CV_8UC1,
               fd.imFrameBW[0].data(), fd.imFrameBW[0].row_stride());
  cv::Mat img2(irVideoSize.y, irVideoSize.x, CV_8UC1,
               fd.imFrameBW[1].data(), fd.imFrameBW[1].row_stride());

  cv::remap(img1, mImg1r, mMap11, mMap12, cv::INTER_LINEAR);
  cv::remap(img2, mImg2r, mMap21, mMap22, cv::INTER_LINEAR);

  mDispGenerator.numberOfDisparities = ((GetFrameSize().x/8) + 15) & -16;
  mDispGenerator.Generate(mImg1r, mImg2r, mRoi1, mRoi2, mQ, mDisp);

  cv::Mat xyz;
  cv::reprojectImageTo3D(mDisp, xyz, mQ);

  // Convert the Mat into a list of points (with some filtering)
  ExtractPointCloud(xyz, mPointCloud);

  cv::Mat disp8;
  mDisp.convertTo(disp8, CV_8U, 255.0/mDispGenerator.numberOfDisparities);
  cv::imshow("disp", disp8);
  cv::waitKey(1);
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

  Matrix<3> rot = wrapMatrix((double*)R.data, 3, 3);
  mse3RightCamFromLeft.get_rotation() = rot.T();
  mse3RightCamFromLeft.get_translation() = wrapVector((double*)T.data, 3);

  cv::Size img_size(640, 480);

  cv::stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, mQ,
                    cv::CALIB_ZERO_DISPARITY, -1, img_size, &mRoi1, &mRoi2);


  cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, mMap11, mMap12);
  cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, mMap21, mMap22);
}

VideoSource* FrameGrabber::CreateVideoSource(const std::string &sName) const
{
  string sVideoFilename = GV3::get<string>(sName + ".VideoFile", "");
  if (!sVideoFilename.empty()) {
    return new VideoSource_Linux_Gstreamer_File(sVideoFilename);
  }

  string sImageFilename = GV3::get<string>(sName + ".ImageFile", "");
  if (!sImageFilename.empty()) {
    return new VideoSource_Image(sName);
  }

  return new VideoSource_Linux_V4L(sName);
}

const FrameData& FrameGrabber::GrabFrame()
{
  std::unique_lock<std::mutex> lock(mMutex);
  while (!mbHasNewFrame) {
      mCond.wait(lock);
  }
  mnFrameDataIndex ^= 1;
  mbHasNewFrame = false;
  return maFrameData[mnFrameDataIndex];
}

void FrameGrabber::FetchNextFrame()
{
  mpPerfMon->StartTimer("grab_frame");

  if (!mbFreezeVideo) {
    FrameData& fd = mTempFrameData;
    mpVideoSource1->GetAndFillFrameBWandRGB(fd.imFrameBW[0], fd.imFrameRGB[0]);
    if (mbUseStereo) {
      mpVideoSource2->GetAndFillFrameBWandRGB(fd.imFrameBW[1], fd.imFrameRGB[1]);
    }
  }

  {
    std::unique_lock<std::mutex> lock(mMutex);
    FrameData& destFd = maFrameData[mnFrameDataIndex ^ 1];

    if (mbFreezeVideo) {
      FrameData& currentFd = maFrameData[mnFrameDataIndex];
      for (int i = 0; i < 2; ++i) {
        destFd.imFrameBW[i].copy_from(currentFd.imFrameBW[i]);
        destFd.imFrameRGB[i].copy_from(currentFd.imFrameRGB[i]);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      using std::swap;
      swap(mTempFrameData, destFd);
    }

    mbHasNewFrame = true;
    mCond.notify_one();
  }

  mpPerfMon->UpdateRateCounter("frame_grabber");
  mpPerfMon->StopTimer("grab_frame");
}

const CVD::ImageRef& FrameGrabber::GetFrameSize() const
{
  return mpVideoSource1->Size();
}

const FrameData& FrameGrabber::GetFrameData() const
{
  std::unique_lock<std::mutex> lock(mMutex);
  return maFrameData[mnFrameDataIndex];
}

}
