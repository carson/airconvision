#include "FrameGrabber.h"
#include "Timing.h"
#include "PerformanceMonitor.h"
#include "VideoSource.h"
#include "VideoSource_Linux_V4L.h"
#include "VideoSource_Linux_Gstreamer_File.h"
#include "VideoSource_FlyCapture.h"
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

StereoProcessor::StereoProcessor()
{
}

void StereoProcessor::LoadCalibration(const std::string &sIntrinsicsFile,
    const std::string &sExtrinsicsFile, const CVD::ImageRef &irImageSize)
{
  // reading intrinsic parameters
  cv::FileStorage fs(sIntrinsicsFile, CV_STORAGE_READ);
  if (!fs.isOpened()) {
    throw std::runtime_error("Failed to intrinsics calibration file");
  }

  cv::Mat M1, D1, M2, D2;
  fs["M1"] >> M1;
  fs["D1"] >> D1;
  fs["M2"] >> M2;
  fs["D2"] >> D2;

  fs.open(sExtrinsicsFile, CV_STORAGE_READ);
  if (!fs.isOpened()) {
    throw std::runtime_error("Failed to open extrinsics calibration file");
  }

  cv::Mat R, T, R1, P1, R2, P2;
  fs["R"] >> R;
  fs["T"] >> T;

  SO3<> so3Rotation(wrapMatrix((double*)R.data, 3, 3));
  mse3RightCamFromLeft.get_rotation() = so3Rotation;
  mse3RightCamFromLeft.get_translation() = so3Rotation.inverse() * wrapVector((double*)T.data, 3);

  cv::Size img_size(irImageSize.x, irImageSize.y);

  cv::stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, mQ,
                    cv::CALIB_ZERO_DISPARITY, -1, img_size, &mRoi1, &mRoi2);

  cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, mMap11, mMap12);
  cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, mMap21, mMap22);
}

void StereoProcessor::ExtractPointCloudFrom3DImage(const cv::Mat &_3dImage,
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

void StereoProcessor::ProcessStereoImages(const FrameData& fd)
{
  ImageRef irVideoSize = fd.imFrameBW[0].size();
  const cv::Mat img1(irVideoSize.y, irVideoSize.x, CV_8UC1,
               const_cast<uint8_t*>(fd.imFrameBW[0].data()),
               fd.imFrameBW[0].row_stride());
  const cv::Mat img2(irVideoSize.y, irVideoSize.x, CV_8UC1,
               const_cast<uint8_t*>(fd.imFrameBW[1].data()),
               fd.imFrameBW[1].row_stride());

  cv::remap(img1, mImg1r, mMap11, mMap12, cv::INTER_LINEAR);
  cv::remap(img2, mImg2r, mMap21, mMap22, cv::INTER_LINEAR);

  mDispGenerator.numberOfDisparities = ((irVideoSize.x/8) + 15) & -16;
  mDispGenerator.Generate(mImg1r, mImg2r, mRoi1, mRoi2, mQ, mDisp);

  cv::Mat disp8;
  mDisp.convertTo(disp8, CV_8U, 255.0/mDispGenerator.numberOfDisparities);
  cv::imshow("disp", disp8);
  cv::waitKey(1);
}

void StereoProcessor::GeneratePointCloud(
    std::vector<TooN::Vector<3> > &vv3PointCloud) const
{
  cv::Mat xyz;
  cv::reprojectImageTo3D(mDisp, xyz, mQ);

  // Convert the Mat into a list of points (with some filtering)
  ExtractPointCloudFrom3DImage(xyz, vv3PointCloud);
}

const CVD::SubImage<float> StereoProcessor::GetDisparityMap() const
{
  return SubImage<float>(reinterpret_cast<float*>(mDisp.data),
                         ImageRef(mDisp.cols, mDisp.rows),
                         (size_t)mDisp.step/sizeof(float));
}

double StereoProcessor::GetFocalLength() const
{
  return mQ.at<double>(2, 3);
}

double StereoProcessor::GetBaseline() const
{
  return 1.0 / mQ.at<double>(3, 2);
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

VideoSource* FrameGrabber::CreateVideoSource(const std::string &sName) const
{
  int nFlyCaptureIndex = GV3::get<int>(sName + ".PointGreyIndex", -1);
  int nFlyCaptureChannel = GV3::get<int>(sName + ".PointGreyMemoryChannel", 0);
  if (nFlyCaptureIndex >= 0) {
    return new VideoSource_FlyCapture(nFlyCaptureIndex, nFlyCaptureChannel);
  }

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
    fd.tpCaptureTime = std::chrono::high_resolution_clock::now();
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
