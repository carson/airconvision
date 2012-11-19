#ifndef __FRAME_GRABBER_H
#define __FRAME_GRABBER_H

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>

#include <opencv2/opencv.hpp>

#include <string>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>

namespace PTAMM {

class VideoSource;
class PerformanceMonitor;

class DisparityGenerator {
  public:
    enum DisparityAlgorithm {
      STEREO_BM,
      STEREO_SGBM,
      STEREO_HH,
      STEREO_VAR
    };

    DisparityGenerator();

    void Generate(const cv::Mat &img1, const cv::Mat &img2,
                  const cv::Rect &roi1, const cv::Rect &roi2,
                  const cv::Mat &Q, cv::Mat &disp);

    DisparityAlgorithm algorithm;
    int numberOfDisparities;
    int SADWindowSize;

  private:
    cv::StereoBM bm;
    cv::StereoSGBM sgbm;
    cv::StereoVar var;
};

struct FrameData {
  void Resize(const CVD::ImageRef &irImageSize);
  CVD::Image<CVD::Rgb<CVD::byte>> imFrameRGB[2];    // RGB frame
  CVD::Image<CVD::byte> imFrameBW[2];             // BW frame

  std::chrono::time_point<
    std::chrono::high_resolution_clock> tpCaptureTime;
};

class StereoProcessor {
  public:
    StereoProcessor();
    void LoadCalibration(const std::string &sIntrinsicsFile,
        const std::string &sExtrinsicsFile, const CVD::ImageRef &irImageSize);
    void ProcessStereoImages(const FrameData& fd);
    void GeneratePointCloud(std::vector<TooN::Vector<3> > &vv3PointCloud) const;
    const TooN::SE3<>& GetRightCameraPose() const { return mse3RightCamFromLeft; }
    const CVD::SubImage<float> GetDisparityMap() const;
    double GetFocalLength() const;
    double GetBaseline() const;
  private:
    void ExtractPointCloudFrom3DImage(const cv::Mat &_3dImage,
        std::vector<TooN::Vector<3> >& points) const;
  private:
    // Stereo rectification
    cv::Mat mMap11, mMap12, mMap21, mMap22;
    cv::Rect mRoi1, mRoi2;
    cv::Mat mQ;
    cv::Mat mImg1r, mImg2r;
    cv::Mat mDisp;

    TooN::SE3<> mse3RightCamFromLeft;

    // Disparity generation when using stereo vision
    DisparityGenerator mDispGenerator;
};

class FrameGrabber {
  public:
    FrameGrabber(PerformanceMonitor *pPerfMon);
    ~FrameGrabber();

    void operator()();

    void StopThread() { mbDone = true; }

    const FrameData& GrabFrame();

    bool IsUsingStereo() const { return mbUseStereo; }

    void SetFreezeFrame(bool bFreeze) { mbFreezeVideo = bFreeze; }
    bool IsFrameFrozen() const { return mbFreezeVideo; }

    const CVD::ImageRef& GetFrameSize() const;
    const FrameData& GetFrameData() const;

  private:
    VideoSource* CreateVideoSource(const std::string &sName) const;
    void FetchNextFrame(); // Fetches new frame data from camera

  private:
    bool mbDone;

    PerformanceMonitor *mpPerfMon;

    bool mbUseStereo;

    VideoSource *mpVideoSource1;                     // Camera 1
    VideoSource *mpVideoSource2;                     // Camera 2

    // Data members for keeping track of the video data and synchronize between threads
    FrameData maFrameData[2];
    FrameData mTempFrameData;
    size_t mnFrameDataIndex;
    bool mbHasNewFrame;
    mutable std::mutex mMutex;
    std::condition_variable mCond;

    // Possibility to freeze the video on a frame
    bool mbFreezeVideo;
};

}

#endif
