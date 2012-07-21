#ifndef __FRAME_GRABBER_H
#define __FRAME_GRABBER_H

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <TooN/TooN.h>

#include <opencv2/opencv.hpp>

#include <string>
#include <condition_variable>
#include <mutex>
#include <thread>

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
};

class FrameGrabber {
  public:
    FrameGrabber(PerformanceMonitor *pPerfMon);
    ~FrameGrabber();

    void operator()();

    void StopThread() { mbDone = true; }

    const FrameData& GrabFrame();
    void ProcessStereoImages();

    bool IsUsingStereo() const { return mbUseStereo; }

    const CVD::ImageRef& GetFrameSize() const;

    const FrameData& GetFrameData() const;

    const std::vector<TooN::Vector<3> >& GetPointCloud() const { return mPointCloud; }

  private:
    VideoSource* CreateVideoSource(const std::string &sName) const;
    void LoadCalibration();

    void FetchNextFrame(); // Fetches new frame data from camera

    void ExtractPointCloud(const cv::Mat &_3dImage,
                           std::vector<TooN::Vector<3> >& points) const;

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

    // Stereo rectification
    cv::Mat mMap11, mMap12, mMap21, mMap22;
    cv::Rect mRoi1, mRoi2;
    cv::Mat mQ;
    cv::Mat mImg1r, mImg2r;
    cv::Mat mDisp;

    // 3d point cloud when using stereo vision
    std::vector<TooN::Vector<3> > mPointCloud;

    // Disparity generation when using stereo vision
    DisparityGenerator mDispGenerator;
};

}

#endif
