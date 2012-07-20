#ifndef __FRAME_GRABBER_H
#define __FRAME_GRABBER_H

#include <cvd/image.h>
#include <cvd/rgb.h>
#include <TooN/TooN.h>

#include <opencv2/opencv.hpp>

#include <string>

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

class FrameGrabber {
  public:
    FrameGrabber(PerformanceMonitor *pPerfMon);

    void GrabNextFrame();
    void ProcessStereoImages();

    bool IsUsingStereo() const { return mbUseStereo; }

    const CVD::ImageRef& GetFrameSize() const;

    const CVD::Image<CVD::Rgb<CVD::byte>>& GetFrameRGB1() const { return mimFrameRGB1; }
    const CVD::Image<CVD::byte>& GetFrameBW1() const { return mimFrameBW1; }

    const std::vector<TooN::Vector<3> >& GetPointCloud() const { return mPointCloud; }

  private:
    VideoSource* CreateVideoSource(const std::string &sName) const;
    void LoadCalibration();

    void ExtractPointCloud(const cv::Mat &_3dImage,
                           std::vector<TooN::Vector<3> >& points) const;

  private:
    PerformanceMonitor *mpPerfMon;

    bool mbUseStereo;

    VideoSource *mpVideoSource1;                     // Camera 1
    VideoSource *mpVideoSource2;                     // Camera 2

    CVD::Image<CVD::Rgb<CVD::byte>> mimFrameRGB1;    // RGB frame from camera 1

    CVD::Image<CVD::byte> mimFrameBW1;               // BW frame from camera 1
    CVD::Image<CVD::byte> mimFrameBW2;               // BW frame from camera 2

    bool mbFreezeVideo;

    cv::Mat mMap11, mMap12, mMap21, mMap22;
    cv::Rect mRoi1, mRoi2;
    cv::Mat mQ;
    cv::Mat mImg1r, mImg2r;
    cv::Mat mDisp;

    std::vector<TooN::Vector<3> > mPointCloud;

    DisparityGenerator mDispGenerator;

};

}

#endif
