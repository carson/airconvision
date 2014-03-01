#include "Frontend.h"
#include "VideoSource.h"
#include "Tracker.h"
#include "PerformanceMonitor.h"
#include "FeatureGrid.h"
#include "FrameGrabber.h"
#include "Utils.h"
#include "PatchFinder.h"
#include "MathUtils.h"

#include <cvd/image_ref.h>
#include <cvd/image_io.h>
#include <TooN/SVD.h>

#include <iostream>

using namespace std;
using namespace CVD;
using namespace GVars3;

namespace PTAMM {

void FrontendMonitor::PushDrawData(FrontendDrawData &drawData)
{
  std::lock_guard<std::mutex> lock(mMutex);
  using std::swap;
  swap(mDrawData, drawData);
  mbHasFrontendData = true;
}

bool FrontendMonitor::PopDrawData(FrontendDrawData &drawData)
{
  std::lock_guard<std::mutex> lock(mMutex);
  if (!mbHasFrontendData) {
    return false;
  }

  using std::swap;
  swap(mDrawData, drawData);
  mbHasFrontendData = false;
  return true;
}

void FrontendMonitor::PushUserInvoke()
{
  std::lock_guard<std::mutex> lock(mMutex);
  mbUserInvoke = true;
}

bool FrontendMonitor::PopUserInvoke()
{
  std::lock_guard<std::mutex> lock(mMutex);
  if (mbUserInvoke) {
    mbUserInvoke = false;
    return true;
  }

  return false;
}

void FrontendMonitor::PushUserResetInvoke()
{
  std::lock_guard<std::mutex> lock(mMutex);
  mbUserResetInvoke = true;
}

bool FrontendMonitor::PopUserResetInvoke()
{
  std::lock_guard<std::mutex> lock(mMutex);
  if (mbUserResetInvoke) {
    mbUserResetInvoke = false;
    return true;
  }

  return false;
}

void FrontendMonitor::SetCurrentPose(const SE3<> &se3CurrenPose)
{
  std::lock_guard<std::mutex> lock(mMutex);
  mse3CurrentPose = se3CurrenPose;
}

SE3<> FrontendMonitor::GetCurrentPose() const
{
  std::lock_guard<std::mutex> lock(mMutex);
  return mse3CurrentPose;
}

Frontend::Frontend(FrameGrabber *pFrameGrabber,
                   const ATANCamera &camera,
                   MapMaker *pMapMaker,
                   InitialTracker *pInitialTracker,
                   Tracker *pTracker,
                   ScaleMarkerTracker *pScaleMarkerTracker,
                   PerformanceMonitor *pPerfMon)
  : mbDone(false)
  , mbInitialTracking(true)
  , mbHasDeterminedScale(false)
  , mCamera(camera)
  , mpFrameGrabber(pFrameGrabber)
  , mpInitialTracker(pInitialTracker)
  , mpTracker(pTracker)
  , mpScaleMarkerTracker(pScaleMarkerTracker)
  , mpMapMaker(pMapMaker)
  , mpPerfMon(pPerfMon)
  , mKeyFrame(camera)
  , mbSetScaleNextTime(false)
{
  GV3::Register(mgvnFeatureDetector, "FeatureDetector", (int)PLAIN_FAST10, SILENT);

  mStereoProcessor.LoadCalibration("Data/intrinsics.yml", "Data/extrinsics.yml",
                                   pFrameGrabber->GetFrameSize());
}

void Frontend::operator()()
{
  static gvar3<int> gvnOutputWorldCoordinates("Debug.OutputWorldCoordinates", 0, HIDDEN|SILENT);

  std::ofstream coordinateLogFile("coordinates.txt", ios::out | ios::trunc);
  if (!coordinateLogFile) {
    cerr << "Failed to open coordinates.txt" << endl;
  }

  StopWatch stopWatch;
  stopWatch.Start();

  while (!mbDone) {
    bool bUserInvoke = monitor.PopUserInvoke();
    bool bUserResetInvoke = monitor.PopUserResetInvoke();

    const FrameData& fd = mpFrameGrabber->GrabFrame();

    // Initialize keyframe, find features etc
    mKeyFrame.InitFromImage(fd.imFrameBW[0],
                            static_cast<FeatureDetector>(*mgvnFeatureDetector));

    // Set some of the draw data
    mDrawData.imFrame.copy_from(fd.imFrameRGB[0]);

    if (mpFrameGrabber->IsUsingStereo()) {
      mDrawData.imFrameStereo.copy_from(fd.imFrameRGB[1]);
    }

    mDrawData.bInitialTracking = mbInitialTracking;

    if (bUserResetInvoke) {
      // Go back to initial tracking again
      Reset();
    }

    mpPerfMon->StartTimer("tracking_total");

    if (mbInitialTracking) {
      ProcessInitialization(bUserInvoke);
    }

    bool bRunTracker = !mbInitialTracking;
    mpTracker->ProcessFrame(mKeyFrame, bRunTracker);

    if (bRunTracker) {
      const SE3<> &se3CurrentPose = mpTracker->GetCurrentPose();
      monitor.SetCurrentPose(se3CurrentPose);


      if (!mbHasDeterminedScale) {
        DetermineScaleFromMarker(fd, bUserInvoke);
      } else {
        if (*gvnOutputWorldCoordinates) {

          auto timestamp = std::chrono::duration_cast<
              std::chrono::microseconds>(fd.tpCaptureTime.time_since_epoch()).count();
          coordinateLogFile << timestamp << " " << stopWatch.Elapsed() << " " << mpTracker->RealWorldCoordinate() << std::endl;
        }

        SE3<> se3RotatedPose = se3CurrentPose;

        mOnTrackedPoseUpdatedSlot(se3RotatedPose, 
            !mpTracker->IsLost(), fd.tpCaptureTime);
      }

      mpTracker->GetDrawData(mDrawData.tracker);
      mDrawData.bHasDeterminedScale = mbHasDeterminedScale;
      mDrawData.se3MarkerPose = mse3MarkerPose;
      mDrawData.sStatusMessage = mpTracker->GetMessageForUser();
      mDrawData.bInitialTracking = false;
    }

    monitor.PushDrawData(mDrawData);

    mpPerfMon->UpdateRateCounter("frontend");

    mpPerfMon->StopTimer("tracking_total");
  }
}

void Frontend::Reset()
{
  mpInitialTracker->Reset();
  mpTracker->Reset();
  mKeyFrame.Reset();
  mbInitialTracking = true;
  mbHasDeterminedScale = false;
  mbSetScaleNextTime = false;
}

void Frontend::ProcessInitialization(bool bUserInvoke)
{
  if (mpFrameGrabber->IsUsingStereo()) {

    mbHasDeterminedScale = true;

    const FrameData& fd = mpFrameGrabber->GetFrameData();

    vector<Vector<3>> vv3PointCloud;
    mStereoProcessor.ProcessStereoImages(fd);
//    mStereoProcessor.GeneratePointCloud(vv3PointCloud);

    double f = mStereoProcessor.GetFocalLength();
    double L = mStereoProcessor.GetBaseline();

    cout << "Cam: " << f << ", " << L << endl;

    // Find the ground plane in the point cloud
    mStereoPlaneFinder.Update(mStereoProcessor.GetDisparityMap(), f, L);

    vector<ImageRef> vBestFeatures;

    FeatureGrid featureGrid(640, 480, 8, 4, 15);
    featureGrid.FindFeatures(fd.imFrameBW[0]);
    featureGrid.FindBestFeatures(fd.imFrameBW[0]);
    featureGrid.GetBestFeatures(150, vBestFeatures);

    KeyFrame rightKF(mCamera);
    rightKF.InitFromImage(fd.imFrameBW[1],
                          static_cast<FeatureDetector>(*mgvnFeatureDetector));

    ATANCamera c = mCamera;
    Vector<4> v4Plane = mStereoPlaneFinder.GetPlane();
    v4Plane[3] *= -1;

    vector<ImageRef> vBackProjectedPts;

    SE3<> se3RightCamFromLeft = mStereoProcessor.GetRightCameraPose();

    static PatchFinder Finder;

    ImageRef irImageSize = fd.imFrameBW[0].size();


    vector<Vector<3>> vv3PlanePoints;

    vector<pair<ImageRef, ImageRef>> vMatches;

    for (auto it = vBestFeatures.begin(); it != vBestFeatures.end(); ++it) {
      Vector<3> v3WorldPoint;
      if (PickPointOnPlane(c, v4Plane, makeVector(it->x, it->y), v3WorldPoint)) {

        Vector<3> v3Cam = se3RightCamFromLeft * v3WorldPoint;

        if(v3Cam[2] < 0.001) { // Behind the camera
          cout << "behind!" << endl;
          continue;
        }

        Vector<2> v2ImPlane = project(v3Cam);
        if ((v2ImPlane * v2ImPlane) > (c.LargestRadiusInImage() * c.LargestRadiusInImage())) {
          continue;
        }

        Vector<2> v2Image = c.Project(v2ImPlane);
        if (c.Invalid()) {
          continue;
        }

        if(v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize[0] || v2Image[1] > irImageSize[1]) {
          continue;
        }

        Finder.MakeTemplateCoarseNoWarpFromImage(fd.imFrameBW[0], *it);

        if (Finder.TemplateBad()) {
          continue;
        }

        bool bFound = Finder.FindPatchCoarse(ir(v2Image), rightKF, 15);
        if (!bFound) {
          continue;
        }

        Vector<2> v2RootPos = Finder.GetCoarsePosAsVector();

        //  Found a likely candidate along epipolar ray
        Finder.MakeSubPixTemplate();
        Finder.SetSubPixPos(v2RootPos);
        bool bSubPixConverges = Finder.IterateSubPixToConvergence(rightKF, 10);
        if(!bSubPixConverges) {
          cout << "subpix failed" << endl;
          continue;
        }

        vMatches.emplace_back(ir(v2RootPos), *it);

        // Now triangulate the 3d point...
        /*
        Vector<3> v3New =
          ReprojectPoint(se3RightCamFromLeft,
                         c.UnProject(Finder.GetSubPixPos()),
                         c.UnProject(vec(*it)));

        vv3PlanePoints.push_back(v3New);
        */

       // cout << v2Image << "  ->  "  << v2RootPos << " : " <<v3WorldPoint << " ---> " << v3New << endl;
       //vBackProjectedPts.push_back(ImageRef(v2Image[0] + 640, v2Image[1]));
        vBackProjectedPts.push_back(ImageRef(v2RootPos[0] + 640, v2RootPos[1]));
        vBackProjectedPts.push_back(*it);
//        vBackProjectedPts.push_back(ir(c.Project(project((v3New))) ));
      }
    }


    if (vv3PlanePoints.size() > 10) {
      StereoPlaneFinder spf;
      spf.Update(vv3PlanePoints);

      int n = vv3PlanePoints.size();

      Matrix<3> A = Zeros;
      Vector<3> b = Zeros;

      for (int i = 0; i < n; ++i) {
        const Vector<3> &p = vv3PlanePoints[i];

        A(0, 0) += p[0] * p[0];
        A(0, 1) += p[0] * p[1];
        A(0, 2) += p[0];
        A(1, 0) += p[0] * p[1]; // same as 0x1
        A(1, 1) += p[1] * p[1];
        A(1, 2) += p[1];
        A(2, 0) += p[0]; // Same as 0x2
        A(2, 1) += p[1]; // Same as 1x2

        b[0] += p[0] * p[2];
        b[1] += p[1] * p[2];
        b[2] += p[2];
      }

      A(2,2) = n;

      TooN::SVD<> svd(A);
      Vector<3> v3PlaneLSE = svd.backsub(b);
      Vector<4> v4PlaneLSE = makeVector(v3PlaneLSE[0], v3PlaneLSE[1], -1, v3PlaneLSE[2]);

      Vector<3> v3N = v4PlaneLSE.slice<0,3>();
      double d = sqrt(v3N * v3N);
      v4PlaneLSE *= -(1.0/d);

      mDrawData.v4GroundPlane = v4PlaneLSE; //spf.GetPlane();
      mDrawData.v4DispGroundPlane = mStereoPlaneFinder.GetPlane();
      cout << mStereoPlaneFinder.GetPlane() << endl;
      cout << spf.GetPlane() << endl;
      cout << v4PlaneLSE << endl;
      //mDrawData.v4GroundPlane[3] *= -1;
    } else {
      mDrawData.v4GroundPlane = Zeros;
      mDrawData.v4DispGroundPlane = mStereoPlaneFinder.GetPlane();
    }

    Vector<4> v4GroundPlane = mStereoPlaneFinder.GetPlane();
    v4GroundPlane[3] *= 0.01; // Scale from cm to meters


    mDrawData.vBackProjectedPts = vBackProjectedPts;


    if (bUserInvoke && vMatches.size() > 10) {
      SE3<> se3CurrentPose;
   //   mpMapMaker->InitFromStereo(rightKF, mKeyFrame, vMatches, &se3CurrentPose, false);
      mpMapMaker->InitFromKnownPlane(mKeyFrame, v4GroundPlane, se3CurrentPose);
      mpTracker->SetCurrentPose(se3CurrentPose);
      //mpTracker->ForceRecovery();
      mbInitialTracking = false;
    }

    mDrawData.bUseStereo = true;
    mDrawData.bInitialTracking = true;
    mDrawData.sStatusMessage = "Press spacebar to init";

  } else {
    // Initial tracking path
    if (bUserInvoke) {
      mpInitialTracker->UserInvoke();
    }

    mpInitialTracker->ProcessFrame(mKeyFrame);
    if (mpInitialTracker->IsDone()) {
      mpTracker->SetCurrentPose(mpInitialTracker->GetCurrentPose());
      mbInitialTracking = false;

      cout << "Inited to pose: " << mpInitialTracker->GetCurrentPose() << endl;
    }

    mDrawData.bUseStereo = false;
    mDrawData.bInitialTracking = true;
    mDrawData.sStatusMessage = mpInitialTracker->GetMessageForUser();
    mpInitialTracker->GetDrawData(mDrawData.initialTracker);
  }
}

void Frontend::DetermineScaleFromMarker(const FrameData& fd, bool bUserInvoke)
{
  mbSetScaleNextTime = mbSetScaleNextTime || bUserInvoke;

  SE3<> se3WorldFromNormWorld;
  double dScale = 1.0;
  if (mpScaleMarkerTracker->DetermineScaleFromMarker(fd.imFrameBW[0],
                                                     mpTracker->GetCurrentPose(),
                                                     se3WorldFromNormWorld, dScale))
  {
    // cout << "SCALE: " << dScale << endl;
    mse3MarkerPose = se3WorldFromNormWorld;


    if (mbSetScaleNextTime) {

      mpMapMaker->TransformMapPoints(mse3MarkerPose.inverse());
      mpMapMaker->ScaleMapPoints(dScale);

      SE3<> se3CamFromWorld = mpTracker->GetCurrentPose() * mse3MarkerPose;
      se3CamFromWorld.get_translation() *= dScale;
      mpTracker->SetCurrentPose(se3CamFromWorld);

      mbHasDeterminedScale = true;

      cout << "Scale is set! " <<  dScale << endl;
    }
  }


}

}
