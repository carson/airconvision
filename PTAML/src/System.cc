#include "System.h"
#include "OpenGL.h"
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"
#include "MapViewer.h"
#include "MapSerializer.h"
#include "Timing.h"
#include "MikroKopter.h"
#include "Frontend.h"
#include "InitialTracker.h"
#include "FrameGrabber.h"
#include "Utils.h"
#include "FrontendRenderer.h"

#include <gvars3/GStringUtil.h>
#include <cvd/image_io.h>

#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <functional>
#include <thread>
#include <stdexcept>

#ifdef _LINUX
#   include <fcntl.h>
#   include <pthread.h>
#endif

#ifdef WIN32
#   include <Windows.h>
#endif

namespace PTAMM {

using namespace std;
using namespace std::placeholders;
using namespace CVD;
using namespace GVars3;


string FeatureDetector2String(FeatureDetector featureDetector)
{
  switch (featureDetector) {
  case PLAIN_FAST7: return "PLAIN_FAST7";
  case PLAIN_FAST8: return "PLAIN_FAST8";
  case PLAIN_FAST9: return "PLAIN_FAST9";
  case PLAIN_FAST10: return "PLAIN_FAST10";
  case PLAIN_FAST11: return "PLAIN_FAST11";
  case PLAIN_FAST12: return "PLAIN_FAST12";
  case FAST10: return "FAST10";
  case OAST9_16: return "OAST9_16";
  case AGAST7_12d: return "AGAST7_12d";
  case AGAST7_12s: return "AGAST7_12s";
  case AGAST5_8: return "AGAST5_8";
  default: break;
  }

  return "Unknown";
}

Modules::Modules()
  : pFrameGrabber(NULL)
  , pRelocaliser(NULL)
  , pMapMaker(NULL)
  , pTracker(NULL)
  , pInitialTracker(NULL)
  , pScaleMarkerTracker(NULL)
  , pCamera(NULL)
  , pMapViewer(NULL)
  , pFrontend(NULL)
  , pMikroKopter(NULL)
  , pSwarmLab(NULL)
{
}

Modules::~Modules()
{
  delete pFrameGrabber;
  delete pRelocaliser;
  delete pMapMaker;
  delete pTracker;
  delete pInitialTracker;
  delete pScaleMarkerTracker;
  delete pCamera;
  delete pMapViewer;
  delete pFrontend;
  delete pMikroKopter;
  delete pSwarmLab;
}

System::System()
  : mGLWindow(ImageRef(640, 480), "PTAML")
  , mbDone(false)
  , mbDisableRendering(false)
{
  // Create the on-screen menu and register all the commands
  CreateMenu();
}

System::~System()
{
  if( mpMap != NULL )  {
    mpMap->mapLockManager.UnRegister( this );
  }
}

void System::CreateMenu()
{
  // Register all commands
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);

  GUI.RegisterCommand("Realign", GUICommandCallBack, this);

  GUI.RegisterCommand("LoadMap", GUICommandCallBack, this);
  GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);
  GUI.RegisterCommand("SaveMaps", GUICommandCallBack, this);

  GUI.RegisterCommand("ClearPath", GUICommandCallBack, this);
  GUI.RegisterCommand("FlyPath", GUICommandCallBack, this);
  GUI.RegisterCommand("AddWaypoint", GUICommandCallBack, this);
  GUI.RegisterCommand("PositionHold", GUICommandCallBack, this);

  GUI.RegisterCommand("ChangeFeatureDetector", GUICommandCallBack, this);

  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
  GUI.RegisterCommand("Mouse.Click", GUICommandCallBack, this);

  // Create the menus
  GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  GUI.ParseLine("Menu.AddMenuButton Root Realign Realign Root");
  GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
  GUI.ParseLine("DrawMap=0");
  GUI.ParseLine("DrawDebugInfo=0");
  GUI.ParseLine("Menu.AddMenuToggle Root \"Debug Info\" DrawDebugInfo Root");
  GUI.ParseLine("DrawPerfInfo=0");
  GUI.ParseLine("Menu.AddMenuToggle Root \"Perf. Info\" DrawPerfInfo Root");

  GUI.ParseLine("GLWindow.AddMenu InternalMenu Internal");
  GUI.ParseLine("InternalMenu.AddMenuButton Root \"Feature Detector\" ChangeFeatureDetector Root");

  GUI.ParseLine("GLWindow.AddMenu MapsMenu Maps");
  GUI.ParseLine("MapsMenu.AddMenuButton Root \"Serialize\" \"\" Serial");
  GUI.ParseLine("MapsMenu.AddMenuButton Serial \"Save Maps\" SaveMaps Root");
  GUI.ParseLine("MapsMenu.AddMenuButton Serial \"Save Map\" SaveMap Root");
  GUI.ParseLine("MapsMenu.AddMenuButton Serial \"Load Map\" LoadMap Root");
#ifdef _LINUX
  GUI.ParseLine("MapsMenu.AddMenuToggle Serial \"Save Video\" SaveFIFO Serial");
  GUI.ParseLine("MapsMenu.AddMenuSlider Serial Bitrate Bitrate 100 20000 Serial");
#endif
  GUI.ParseLine("LockMap=0");
  GUI.ParseLine("MapsMenu.AddMenuToggle Root \"Lock Map\" LockMap Root");

  GUI.ParseLine("GLWindow.AddMenu MapViewerMenu Viewer");
  GUI.ParseLine("MapViewerMenu.AddMenuToggle Root \"View Map\" DrawMap Root");

  GUI.ParseLine("EnableMouseControl=0");
  GUI.ParseLine("GLWindow.AddMenu HelicopterMenu Helicopter");
  GUI.ParseLine("HelicopterMenu.AddMenuToggle Root \"Mouse control\" EnableMouseControl Root");
  GUI.ParseLine("HelicopterMenu.AddMenuButton Root \"Clear path\" ClearPath Root");
  GUI.ParseLine("HelicopterMenu.AddMenuButton Root \"Fly path\" FlyPath Root");
  GUI.ParseLine("HelicopterMenu.AddMenuButton Root \"Add waypoint\" AddWaypoint Root");
  GUI.ParseLine("HelicopterMenu.AddMenuButton Root \"Position Hold\" PositionHold Root");
}

void System::CreateModules()
{
  mPerfMonitor.AddTimer("render");
  mPerfMonitor.AddTimer("main_loop");
  mPerfMonitor.AddTimer("track");
  mPerfMonitor.AddTimer("sbi_init");
  mPerfMonitor.AddTimer("sbi");
  mPerfMonitor.AddTimer("pvs");
  mPerfMonitor.AddTimer("track_fine");
  mPerfMonitor.AddTimer("track_coarse");
  mPerfMonitor.AddTimer("tracking_total");
  mPerfMonitor.AddTimer("grab_frame");

  mPerfMonitor.AddRateCounter("main");
  mPerfMonitor.AddRateCounter("render");
  mPerfMonitor.AddRateCounter("frontend");
  mPerfMonitor.AddRateCounter("mk");
  mPerfMonitor.AddRateCounter("frame_grabber");

  // First, check if the camera is calibrated.
  // If not, we need to run the calibration widget.
  Vector<NUMTRACKERCAMPARAMETERS> vTest;
  vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
  if(vTest == ATANCamera::mvDefaultParams) {
    cerr << endl;
    cerr << "! Camera.Parameters is not set, need to run the CameraCalibrator tool" << endl;
    cerr << "  and/or put the Camera.Parameters= line into the appropriate .cfg file." << endl;
    throw std::runtime_error("Missing camera parameters");
  }

  mModules.pFrameGrabber = new FrameGrabber(&mPerfMonitor);
  ImageRef irVideoSize = mModules.pFrameGrabber->GetFrameSize();

  if (mModules.pFrameGrabber->IsUsingStereo()) {
    ImageRef irWindowSize = irVideoSize;
    irWindowSize.x *= 2;
    mGLWindow.set_size(irWindowSize);
  } else {
    mGLWindow.set_size(irVideoSize);
  }

  mModules.pCamera = new ATANCamera("Camera");
  mModules.pCamera->SetImageSize(irVideoSize);

  if (!mARTracker.Init(irVideoSize)) {
    cerr << "Failed to initialize AR toolkit." << std::endl;
    throw std::runtime_error("Failed to initialize AR toolkit");
  }

  // Create the first map
  mpMap = new Map();

  // Create all the sub-systems
  mModules.pMapMaker = new MapMaker(mpMap);
  mModules.pMapViewer = new MapViewer(mpMap, mGLWindow);
  // mModules.pMapSerializer = new MapSerializer(mvpMaps);


  // TODO: move these into the frontend
  mModules.pRelocaliser = new Relocaliser(*mModules.pCamera);
  mModules.pTracker = new Tracker(irVideoSize,
                                  *mModules.pCamera,
                                  mpMap,
                                  mModules.pMapMaker,
                                  mModules.pRelocaliser,
                                  &mPerfMonitor);

  mModules.pInitialTracker = new InitialTracker(irVideoSize,
                                                *mModules.pCamera,
                                                mpMap,
                                                mModules.pMapMaker);

  mModules.pScaleMarkerTracker = new ScaleMarkerTracker(*mModules.pCamera,
                                                        mARTracker);


  mModules.pFrontend = new Frontend(mModules.pFrameGrabber,
                                    *mModules.pCamera,
                                    mModules.pMapMaker,
                                    mModules.pInitialTracker,
                                    mModules.pTracker,
                                    mModules.pScaleMarkerTracker,
                                    &mPerfMonitor);


  mModules.pMikroKopter = new MikroKopter(mModules.pTracker, &mPerfMonitor);
  mModules.pSwarmLab = new SwarmLab();
}

  // Run the main system thread.
  // This handles the tracker and the map viewer.
void System::Run()
{
  static gvar3<int> gvnLockMap("LockMap", 0, HIDDEN|SILENT);
  static gvar3<int> gvnEnableMikroKopter("EnableMikroKopter", 0, HIDDEN|SILENT);
  static gvar3<int> gvnEnableSwarmLab("EnableSwarmLab", 0, HIDDEN|SILENT);

  CreateModules();

  mModules.pFrontend->DoOnTrackedPoseUpdated(std::bind(
      &MikroKopter::UpdatePose, mModules.pMikroKopter,
      std::placeholders::_1, std::placeholders::_2));
  mModules.pFrontend->DoOnTrackedPoseUpdated(std::bind(
      &SwarmLab::UpdatePose, mModules.pSwarmLab,
      std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  // Start threads
  std::thread mapMakerThread(std::ref(*mModules.pMapMaker));
  std::thread frontendThread(std::ref(*mModules.pFrontend));
  std::thread mikroKopterThread(std::ref(*mModules.pMikroKopter));
  std::thread frameGrabberThread(std::ref(*mModules.pFrameGrabber));
  std::thread swarmThread(std::ref(*mModules.pSwarmLab));

  // System: main loop
  while (!mbDone) {
    // Check if the map has been locked by another thread, and wait for release.
    // bool bWasLocked = mpMap->mapLockManager.CheckLockAndWait( this, 0 );

    mPerfMonitor.StartTimer("main_loop");

    mpMap->bEditLocked = *gvnLockMap; //sync up the maps edit lock with the gvar bool.

    // Get the latest value of the tracked pose
    mse3CurrentPose = mModules.pFrontend->monitor.GetCurrentPose();

    if (!mbDisableRendering) {
      Draw();

#ifdef _LINUX
      static gvar3<int> gvnSaveFIFO("SaveFIFO", 0, HIDDEN|SILENT);
      if (*gvnSaveFIFO) {
        SaveFIFO();
      }
#endif
    }

    mGLWindow.HandlePendingEvents();

    mPerfMonitor.StopTimer("main_loop");
    mPerfMonitor.UpdateRateCounter("main");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  mModules.pMapMaker->StopThread();
  mModules.pFrontend->StopThread();
  mModules.pMikroKopter->StopThread();
  mModules.pFrameGrabber->StopThread();
  mModules.pSwarmLab->StopThread();

  mapMakerThread.join();
  frontendThread.join();
  mikroKopterThread.join();
  frameGrabberThread.join();
  swarmThread.join();
}


  // Parse commands sent via the GVars command system.
  // @param ptr Object callback
  // @param sCommand command string
  // @param sParams parameters

void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  System* pSystem = static_cast<System*>(ptr);
  pSystem->GUICommandCallBack(sCommand, sParams);
}

void System::GUICommandCallBack(const string &sCommand, const string &sParams)
{
  if( sCommand=="quit" || sCommand == "exit" ) {
    Quit();
  }
  else if(sCommand == "Realign") {
    mModules.pMapMaker->RealignGroundPlane(true);
  }
  else if( sCommand == "SaveMap" || sCommand == "SaveMaps" || sCommand == "LoadMap")  {
    StartMapSerialization( sCommand, sParams );
  }
  else if( sCommand == "PositionHold")  {
    PositionHold();
  }
  else if( sCommand == "AddWaypoint")  {
    AddWaypoint();
  }
  else if( sCommand == "ClearWaypoints")  {
    ClearWaypoints();
  }
  else if( sCommand == "ChangeFeatureDetector") {
    ChangeFeatureDetector();
  }
  else if( sCommand == "Mouse.Click" ) {
    vector<string> vs = ChopAndUnquoteString(sParams);
    if( vs.size() != 3 ) {
      return;
    }

    istringstream is(sParams);
    int nButton;
    ImageRef irWin;
    is >> nButton >> irWin.x >> irWin.y;

    HandleClick(nButton, irWin);
  }
  else if( sCommand == "KeyPress" )
  {
    if(sParams == "g") {
      mbDisableRendering = !mbDisableRendering;
    }
    else if(sParams == "q" || sParams == "Escape") {
      GUI.ParseLine("quit");
    }
    else if(sParams == "r") {
      mModules.pFrontend->monitor.PushUserResetInvoke();
    }
    else if(sParams == "Space") {
      mModules.pFrontend->monitor.PushUserInvoke();
    }
    else if(sParams == "f") {
      mModules.pFrameGrabber->SetFreezeFrame(!mModules.pFrameGrabber->IsFrameFrozen());
    }
  }
}

void System::HandleClick(int nButton, const CVD::ImageRef &irWin)
{
  static gvar3<int> gvnEnableMouseControl("EnableMouseControl", 0, HIDDEN|SILENT);
  if (*gvnEnableMouseControl) {
    Vector<3> v3PointOnPlane;
    if (PickPointOnGround(*mModules.pCamera, mse3CurrentPose,
                          makeVector(irWin.x, irWin.y), v3PointOnPlane))
    {
      Vector<2> v2TargetLocation;
      v2TargetLocation[0] = v3PointOnPlane[0];
      v2TargetLocation[1] = v3PointOnPlane[1];
      mModules.pMikroKopter->GoToLocation(v2TargetLocation);
    }
  }
}

  // Set up the map serialization thread for saving/loading and the start the thread
  // @param sCommand the function that was called (eg. SaveMap)
  // @param sParams the params string, which may contain a filename and/or a map number

void System::StartMapSerialization(std::string sCommand, std::string sParams)
{
  if (mModules.pMapSerializer->Init(sCommand, sParams, *mpMap)) {
    mModules.pMapSerializer->start();
  }
}

void System::PositionHold()
{
  // TBD
}

void System::AddWaypoint()
{
  // TBD
}

void System::ClearWaypoints()
{
  // TBD
}

void System::FlyPath()
{
  // TBD
}

void System::ChangeFeatureDetector()
{
  int &current = GV3::get<int>("FeatureDetector", 0);
  if (current >= NUM_FEATURE_DETECTORS - 1) {
    current = 0;
  } else {
    ++current;
  }
}

void System::Draw()
{
  mPerfMonitor.StartTimer("render");

  static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN|SILENT);
  bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;

  if (!bDrawMap && !mModules.pFrontend->monitor.PopDrawData(mFrontendDrawData)) {
    return;
  }

  mGLWindow.SetupViewport();
  mGLWindow.SetupVideoOrtho();
  mGLWindow.SetupVideoRasterPosAndZoom();

  string sCaption;

  if(bDrawMap) {
    // TODO: This is not thread safe at all.
    // The whole mapviewer thing is in a rather bad state...
    mModules.pMapViewer->DrawMap(mse3CurrentPose);
    sCaption = mModules.pMapViewer->GetMessageForUser();
  } else {
    FrontendRenderer renderer(*mModules.pCamera, mFrontendDrawData);
    renderer.Draw();
    sCaption = mFrontendDrawData.sStatusMessage;
  }

  static gvar3<int> gvnDrawPerfInfo("DrawPerfInfo", 0, HIDDEN|SILENT);
  if(*gvnDrawPerfInfo) {
    DrawPerfInfo();
  }

  static gvar3<int> gvnDrawDebugInfo("DrawDebugInfo", 0, HIDDEN|SILENT);
  if (*gvnDrawDebugInfo) {
    DrawDebugInfo();
  }

  mGLWindow.DrawCaption(sCaption);
  mGLWindow.DrawMenus();

  mGLWindow.swap_buffers();

  mPerfMonitor.UpdateRateCounter("render");
  mPerfMonitor.StopTimer("render");
}

void System::DrawPerfInfo()
{
  int nLines = 15;
  int x = 5, y = 120, w = 160, nBorder = 5;

  mGLWindow.DrawBox( x, y, w, nLines, 0.7f );

  glColor3f(1,1,1);
  std::ostringstream os;

  std::vector<std::pair<std::string, double>> vNameVal;

  os << setiosflags(ios::fixed) << setprecision(2)
     << "Rates (Hz):\n";

  mPerfMonitor.QueryAllRates(vNameVal);
  for (auto it = vNameVal.begin(); it != vNameVal.end(); ++it) {
    os << " " << it->first << ": " << it->second << "\n";
  }

  os << "\nTimers (ms):\n";


  mPerfMonitor.QueryAllTimers(vNameVal);
  for (auto it = vNameVal.begin(); it != vNameVal.end(); ++it) {
    os << " " << it->first << ": " << it->second * 1000.0 << "\n";
  }

  glColor3f(1,1,0);
  mGLWindow.PrintString( ImageRef( x + nBorder , y + nBorder + 17), os.str() );
}

void System::DrawDebugInfo()
{
  int nLines = 9;
  int x = 295, y = 300, w = 340, nBorder = 5;

  mGLWindow.DrawBox( x, y, w, nLines, 0.7f );

  stringstream os;

  os << "Position: " << std:: fixed << std::setprecision(3) << mModules.pMikroKopter->GetPosInWorld() << endl << endl;
  os << "EulerAngles: " << mModules.pMikroKopter->GetEulerAngles() << endl;
  os << "Velocity: " << mModules.pMikroKopter->GetVelocity() << endl;
  os << "Target: " << mModules.pMikroKopter->GetTargetOffset() << endl << endl;
  os << "Control: " << mModules.pMikroKopter->GetControl()[0] << " "
    << mModules.pMikroKopter->GetControl()[1] << " "
    << mModules.pMikroKopter->GetControl()[2] << " "
    << mModules.pMikroKopter->GetControl()[3] << " "
    << mModules.pMikroKopter->GetControl()[4] << endl;
  os << "Controller Config: " << (int)mModules.pMikroKopter->GetConfig() << endl << endl;
  os << "Debug: " << mModules.pMikroKopter->GetMKData()[0] << " "
    << mModules.pMikroKopter->GetMKData()[1] << " "
    << mModules.pMikroKopter->GetMKData()[2] << endl << endl;

  glColor3f(1,1,0);
  mGLWindow.PrintString( ImageRef( x + nBorder , y + nBorder + 17), os.str() );
}


  // Save the current frame to a FIFO.
  // This function is called on each frame to create a video.
  // The GVar SaveFIFO starts and stops the saving, and the GVar
  // Bitrate sets the quality.
  // Bitrate can only be set before the first call of SaveFIFO.

void System::SaveFIFO()
{
#ifdef _LINUX
  //Some static variables
  static CVD::byte* pcImage = NULL;
  static int fd = 0;
  static bool bFIFOInitDone = false;
  static ImageRef irWindowSize;

  const int BITRATE = 1500;

  if( !bFIFOInitDone )
  {
    irWindowSize = mGLWindow.size();

    ostringstream os;
    os << /*"/bin/bash\n" <<*/
        "file=\"`date '+%Y-%m-%d_%H-%M-%S'`.avi\"; " <<
        "if [ ! -e FIFO ]; then mkfifo FIFO; echo Made FIFO...; fi; " <<
        "echo Mencoding to $file....; " <<
        "cat FIFO |nice mencoder -flip -demuxer rawvideo -rawvideo fps=10:w=" <<
        irWindowSize.x << ":h=" << irWindowSize.y <<
        ":format=rgb24 -o $file -ovc lavc -lavcopts vcodec=mpeg4:vbitrate=" << BITRATE <<
        ":keyint=45 -ofps 30 -ffourcc DIVX - &";

    cout << "::" << os.str()<< "::" << endl;
    int i = system( os.str().c_str() );
    if( i != 0 ) {
      cerr << "ERROR: could not set up the FIFO!" << endl;
      return;
    }

    if (posix_memalign((void**)(&pcImage), 16, irWindowSize.x*irWindowSize.y*3) != 0) {
      return;
    }

    string s = "FIFO";
    fd = open(s.c_str(), O_RDWR | O_ASYNC);

    bFIFOInitDone = true;
  }

  if( irWindowSize != mGLWindow.size() )
  {
    cerr << "ERROR: Aborting FIFO as window size has changed!!" << endl;
    //*mgvnSaveFIFO = 0;
    return;
  }

  glReadBuffer(GL_BACK);
  glReadPixels(0,0,irWindowSize.x,irWindowSize.y,GL_RGB, GL_UNSIGNED_BYTE, pcImage);
  if (write(fd, (char*) pcImage, irWindowSize.x*irWindowSize.y*3) < 0) {
    return;
  }

#else
  cout << "Video Saving using FIFOs is only available under Linux" << endl;
#endif

}

}
