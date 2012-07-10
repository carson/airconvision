// Copyright 2009 Isis Innovation Limited
#include "System.h"
#include "OpenGL.h"
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"
#include "MapViewer.h"
#include "MapSerializer.h"
#include "FPSCounter.h"
#include "Timing.h"
#include "MikroKopter.h"
#include "Frontend.h"
#include "InitialTracker.h"
#include "ScaleMarkerTracker.h"
#include "LevelHelpers.h"
#include "FrameGrabber.h"
#include "Utils.h"

#include <gvars3/GStringUtil.h>
#include <cvd/image_io.h>

#include <cstdlib>
#include <iostream>
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

TimingTimer gFrameTimer;
TimingTimer gVideoSourceTimer;
TimingTimer gFeatureTimer;
TimingTimer gPvsTimer;
TimingTimer gCoarseTimer;
TimingTimer gFineTimer;
TimingTimer gTrackTimer;
TimingTimer gSBIInitTimer;
TimingTimer gSBITimer;
TimingTimer gTrackingQualityTimer;
TimingTimer gDrawGridTimer;
TimingTimer gDrawUITimer;
TimingTimer gGLSwapTimer;
TimingTimer gTrackFullTimer;

using namespace std;
using namespace std::placeholders;
using namespace CVD;
using namespace GVars3;

/**
 * Parse and allocate a single integer variable from a string parameter
 * @param nAnswer the result
 * @param sCommand the command (used to display usage info)
 * @param sParams  the parameters to parse
 * @return success or failure.
 */
bool GetSingleParam(int &nAnswer, string sCommand, string sParams)
{
  vector<string> vs = ChopAndUnquoteString(sParams);

  if(vs.size() == 1)
  {
    //is param a number?
    bool bIsNum = true;
    for( size_t i = 0; i < vs[0].size(); i++ ) {
      bIsNum = isdigit( vs[0][i] ) && bIsNum;
    }

    if( !bIsNum )
    {
      return false;
    }

    int *pN = ParseAndAllocate<int>(vs[0]);
    if( pN )
    {
      nAnswer = *pN;
      delete pN;
      return true;
    }
  }

  cout << sCommand << " usage: " << sCommand << " value" << endl;

  return false;
}

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

class FrontendRenderer {
  public:
    FrontendRenderer(const ATANCamera &camera, const FrontendDrawData &drawData)
      : mDrawData(drawData)
      , mCamera(camera)
      , mse3CamFromWorld(drawData.tracker.se3CamFromWorld)
    {
    }

    void Draw();

  private:
    Vector<2> ProjectPoint(const SE3<> &se3CamFromWorld, const Vector<3> &v3Point);
    void DrawTrails(const std::vector<std::pair<CVD::ImageRef, CVD::ImageRef>> &vTrails,
                    const std::vector<CVD::ImageRef> &vDeadTrails);

    void DrawCorners(const std::vector<CVD::ImageRef> &vCorners);
    void DrawGrid(const SE3<> &se3CamFromWorld);
    void DrawMapPoints(const std::vector<std::pair<int, TooN::Vector<2> >> &vMapPoints);
    void DrawMarkerPose(const SE3<> &se3WorldFromNormWorld);

  private:
    const FrontendDrawData &mDrawData;
    ATANCamera mCamera;
    SE3<> mse3CamFromWorld;
};

Vector<2> FrontendRenderer::ProjectPoint(const SE3<> &se3CamFromWorld, const Vector<3> &v3Point)
{
  Vector<3> v3Cam = se3CamFromWorld * v3Point;

  if(v3Cam[2] < 0.001) {
    v3Cam[2] = 0.001;
  }

  return mCamera.Project(project(v3Cam));
}

void FrontendRenderer::DrawTrails(const std::vector<std::pair<CVD::ImageRef, CVD::ImageRef>> &vTrails,
                                  const std::vector<CVD::ImageRef> &vDeadTrails)
{
  glPointSize(5);
  glLineWidth(2);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  glBegin(GL_LINES);
  for (auto i = vTrails.begin(); i != vTrails.end(); ++i) {
    glColor3f(1,1,0);
    glVertex(i->first);
    glColor3f(1,0,0);
    glVertex(i->second);
  }
  glEnd();

  glBegin(GL_POINTS);
  for (auto it = vDeadTrails.begin(); it != vDeadTrails.end(); ++it) {
    glColor3f(0.5,0.1,0.7);
    glVertex(*it);
  }
  glEnd();
}

void FrontendRenderer::DrawCorners(const std::vector<CVD::ImageRef> &vCorners)
{
  glColor3f(1,0,1);
  glPointSize(1);
  glBegin(GL_POINTS);
  for (auto c = vCorners.begin(); c != vCorners.end(); ++c) {
    glVertex(*c);
  }
  glEnd();
}

void FrontendRenderer::DrawGrid(const SE3<> &se3CamFromWorld)
{
  // The grid is projected manually, i.e. GL receives projected 2D coords to draw.
  int nHalfCells = 4;
  int nTot = nHalfCells * 2 + 1;
  Image<Vector<2> >  imVertices(ImageRef(nTot,nTot));
  for(int i=0; i<nTot; i++)
  {
    for(int j=0; j<nTot; j++)
    {
      Vector<3> v3;
      v3[0] = (i - nHalfCells) * 0.1;
      v3[1] = (j - nHalfCells) * 0.1;
      v3[2] = 0.0;
      imVertices[i][j] = ProjectPoint(se3CamFromWorld, v3);
    }
  }

  glDisable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glLineWidth(1);
  for(int i=0; i<nTot; i++)
  {
    glBegin(GL_LINE_STRIP);
    for(int j=0; j<nTot; j++)
      glVertex(imVertices[i][j]);
    glEnd();

    glBegin(GL_LINE_STRIP);
    for(int j=0; j<nTot; j++)
      glVertex(imVertices[j][i]);
    glEnd();
  }

  glLineWidth(1);
  glColor3f(1,0,0);
}

void FrontendRenderer::DrawMapPoints(const std::vector<std::pair<int, TooN::Vector<2>>> &vMapPoints)
{
  glPointSize(3);
  glDisable(GL_BLEND);
  glBegin(GL_POINTS);
  for(auto it = vMapPoints.begin(); it != vMapPoints.end(); ++it) {
    glColor(gavLevelColors[it->first]);
    glVertex(it->second);
  }
  glEnd();
}

void FrontendRenderer::DrawMarkerPose(const SE3<> &se3WorldFromNormWorld)
{
  glEnable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
  glLineWidth(2);
  glPointSize(15);

  glBegin(GL_POINTS);
    glColor3f(0,1,1);
    glVertex(ProjectPoint(mse3CamFromWorld, se3WorldFromNormWorld.get_translation()));
  glEnd();

  Vector<3> wo2 = se3WorldFromNormWorld * makeVector(0, 0, 0);
  Vector<3> wx2 = se3WorldFromNormWorld * makeVector(8, 0, 0);
  Vector<3> wy2 = se3WorldFromNormWorld * makeVector(0, 8, 0);
  Vector<3> wz2 = se3WorldFromNormWorld * makeVector(0, 0, 8);

  std::vector<Vector<3> > pts;

  pts.push_back(wo2); pts.push_back(wx2);
  pts.push_back(wo2); pts.push_back(wy2);
  pts.push_back(wo2); pts.push_back(wz2);

  glBegin(GL_LINES);

  std::vector<Vector<2> > screenPts;
  for (auto it = pts.begin(); it != pts.end(); ++it) {
    glColor3f(1,0,0);
    glVertex(ProjectPoint(mse3CamFromWorld, *it));
  }

  glEnd();

  glLineWidth(1);
  glPointSize(1);
}

void FrontendRenderer::Draw()
{
  glColor4f(1,1,1,1);
  glDrawPixels(mDrawData.imFrame);

  if (mDrawData.bInitialTracking) {
    if (GV3::get<int>("Tracker.DrawFASTCorners",0, SILENT)) {
      DrawCorners(mDrawData.initialTracker.vCorners);
    }

    DrawTrails(mDrawData.initialTracker.vTrails, mDrawData.initialTracker.vDeadTrails);

    Vector<3> v3PointOnPlane;
    if (PickPointOnPlane(mCamera, mDrawData.v4GroundPlane,
    		             makeVector(320, 240), v3PointOnPlane))
    {
		v3PointOnPlane *= -0.02;
		Vector<3> v3Normal = mDrawData.v4GroundPlane.slice<0, 3>();
		SE3<> se3AlignedPlane = AlignerFromPointAndUp(v3PointOnPlane, v3Normal);
		SE3<> se3CamFromPlane = se3AlignedPlane.inverse();

		glColor3f(1,1,1);
		DrawGrid(se3CamFromPlane);
    }

  } else {
    if (GV3::get<int>("Tracker.DrawFASTCorners",0, SILENT)) {
      DrawCorners(mDrawData.tracker.vCorners);
    }

    // The colour of the ref grid shows if the coarse stage of tracking was used
    // (it's turned off when the camera is sitting still to reduce jitter.)
    if (mDrawData.tracker.bDidCoarse) {
      glColor4f(0.0f, 0.5f, 0.0f, 0.6f);
    } else {
      glColor4f(0.0f,0.0f,0.0f,0.6f);
    }

    DrawGrid(mse3CamFromWorld);

    // Draw all the matched map points
    DrawMapPoints(mDrawData.tracker.vMapPoints);
  }
}


System::System()
  : mGLWindow(ImageRef(640, 480), "PTAML")
  , mbDone(false)
  , mbDisableRendering(false)
{
  // Create the on-screen menu and register all the commands
  CreateMenu();

  // Force the program to run on CPU0
  /*
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(0, &cpuset);
  pthread_t thread = pthread_self();
  if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset) != 0) {
    cerr << "pthread_setaffinity_np failed for main thread" << endl;
  }
  */
}

/**
 * Destructor
 */
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
  GUI.ParseLine("Menu.ShowMenu Root");
  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  GUI.ParseLine("Menu.AddMenuButton Root Realign Realign Root");
  GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
  GUI.ParseLine("DrawMap=0");
  GUI.ParseLine("DrawMKDebug=1");
  GUI.ParseLine("Menu.AddMenuToggle Root \"Show debug\" DrawMKDebug Root");

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
  GUI.ParseLine("MapInfo=0");
  GUI.ParseLine("MapsMenu.AddMenuToggle Root \"Map Info\" MapInfo Root");

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

  mModules.pFrameGrabber = new FrameGrabber();
  ImageRef irVideoSize = mModules.pFrameGrabber->GetFrameSize();

  mModules.pCamera = new ATANCamera("Camera");
  mModules.pCamera->SetImageSize(irVideoSize);

  if (!mARTracker.Init(irVideoSize)) {
    cerr << "Failed to init AR toolkit." << std::endl;
    throw std::runtime_error("Failed to init AR toolkit");
  }

  //create the first map
  mpMap = new Map();

  // Create all the sub-systems
  mModules.pMapMaker = new MapMaker(mpMap);
  mModules.pMapViewer = new MapViewer(mpMap, mGLWindow);
//  mModules.pMapSerializer = new MapSerializer(mvpMaps);


  // Move these into the frontend
  mModules.pRelocaliser = new Relocaliser(*mModules.pCamera);
  mModules.pTracker = new Tracker(irVideoSize, *mModules.pCamera, mpMap, mModules.pMapMaker, mModules.pRelocaliser);
  mModules.pInitialTracker = new InitialTracker(irVideoSize, *mModules.pCamera, mpMap, mModules.pMapMaker);
  mModules.pScaleMarkerTracker = new ScaleMarkerTracker(*mModules.pCamera, mARTracker);


  mModules.pFrontend = new Frontend(mModules.pFrameGrabber, *mModules.pCamera,
                                    mModules.pMapMaker,
                                    mModules.pInitialTracker,
                                    mModules.pTracker,
                                    mModules.pScaleMarkerTracker);


  mModules.pMikroKopter = new MikroKopter(mModules.pTracker);
}

/**
 * Run the main system thread.
 * This handles the tracker and the map viewer.
 */
void System::Run()
{
  static gvar3<int> gvnLockMap("LockMap", 0, HIDDEN|SILENT);

  CreateModules();

  // Start threads
  std::thread mapMakerThread(std::ref(*mModules.pMapMaker));
  std::thread frontendThread(std::ref(*mModules.pFrontend));
  std::thread mikroKopterThread(std::ref(*mModules.pMikroKopter));

  FPSCounter fpsCounter;

  while(!mbDone)
  {
    //Check if the map has been locked by another thread, and wait for release.
    //bool bWasLocked = mpMap->mapLockManager.CheckLockAndWait( this, 0 );

    gFrameTimer.Start();

    mpMap->bEditLocked = *gvnLockMap; //sync up the maps edit lock with the gvar bool.

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

    // Update FPS counter, this should be the last thing in the main loop
    if (fpsCounter.Update()) {
      stringstream ss; ss << "PTAML - " << setiosflags(ios::fixed) << setprecision(2) << fpsCounter.Fps() << " fps";
      mGLWindow.set_title(ss.str());
    }

    gFrameTimer.Stop();

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

/**
 * Parse commands sent via the GVars command system.
 * @param ptr Object callback
 * @param sCommand command string
 * @param sParams parameters
 */
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
      // TODO: Create a "VideoGrabber" class and add this as a function call
      //mbFreezeVideo = !mbFreezeVideo;
    }
  }
}

void System::HandleClick(int nButton, const CVD::ImageRef &irWin)
{
  static gvar3<int> gvnEnableMouseControl("EnableMouseControl", 0, HIDDEN|SILENT);
  if (*gvnEnableMouseControl) {
    Vector<3> v3PointOnPlane;
    if (PickPointOnGround(*mModules.pCamera, mModules.pTracker->GetCurrentPose(),
                          makeVector(irWin.x, irWin.y), v3PointOnPlane))
    {
      // Set the targets Z value same as the current positions
      v3PointOnPlane[2] = mModules.pTracker->GetCurrentPose().inverse().get_translation()[2];

      SE3<> se3TargetPose;
      se3TargetPose.get_translation() = v3PointOnPlane;
      mModules.pMikroKopter->GoToPosition(se3TargetPose);
    }
  }
}

/**
 * Set up the map serialization thread for saving/loading and the start the thread
 * @param sCommand the function that was called (eg. SaveMap)
 * @param sParams the params string, which may contain a filename and/or a map number
 */
void System::StartMapSerialization(std::string sCommand, std::string sParams)
{
  if (mModules.pMapSerializer->Init(sCommand, sParams, *mpMap)) {
    mModules.pMapSerializer->start();
  }
}

void System::PositionHold()
{
  mModules.pMikroKopter->GoToPosition(mModules.pTracker->GetCurrentPose().inverse());
}

void System::AddWaypoint()
{
  mModules.pMikroKopter->AddWaypoint(mModules.pTracker->GetCurrentPose().inverse());
}

void System::ClearWaypoints()
{
  mModules.pMikroKopter->ClearWaypoints();
}

void System::FlyPath()
{
  mModules.pMikroKopter->FlyPath();
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
  gDrawUITimer.Start();

  static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN|SILENT);
  bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;

  mGLWindow.SetupViewport();
  mGLWindow.SetupVideoOrtho();
  mGLWindow.SetupVideoRasterPosAndZoom();

  string sCaption;

  if(bDrawMap) {
    // TODO: This is not thread safe at all... The whole mapviewer thing is in a rather bad state...
    mModules.pMapViewer->DrawMap(mModules.pTracker->GetCurrentPose());
    sCaption = mModules.pMapViewer->GetMessageForUser();
  } else {
    if (mModules.pFrontend->monitor.PopDrawData(mFrontendDrawData)) {
      FrontendRenderer renderer(*mModules.pCamera, mFrontendDrawData);
      renderer.Draw();
      sCaption = mFrontendDrawData.sStatusMessage;
    }
  }

  static gvar3<int> gvnDrawMapInfo("MapInfo", 0, HIDDEN|SILENT);
  if(*gvnDrawMapInfo) {
    DrawMapInfo();
  }

  static gvar3<int> gvnDrawMkDebugOutput("DrawMKDebug", 0, HIDDEN|SILENT);
  if (*gvnDrawMkDebugOutput) {
    DrawDebugInfo();
  }

  mGLWindow.DrawCaption(sCaption);
  mGLWindow.DrawMenus();

  gDrawUITimer.Stop();

  gGLSwapTimer.Start();
  mGLWindow.swap_buffers();
  gGLSwapTimer.Stop();
}

void System::DrawDebugInfo()
{
  stringstream ss;


  FeatureDetector featureDetector = (FeatureDetector)GV3::get<int>("FeatureDetector", 0);
  ss << "Features: " << FeatureDetector2String(featureDetector) << endl << endl;

  ss << "Frame: " << gFrameTimer.Milliseconds() << endl
     << "Video: " << gVideoSourceTimer.Milliseconds() << endl
     << "Feature: " << gFeatureTimer.Milliseconds() << endl
     << "PVS: " << gPvsTimer.Milliseconds() << endl
     << "Coarse: " << gCoarseTimer.Milliseconds() << endl
     << "Fine: " << gFineTimer.Milliseconds() << endl
     << "Track: " << gTrackTimer.Milliseconds() << endl
     << "FullTrack: " << gTrackFullTimer.Milliseconds() << endl
     //<< "SBI Init: " << gSBIInitTimer.Milliseconds() << endl
     //<< "SBI: " << gSBITimer.Milliseconds() << endl
     //<< "TrackQ: " << gTrackingQualityTimer.Milliseconds() << endl
     //<< "Grid: " << gDrawGridTimer.Milliseconds() << endl
     << "UI: " << gDrawUITimer.Milliseconds() << endl;
     //<< "GLSwap: " << gGLSwapTimer.Milliseconds() << endl;


  /*
  ss << "X: " << mPositionHold.GetTargetOffsetFiltered()[0] << "\n"
     << "Y: " << mPositionHold.GetTargetOffsetFiltered()[1] << "\n"
     << "VX: " << mPositionHold.GetVelocityFiltered()[0] << "\n"
     << "VY: " << mPositionHold.GetVelocityFiltered()[1];
*/

  mGLWindow.DrawDebugOutput(ss.str());
}


/**
 * Draw a box with information about the maps.
 */
void System::DrawMapInfo()
{
  int nLines = 3;
  int x = 5, y = 120, w = 160, nBorder = 5;

  mGLWindow.DrawBox( x, y, w, nLines, 0.7f );

  y += 17;

  glColor3f(1,1,1);
  std::ostringstream os;

  os << "M: " << mpMap->MapID() << "  P: " << mpMap->GetMapPoints().size() << "  K: " << mpMap->GetKeyFrames().size();

  glColor3f(1,1,0);
  mGLWindow.PrintString( ImageRef( x + nBorder , y + nBorder + 17), os.str() );
}

/**
 * Save the current frame to a FIFO.
 * This function is called on each frame to create a video.
 * The GVar SaveFIFO starts and stops the saving, and the GVar
 * Bitrate sets the quality.
 * Bitrate can only be set before the first call of SaveFIFO.
 */
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
        "cat FIFO |nice mencoder -flip -demuxer rawvideo -rawvideo fps=30:w=" <<
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
    *mgvnSaveFIFO = 0;
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

