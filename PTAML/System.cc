// Copyright 2009 Isis Innovation Limited
#include "System.h"
#include "OpenGL.h"
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"
#include "ARDriver.h"
#include "MapViewer.h"
#include "MapSerializer.h"
#include "FPSCounter.h"
#include "Timing.h"
#include "MikroKopter.h"

#include <gvars3/GStringUtil.h>
#include <cvd/image_io.h>

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <functional>
#include <thread>

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


System::System(VideoSource* videoSource)
  : mGLWindow(videoSource->Size(), "PTAML")
  , mVideoSource(videoSource)
  , mbFreezeVideo(false)
  , mbDone(false)
{
  GV3::Register(mgvnLockMap, "LockMap", 0, SILENT);
  GV3::Register(mgvnDrawMapInfo, "MapInfo", 0, SILENT);
  GV3::Register(mgvnDisableRendering, "DisableRendering", 0, SILENT);

#ifdef _LINUX
  GV3::Register(mgvnSaveFIFO, "SaveFIFO", 0, SILENT);
  GV3::Register(mgvnBitrate, "Bitrate", 15000, SILENT);
#endif

  ImageRef irVideoSize = videoSource->Size();

  mimFrameBW.resize(irVideoSize);
  mimFrameRGB.resize(irVideoSize);

  // First, check if the camera is calibrated.
  // If not, we need to run the calibration widget.
  Vector<NUMTRACKERCAMPARAMETERS> vTest;
  vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
  if(vTest == ATANCamera::mvDefaultParams) {
    cout << endl;
    cout << "! Camera.Parameters is not set, need to run the CameraCalibrator tool" << endl;
    cout << "  and/or put the Camera.Parameters= line into the appropriate .cfg file." << endl;
    exit(1);
  }

  mpCamera = new ATANCamera("Camera");
  mpCamera->SetImageSize(irVideoSize);

  if (!mARTracker.Init(irVideoSize)) {
    cout << "Failed to init AR toolkit." << std::endl;
    exit(1);
  }

  //create the first map
  mpMap = new Map();
  mvpMaps.push_back( mpMap );
  mpMap->mapLockManager.Register(this);

  // Create all the sub-systems
  mpMapMaker = new MapMaker(mvpMaps, mpMap);
  mpTracker = new Tracker(irVideoSize, *mpCamera, mvpMaps, mpMap, *mpMapMaker, mARTracker);
  mpARDriver = new ARDriver(*mpCamera, irVideoSize, mGLWindow, *mpMap);
  mpMapViewer = new MapViewer(mvpMaps, mpMap, mGLWindow);
  mpMapSerializer = new MapSerializer(mvpMaps);
  mpMikroKopter = new MikroKopter();

  mCoordinateLogFile.open("coordinates.txt", ios::out | ios::trunc);
  if (!mCoordinateLogFile) {
    cerr << "Failed to open coordinates.txt" << endl;
  }

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

  GUI.RegisterCommand("SwitchMap", GUICommandCallBack, this);
  GUI.RegisterCommand("NewMap", GUICommandCallBack, this);
  GUI.RegisterCommand("DeleteMap", GUICommandCallBack, this);
  GUI.RegisterCommand("ResetAll", GUICommandCallBack, this);
  GUI.RegisterCommand("Realign", GUICommandCallBack, this);

  GUI.RegisterCommand("LoadMap", GUICommandCallBack, this);
  GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);
  GUI.RegisterCommand("SaveMaps", GUICommandCallBack, this);

  GUI.RegisterCommand("NextMap", GUICommandCallBack, this);
  GUI.RegisterCommand("PrevMap", GUICommandCallBack, this);
  GUI.RegisterCommand("CurrentMap", GUICommandCallBack, this);

  GUI.RegisterCommand("ClearPath", GUICommandCallBack, this);
  GUI.RegisterCommand("FlyPath", GUICommandCallBack, this);
  GUI.RegisterCommand("AddWaypoint", GUICommandCallBack, this);
  GUI.RegisterCommand("PositionHold", GUICommandCallBack, this);

  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);
  GUI.RegisterCommand("Mouse.Click", GUICommandCallBack, this);

  // Create the menus
  GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GUI.ParseLine("Menu.ShowMenu Root");
  GUI.ParseLine("Menu.AddMenuButton Root \"Reset All\" ResetAll Root");
  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  GUI.ParseLine("Menu.AddMenuButton Root Realign Realign Root");
  GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
  GUI.ParseLine("DrawMap=0");
  GUI.ParseLine("DrawMKDebug=1");
  GUI.ParseLine("Menu.AddMenuToggle Root \"Draw MK\" DrawMKDebug Root");

  GUI.ParseLine("GLWindow.AddMenu MapsMenu Maps");
  GUI.ParseLine("MapsMenu.AddMenuButton Root \"New Map\" NewMap Root");
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
  GUI.ParseLine("MapsMenu.AddMenuButton Root \"Delete Map\" DeleteMap Root");
  GUI.ParseLine("MapInfo=0");
  GUI.ParseLine("MapsMenu.AddMenuToggle Root \"Map Info\" MapInfo Root");

  GUI.ParseLine("GLWindow.AddMenu MapViewerMenu Viewer");
  GUI.ParseLine("MapViewerMenu.AddMenuToggle Root \"View Map\" DrawMap Root");
  GUI.ParseLine("MapViewerMenu.AddMenuButton Root Next NextMap Root");
  GUI.ParseLine("MapViewerMenu.AddMenuButton Root Previous PrevMap Root");
  GUI.ParseLine("MapViewerMenu.AddMenuButton Root Current CurrentMap Root");

  GUI.ParseLine("EnableMouseControl=0");
  GUI.ParseLine("GLWindow.AddMenu HelicopterMenu Helicopter");
  GUI.ParseLine("HelicopterMenu.AddMenuToggle Root \"Mouse control\" EnableMouseControl Root");
  GUI.ParseLine("HelicopterMenu.AddMenuButton Root \"Clear path\" ClearPath Root");
  GUI.ParseLine("HelicopterMenu.AddMenuButton Root \"Fly path\" FlyPath Root");
  GUI.ParseLine("HelicopterMenu.AddMenuButton Root \"Add waypoint\" AddWaypoint Root");
  GUI.ParseLine("HelicopterMenu.AddMenuButton Root \"Position Hold\" PositionHold Root");
}

/**
 * Run the main system thread.
 * This handles the tracker and the map viewer.
 */
void System::Run()
{
  using namespace std::chrono;

  bool bWriteCoordinatesLog = GV3::get<int>("Debug.OutputWorldCoordinates", 0, SILENT);

  FPSCounter fpsCounter;
  auto startTime = high_resolution_clock::now();
  bool bSaveFrame = false;

  while(!mbDone)
  {
    //Check if the map has been locked by another thread, and wait for release.
    bool bWasLocked = mpMap->mapLockManager.CheckLockAndWait( this, 0 );

    gFrameTimer.Start();

    /* This is a rather hacky way of getting this feedback,
       but GVars cannot be assigned to different variables
       and each map has its own edit lock bool.
       A button could be used instead, but the visual
       feedback would not be as obvious.
    */
    mpMap->bEditLocked = *mgvnLockMap; //sync up the maps edit lock with the gvar bool.

    // Grab new video frame...
    if (!mbFreezeVideo) {
      // We use two versions of each video frame:
      // One black and white (for processing by the tracker etc)
      // and one RGB, for drawing.

      gVideoSourceTimer.Start();
      mVideoSource->GetAndFillFrameBWandRGB(mimFrameBW, mimFrameRGB);
      gVideoSourceTimer.Stop();

      bSaveFrame = true;
    } else if (bSaveFrame) {
      img_save<CVD::byte>(mimFrameBW, "freeze.png");
      bSaveFrame = false;
    }

    if (bWasLocked) {
      mpTracker->ForceRecovery();
    }

    gTrackFullTimer.Start();
    mpTracker->TrackFrame(mimFrameBW, false);
    gTrackFullTimer.Stop();

    if (!*mgvnDisableRendering) {
      static gvar3<int> gvnDrawMap("DrawMap", 0, HIDDEN|SILENT);
      bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;

      // Additional rendering goes here
      gDrawUITimer.Start();
      Draw(bDrawMap);
      gDrawUITimer.Stop();
    }

    mpMikroKopter->Update(mpTracker->GetCurrentPose(), !mpTracker->IsLost());

    if (bWriteCoordinatesLog) {
      duration<double> elapsedTime = high_resolution_clock::now() - startTime;
      mCoordinateLogFile << elapsedTime.count() << " " << mpTracker->RealWorldCoordinate() << endl;
    }

    mGLWindow.HandlePendingEvents();

    // Update FPS counter, this should be the last thing in the main loop
    if (fpsCounter.Update()) {
      stringstream ss; ss << "PTAML - " << setiosflags(ios::fixed) << setprecision(2) << fpsCounter.Fps() << " fps";
      mGLWindow.set_title(ss.str());
    }

    gFrameTimer.Stop();

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void System::Draw(bool bDrawMap)
{
  mGLWindow.SetupViewport();
  mGLWindow.SetupVideoOrtho();
  mGLWindow.SetupVideoRasterPosAndZoom();

  if(bDrawMap) {
    mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
  } else {
    mpTracker->Draw();
  }

  if(*mgvnDrawMapInfo) {
    DrawMapInfo();
  }

  static gvar3<int> gvnDrawMkDebugOutput("DrawMKDebug", 0, HIDDEN|SILENT);
  if (*gvnDrawMkDebugOutput) {
    stringstream ss;
    /*
    ss << "X: " << mPositionHold.GetTargetOffsetFiltered()[0] << "\n"
       << "Y: " << mPositionHold.GetTargetOffsetFiltered()[1] << "\n"
       << "VX: " << mPositionHold.GetVelocityFiltered()[0] << "\n"
       << "VY: " << mPositionHold.GetVelocityFiltered()[1];
       */

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


    for (int i = 0; i < LEVELS; ++i) {
      //ss << g_nNumFeaturesFound[i] << endl;
    }

    mGLWindow.DrawDebugOutput(ss.str());
  }

  string sCaption;
  if(bDrawMap) {
    sCaption = mpMapViewer->GetMessageForUser();
  }
  else {
    sCaption = mpTracker->GetMessageForUser();
  }

  mGLWindow.DrawCaption(sCaption);
  mGLWindow.DrawMenus();

#ifdef _LINUX
  if( *mgvnSaveFIFO )
  {
    SaveFIFO();
  }
#endif

  gGLSwapTimer.Start();
  mGLWindow.swap_buffers();
  gGLSwapTimer.Stop();
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

  if( sCommand=="quit" || sCommand == "exit" ) {
    pSystem->Quit();
  }
  else if( sCommand == "SwitchMap" ) {
    int nMapNum = -1;
    if( GetSingleParam(nMapNum, sCommand, sParams) ) {
      pSystem->SwitchMap( nMapNum );
    }
  }
  else if(sCommand == "Realign") {
    pSystem->mpMapMaker->RequestRealignment();
  }
  else if(sCommand == "ResetAll") {
    pSystem->ResetAll();
  }
  else if( sCommand == "NewMap") {
    pSystem->NewMap();
  }
  else if( sCommand == "DeleteMap") {
    int nMapNum = -1;
    if( sParams.empty() ) {
      pSystem->DeleteMap( pSystem->mpMap->MapID() );
    } else if ( GetSingleParam(nMapNum, sCommand, sParams) ) {
      pSystem->DeleteMap( nMapNum );
    }
  }
  else if( sCommand == "NextMap")  {
    pSystem->mpMapViewer->ViewNextMap();
  }
  else if( sCommand == "PrevMap")  {
    pSystem->mpMapViewer->ViewPrevMap();
  }
  else if( sCommand == "CurrentMap")  {
    pSystem->mpMapViewer->ViewCurrentMap();
  }
  else if( sCommand == "SaveMap" || sCommand == "SaveMaps" || sCommand == "LoadMap")  {
    pSystem->StartMapSerialization( sCommand, sParams );
  }
  else if( sCommand == "PositionHold")  {
    pSystem->PositionHold();
  }
  else if( sCommand == "AddWaypoint")  {
    pSystem->AddWaypoint();
  }
  else if( sCommand == "ClearWaypoints")  {
    pSystem->ClearWaypoints();
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

    pSystem->HandleClick(nButton, irWin);
  }
  else if( sCommand == "KeyPress" )
  {
    if(sParams == "g") {
      pSystem->ToggleDisableRendering();
    }
    else if(sParams == "q" || sParams == "Escape") {
      GUI.ParseLine("quit");
    }
    else if(sParams == "r") {
      pSystem->mpTracker->Reset();
    }
    else if(sParams == "Space") {
      // TODO: Remove the HandleKeyPress function
      pSystem->mpTracker->HandleKeyPress( sParams );
    }
    else if(sParams == "f") {
      pSystem->mbFreezeVideo = !pSystem->mbFreezeVideo;
    }
  }
}

void System::HandleClick(int nButton, const CVD::ImageRef &irWin)
{
  static gvar3<int> gvnEnableMouseControl("EnableMouseControl", 0, HIDDEN|SILENT);
  if (*gvnEnableMouseControl) {

    Vector<3> v3PointOnPlane;
    if (mpTracker->PickPointOnGround(makeVector(irWin.x, irWin.y), v3PointOnPlane)) {
      // Set the targets Z value same as the current positions
      v3PointOnPlane[2] = mpTracker->GetCurrentPose().inverse().get_translation()[2];

      SE3<> se3TargetPose;
      se3TargetPose.get_translation() = v3PointOnPlane;
      mpMikroKopter->GoToPosition(se3TargetPose);
    }
  }
}


/**
 * Switch to the map with ID nMapNum
 * @param  nMapNum Map ID
 * @param bForce This is only used by DeleteMap and ResetAll, and is
 * to ensure that MapViewer is looking at a safe map.
 */
bool System::SwitchMap( int nMapNum, bool bForce )
{

  //same map, do nothing. This should not actually occur
  if(mpMap->MapID() == nMapNum) {
    return true;
  }

  if( (nMapNum < 0) )
  {
    cerr << "Invalid map number: " << nMapNum << ". Not changing." << endl;
    return false;
  }


  for( size_t ii = 0; ii < mvpMaps.size(); ii++ )
  {
    Map * pcMap = mvpMaps[ ii ];
    if( pcMap->MapID() == nMapNum ) {
      mpMap->mapLockManager.UnRegister( this );
      mpMap = pcMap;
      mpMap->mapLockManager.Register( this );
    }
  }

  if(mpMap->MapID() != nMapNum)
  {
    cerr << "Failed to switch to " << nMapNum << ". Does not exist." << endl;
    return false;
  }

  /*  Map was found and switched to for system.
      Now update the rest of the system.
      Order is important. Do not want keyframes added or
      points deleted from the wrong map.

      MapMaker is in its own thread.
      System,Tracker, and MapViewer are all in this thread.
  */

  *mgvnLockMap = mpMap->bEditLocked;


  //update the map maker thread
  if( !mpMapMaker->RequestSwitch( mpMap ) ) {
    return false;
  }

  while( !mpMapMaker->SwitchDone() ) {
#ifdef WIN32
    Sleep(1);
#else
    usleep(10);
#endif
  }

  //update the map viewer object
  mpMapViewer->SwitchMap(mpMap, bForce);

  //update the tracker object
//   mpARDriver->Reset();
  mpARDriver->SetCurrentMap( *mpMap );

  if( !mpTracker->SwitchMap( mpMap ) ) {
    return false;
  }

  return true;
}



/**
 * Create a new map and switch all
 * threads and objects to it.
 */
void System::NewMap()
{
  *mgvnLockMap = false;
  mpMap->mapLockManager.UnRegister( this );
  mpMap = new Map();
  mpMap->mapLockManager.Register( this );
  mvpMaps.push_back( mpMap );

  //update the map maker thread
  mpMapMaker->RequestReInit( mpMap );
  while( !mpMapMaker->ReInitDone() ) {
#ifdef WIN32
    Sleep(1);
#else
    usleep(10);
#endif
  }

  //update the map viewer object
  mpMapViewer->SwitchMap(mpMap);

  //update the tracker object
  mpARDriver->SetCurrentMap( *mpMap);
  mpARDriver->Reset();
  mpTracker->SetNewMap( mpMap );

  cout << "New map created (" << mpMap->MapID() << ")" << endl;
}


/**
 * Moves all objects and threads to the first map, and resets it.
 * Then deletes the rest of the maps, placing PTAMM in its
 * original state.
 * This reset ignores the edit lock status on all maps
 */
void System::ResetAll()
{
  //move all maps to first map.
  if( mpMap != mvpMaps.front() )
  {
    if( !SwitchMap( mvpMaps.front()->MapID(), true ) ) {
      cerr << "Reset All: Failed to switch to first map" << endl;
    }
  }
  mpMap->bEditLocked = false;

  //reset map.
  mpTracker->Reset();

  //lock and delete all remaining maps
  while( mvpMaps.size() > 1 )
  {
    DeleteMap( mvpMaps.back()->MapID() );
  }
}


/**
 * Delete a specified map.
 * @param nMapNum map to delete
 */
bool System::DeleteMap( int nMapNum )
{
  if( mvpMaps.size() <= 1 )
  {
    cout << "Cannot delete the only map. Use Reset instead." << endl;
    return false;
  }

  //if the specified map is the current map, move threads to another map
  if( nMapNum == mpMap->MapID() )
  {
    int nNewMap = -1;

    if( mpMap == mvpMaps.front() ) {
      nNewMap = mvpMaps.back()->MapID();
    }
    else {
      nNewMap = mvpMaps.front()->MapID();
    }

    // move the current map users elsewhere
    if( !SwitchMap( nNewMap, true ) ) {
      cerr << "Delete Map: Failed to move threads to another map." << endl;
      return false;
    }
  }

  // find and delete the map
  for( size_t ii = 0; ii < mvpMaps.size(); ii++ )
  {
    Map * pDelMap = mvpMaps[ ii ];
    if( pDelMap->MapID() == nMapNum ) {

      pDelMap->mapLockManager.Register( this );
      pDelMap->mapLockManager.LockMap( this );
      delete pDelMap;
      mvpMaps.erase( mvpMaps.begin() + ii );

      ///@TODO Possible bug. If another thread (eg serialization) was using this
      /// and waiting for unlock, would become stuck or seg fault.
    }
  }

  return true;
}


/**
 * Set up the map serialization thread for saving/loading and the start the thread
 * @param sCommand the function that was called (eg. SaveMap)
 * @param sParams the params string, which may contain a filename and/or a map number
 */
void System::StartMapSerialization(std::string sCommand, std::string sParams)
{
  if( mpMapSerializer->Init( sCommand, sParams, *mpMap) ) {
    mpMapSerializer->start();
  }
}

void System::PositionHold()
{
  mpMikroKopter->GoToPosition(mpTracker->GetCurrentPose().inverse());

}

void System::AddWaypoint()
{
  mpMikroKopter->AddWaypoint(mpTracker->GetCurrentPose().inverse());
}

void System::ClearWaypoints()
{
  mpMikroKopter->ClearWaypoints();
}

void System::FlyPath()
{
  mpMikroKopter->FlyPath();
}


/**
 * Draw a box with information about the maps.
 */
void System::DrawMapInfo()
{
  int nLines = static_cast<int>(mvpMaps.size()) + 2;
  int x = 5, y = 120, w = 160, nBorder = 5;

  mGLWindow.DrawBox( x, y, w, nLines, 0.7f );

  y += 17;

  glColor3f(1,1,1);
  std::ostringstream os;
  os << "Maps " << mvpMaps.size();
  mGLWindow.PrintString( ImageRef(x + nBorder,y + nBorder), os.str() );
  os.str("");

  for( size_t i = 0; i < mvpMaps.size(); i++ )
  {
    Map * pMap = mvpMaps[i];
    if( pMap == mpMap ) {
      glColor3f(1,1,0);
    }
    else if( pMap->bEditLocked ) {
      glColor3f(1,0,0);
    }
    else {
      glColor3f(1,1,1);
    }

    os << "M: " << pMap->MapID() << "  P: " << pMap->GetMapPoints().size() << "  K: " << pMap->GetKeyFrames().size();
    mGLWindow.PrintString( ImageRef( x + nBorder , y + nBorder + (i+1)*17), os.str() );
    os.str("");
  }
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
        ":format=rgb24 -o $file -ovc lavc -lavcopts vcodec=mpeg4:vbitrate=" << *mgvnBitrate <<
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

