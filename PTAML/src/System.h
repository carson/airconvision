// System.h
//
// Defines the System class
//
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
//
// The instantiation of the system class creates the GUI
#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "VideoSource.h"
#include "GLWindow2.h"
#include "ARToolkit.h"
#include "MikroKopter.h"
#include "Frontend.h"
#include "PerformanceMonitor.h"
#include "Swarm.h"

#include <gvars3/instances.h>
#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <TooN/TooN.h>

namespace PTAMM {

class ATANCamera;
class Map;
class MapMaker;
class Tracker;
class MapViewer;
class MapSerializer;
class MikroKopter;
class Frontend;
class InitialTracker;
class ScaleMarkerTracker;
class Relocaliser;
class FrameGrabber;

struct Modules {
  Modules();
  ~Modules();

  FrameGrabber *pFrameGrabber;
  Relocaliser *pRelocaliser;
  MapMaker *pMapMaker;
  Tracker *pTracker;
  InitialTracker *pInitialTracker;
  ScaleMarkerTracker *pScaleMarkerTracker;
  ATANCamera *pCamera;
  MapViewer *pMapViewer;
  Frontend *pFrontend;
  MikroKopter *pMikroKopter;
  SwarmLab *pSwarmLab;

  MapSerializer *pMapSerializer; // Currently not really used..
};

class System
{
  public:
    System();
    ~System();

    void Run();

  private:
    void CreateModules();
    void CreateMenu();

    void Draw();
    void DrawDebugInfo();
    void DrawPerfInfo();                             // draw a little info box about the maps
    void SaveFIFO();                                // save the video out to a FIFO (save to disk)

    static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);  //process a console command
    void GUICommandCallBack(const std::string &sCommand, const std::string &sParams);

    void HandleClick(int nButton, const CVD::ImageRef &irWin);

    void Quit() { mbDone = true; }
    bool SwitchMap(int nMapNum, bool bForce = false);                                    // Switch to a particular map.
    void NewMap();                                  // Create a new map and move all elements to it
    bool DeleteMap(int nMapNum);                  // Delete a specified map
    void ResetAll();                                // Wipes out ALL maps, returning system to initial state
    void StartMapSerialization(std::string sCommand, std::string sParams);   //(de)serialize a map
    void PositionHold();
    void AddWaypoint();
    void ClearWaypoints();
    void FlyPath();
    void ChangeFeatureDetector();

  private:
    GLWindow2 mGLWindow;                            // The OpenGL window
    PerformanceMonitor mPerfMonitor;
    Modules mModules;
    ARToolkitTracker mARTracker;
    Map *mpMap;                                     // The current map

    bool mbDone;                                    // Kill?

    // Rendering
    FrontendDrawData mFrontendDrawData;
    bool mbDisableRendering;

    SE3<> mse3CurrentPose;
};

}

#endif
