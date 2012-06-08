// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited
//
// System.h
//
// Defines the System class
//
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
//
#ifndef __SYSTEM_H
#define __SYSTEM_H

#include "VideoSource.h"
#include "GLWindow2.h"
#include "ARToolkit.h"
#include "MikroKopter.h"
#include "Frontend.h"

#include <gvars3/instances.h>
#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <TooN/TooN.h>

#include <fstream>

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

struct Modules {
  Relocaliser *pRelocaliser;
  MapMaker *pMapMaker;                           // The map maker
  Tracker *pTracker;                             // The tracker
  InitialTracker *pInitialTracker;
  ScaleMarkerTracker *pScaleMarkerTracker;
  ATANCamera *pCamera;                           // The camera model
  MapViewer *pMapViewer;                         // The Map Viewer
  MapSerializer *pMapSerializer;                 // The map serializer for saving and loading maps
  Frontend *pFrontend;
};

class System
{
  public:
    System(VideoSource* videoSource);
    ~System();

    void Run();

  private:
    void CreateModules();
    void CreateMenu();

    void Draw();
    void DrawTracker();
    void DrawMapViewer();
    void DrawDebugInfo();
    void DrawMapInfo();                             // draw a little info box about the maps
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

  private:
    GLWindow2 mGLWindow;                            // The OpenGL window
    VideoSource *mVideoSource;

    Modules mModules;
    MikroKopter mMikroKopter;
    ARToolkitTracker mARTracker;

    std::vector<Map*> mvpMaps;                      // The set of maps
    Map *mpMap;                                     // The current map

    bool mbDone;                                    // Kill?

    FrontendDrawData mFrontendDrawData;
    bool mbDisableRendering;

    std::ofstream mCoordinateLogFile;                  // Debug output file handle
};

}

#endif
