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

#include <gvars3/instances.h>
#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/byte.h>
#include <TooN/TooN.h>

#include <iostream>
#include <fstream>

namespace PTAMM {

class ATANCamera;
class Map;
class MapMaker;
class Tracker;
class ARDriver;
class MapViewer;
class MapSerializer;
class MikroKopter;

class System
{
  public:
    System(VideoSource* videoSource);
    ~System();
    void Run();

    void ToggleDisableRendering() {
      *mgvnDisableRendering = !*mgvnDisableRendering;
    }

  private:
    static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);  //process a console command

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

    void CreateMenu();

    void DrawMapInfo();                             // draw a little info box about the maps
    void SaveFIFO();                                // save the video out to a FIFO (save to disk)
    void Draw(bool bDrawMap);

  private:
    GLWindow2 mGLWindow;                            // The OpenGL window
    VideoSource* mVideoSource;                       // The video image source
    bool mbFreezeVideo;

    CVD::Image<CVD::Rgb<CVD::byte>> mimFrameRGB;   // The RGB image used for AR
    CVD::Image<CVD::byte> mimFrameBW;               // The Black and white image for tracking/mapping

    std::vector<Map*> mvpMaps;                      // The set of maps
    Map *mpMap;                                     // The current map
    MapMaker *mpMapMaker;                           // The map maker
    Tracker *mpTracker;                             // The tracker
    ATANCamera *mpCamera;                           // The camera model
    ARDriver *mpARDriver;                           // The AR Driver
    MapViewer *mpMapViewer;                         // The Map Viewer
    MapSerializer *mpMapSerializer;                 // The map serializer for saving and loading maps
    MikroKopter *mpMikroKopter;
    ARToolkitTracker mARTracker;

    bool mbDone;                                    // Kill?

    GVars3::gvar3<int> mgvnLockMap;                 // Stop a map being edited - i.e. keyframes added, points updated
    GVars3::gvar3<int> mgvnDrawMapInfo;             // Draw map info on the screen
    GVars3::gvar3<int> mgvnDisableRendering;        // Disable all rendering

    std::ofstream mCoordinateLogFile;                // Debug output file handle

#ifdef _LINUX
    GVars3::gvar3<int> mgvnSaveFIFO;                // Output to a FIFO (make a video)
    GVars3::gvar3<int> mgvnBitrate;                 // Bitrate to encode at
#endif
};

}

#endif
