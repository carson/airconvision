// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
#ifndef __GL_WINDOW_2_H
#define __GL_WINDOW_2_H
//
//  A class which wraps a CVD::GLWindow and provides some basic
//  user-interface funtionality: A gvars-driven clickable menu, and a
//  caption line for text display. Also provides some handy GL helpers
//  and a wrapper for CVD's text display routines.

#include "MKConnection.h"

#include <cvd/glwindow.h>
#include <TooN/TooN.h>

namespace PTAMM {


class GLWindowMenu;


class GLWindow2 : public CVD::GLWindow, public CVD::GLWindow::EventHandler
{
public:
  GLWindow2(CVD::ImageRef irSize, std::string sTitle);
  
  // The preferred event handler..
  void HandlePendingEvents();
  
  // Menu interface:
  void AddMenu(const std::string &sName, const std::string &sTitle);
  void DrawMenus();
  
  // Some OpenGL helpers:
  void SetupViewport();
  void SetupVideoOrtho();
  void SetupUnitOrtho();
  const void SetupWindowOrtho() const;
  void SetupVideoRasterPosAndZoom();

  // Text display functions:
  const void PrintString(const CVD::ImageRef& irPos, const std::string& s) const;
  void DrawCaption(const std::string &s);
  void DrawDebugOutput(const std::string &s);
  const void DrawBox(int x, int y, int w, int nLines, float fAlpha = 0.5) const;
  
  // Map viewer mouse interface:
  std::pair<TooN::Vector<6>, TooN::Vector<6> > GetMousePoseUpdate();

  TooN::Vector<2> VidFromWinCoords( CVD::ImageRef irWin );

  const bool IsMouseButtonPressed( const int button ) const;

protected:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  
  // User interface menus:
  std::vector<GLWindowMenu*> mvpGLWindowMenus;

  CVD::ImageRef mirVideoSize;   // The size of the source video material.
  

  // Event handling routines:
  virtual void on_key_down(GLWindow&, int key);
  virtual void on_mouse_move(GLWindow& win, CVD::ImageRef where, int state);
  virtual void on_mouse_down(GLWindow& win, CVD::ImageRef where, int state, int button);
  virtual void on_mouse_up(GLWindow& win, CVD::ImageRef where, int state, int button);
  virtual void on_event(GLWindow& win, int event);
  CVD::ImageRef mirLastMousePos;

  // Storage for map viewer updates:
  TooN::Vector<6> mvMCPoseUpdate;
  TooN::Vector<6> mvLeftPoseUpdate;
  
  std::map< int, bool > mmMouseButtonPressed;   // record of which mouse button is currently pressed

};


}

#endif
