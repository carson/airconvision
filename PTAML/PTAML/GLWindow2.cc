// Copyright 2009 Isis Innovation Limited
#include "GLWindow2.h"
#include "OpenGL.h"
#include "GLWindowMenu.h"

#include <gvars3/GStringUtil.h>
#include <gvars3/instances.h>
#include <TooN/helpers.h>

#include <GL/gl.h>

#include <cstdlib>

namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;
using namespace TooN;

size_t CountLines(const std::string &s)
{
  size_t nLines = 0;
  std::string sendl("\n");
  std::string::size_type st = 0;

  while(1) {
    nLines++;
    st = s.find(sendl, st);
    if(st == std::string::npos)
      break;
    else
      st++;
  }

  return nLines;
}

GLWindow2::GLWindow2(ImageRef irSize, string sTitle)
  : GLWindow(irSize, sTitle)
{

#ifdef WIN32
  // On windows, have to initialise GLEW at the start to enable access
  // to GL extensions
  static bool bGLEWIsInit = false;
  if(!bGLEWIsInit)
  {
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		fprintf(stderr, "GLEW Error: %s\n", glewGetErrorString(err));
		exit(0);
	}
	bGLEWIsInit = true;
  }
#endif

  mirVideoSize = irSize;
  GUI.RegisterCommand("GLWindow.AddMenu", GUICommandCallBack, this);
  glSetFont("sans");
  mvMCPoseUpdate=Zeros;
  mvLeftPoseUpdate=Zeros;
}


void GLWindow2::AddMenu(const string &sName, const string &sTitle)
{
  GLWindowMenu* pMenu = new GLWindowMenu(sName, sTitle); 
  mvpGLWindowMenus.push_back(pMenu);
}

void GLWindow2::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  ((GLWindow2*) ptr)->GUICommandHandler(sCommand, sParams);
}

void GLWindow2::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
  vector<string> vs=ChopAndUnquoteString(sParams);
  if(sCommand=="GLWindow.AddMenu") {
    switch(vs.size())
    {
    case 1:
      AddMenu(vs[0], "Root");
      return;
    case 2:
      AddMenu(vs[0], vs[1]);
      return;
    default:
      cout << "? AddMenu: need one or two params (internal menu name, [caption])." << endl;
      return;
    }
  }
  
  // Should have returned to caller by now - if got here, a command which 
  // was not handled was registered....
  cout << "! GLWindow::GUICommandHandler: unhandled command "<< sCommand << endl;
  exit(1);
}; 

void GLWindow2::DrawMenus()
{
  SetupWindowOrtho();

  glDisable(GL_STENCIL_TEST);
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_TEXTURE_RECTANGLE_ARB);
  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_POLYGON_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColorMask(1,1,1,1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  SetupWindowOrtho();
  glLineWidth(1);
  
  int nTop = 30;
  int nHeight = 30;
  for(vector<GLWindowMenu*>::iterator i = mvpGLWindowMenus.begin();
      i!= mvpGLWindowMenus.end();
      i++)
    {
      (*i)->Render(nTop, nHeight, size()[0], *this);
      nTop+=nHeight+1;
    }
}

void GLWindow2::SetupUnitOrtho()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0,1,1,0,0,1);
}

const void GLWindow2::SetupWindowOrtho() const
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(size());
}

void GLWindow2::SetupVideoOrtho()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-0.5,(double)mirVideoSize.x - 0.5, (double) mirVideoSize.y - 0.5, -0.5, -1.0, 1.0);
}

void GLWindow2::SetupVideoRasterPosAndZoom()
{ 
  glRasterPos2d(-0.5,-0.5);
  double adZoom[2];
  adZoom[0] = (double) size()[0] / (double) mirVideoSize[0];
  adZoom[1] = (double) size()[1] / (double) mirVideoSize[1];
  glPixelZoom(static_cast<float>(adZoom[0]), static_cast<float>(-adZoom[1]));
}

void GLWindow2::SetupViewport()
{
  glViewport(0, 0, size()[0], size()[1]);
}

const void GLWindow2::PrintString(const CVD::ImageRef &irPos, const std::string &str) const
{
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glTranslatef(static_cast<float>(irPos.x), static_cast<float>(irPos.y), 0.0f);
  glScalef(8,-8,1);
  glDrawText(str, NICE, 1.6, 0.1);
  glPopMatrix();
}

void GLWindow2::DrawCaption(const string &s)
{
  if(s.empty())
    return;
  
  SetupWindowOrtho();

  // Find out how many lines are in the caption:
  size_t nLines = CountLines(s);
  
  int nTopOfBox = size().y - nLines * 17;
  
  // Draw a grey background box for the text
  glColor4f(0.0f,0.0f,0.0f,0.4f);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBegin(GL_QUADS);
  glVertex2d(-0.5, nTopOfBox);
  glVertex2d(size().x, nTopOfBox);
  glVertex2d(size().x, size().y);
  glVertex2d(-0.5, size().y);
  glEnd();
  
  // Draw the caption text in yellow
  glColor3f(1,1,0);      
  PrintString(ImageRef(10,nTopOfBox + 13), s);

  glDisable(GL_BLEND);
}

void GLWindow2::DrawDebugOutput(const string &s)
{
  SetupWindowOrtho();

  // Find out how many lines are in the caption:
  size_t nLines = CountLines(s);

  int nLeftOfBox = 5;
  int nTopOfBox = 5;
  int nBottomOfBox = nTopOfBox + nLines * 13;
  int nRightOfBox = 120;

  // Draw a grey background box for the text
  glColor4f(0.0f,0.0f,0.0f,0.4f);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBegin(GL_QUADS);
  glVertex2d(nLeftOfBox, nTopOfBox);
  glVertex2d(nRightOfBox, nTopOfBox);
  glVertex2d(nRightOfBox, nBottomOfBox);
  glVertex2d(nLeftOfBox, nBottomOfBox);
  glEnd();

  // Draw the caption text in yellow
  glColor3f(1,1,0);
  PrintString(ImageRef(10, nTopOfBox + 13), s);
}



/**
 * Draw a grey, semi transparent box
 * @param x Top left corner x
 * @param y Top left corner y
 * @param w Width of box
 * @param nLines Number of lines of text
 */
const void GLWindow2::DrawBox(int x, int y, int w, int nLines, float fAlpha) const
{
  SetupWindowOrtho();
  int h = nLines * 17;
  
  // Draw a grey background box for the text
  glColor4f(0,0,0,fAlpha);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBegin(GL_QUADS);
  glVertex2d( x, y );
  glVertex2d( x + w, y );
  glVertex2d( x + w, y + h );
  glVertex2d( x, y + h );
  glEnd();

  
}


void GLWindow2::HandlePendingEvents()
{
  handle_events(*this);
}

void GLWindow2::on_mouse_move(GLWindow& win, CVD::ImageRef where, int state)
{
  ImageRef irMotion = where - mirLastMousePos;
  mirLastMousePos = where;
  
  double dSensitivity = 0.01;
  if( (state & BUTTON_LEFT) && ! (state & BUTTON_RIGHT) )
    {
      mvMCPoseUpdate[3] -= irMotion[1] * dSensitivity;
      mvMCPoseUpdate[4] += irMotion[0] * dSensitivity;
    }
  else if( !(state & BUTTON_LEFT) && (state & BUTTON_RIGHT) )
    {
      mvLeftPoseUpdate[4] -= irMotion[0] * dSensitivity;
      mvLeftPoseUpdate[3] += irMotion[1] * dSensitivity;
    }
  else if( (state & BUTTON_MIDDLE) || ((state & BUTTON_LEFT) && (state & BUTTON_RIGHT)) )
    {
      mvLeftPoseUpdate[5] -= irMotion[0] * dSensitivity;
      mvLeftPoseUpdate[2] += irMotion[1] * dSensitivity;
    }
  
}

/**
 * What to do when the user presses a mouse button
 * @param win the window
 * @param where the location
 * @param state the button state
 * @param button the button pressed
 */
void GLWindow2::on_mouse_down(GLWindow& win, CVD::ImageRef where, int state, int button)
{
  bool bHandled = false;
  for(unsigned int i=0; !bHandled && i<mvpGLWindowMenus.size(); i++) {
    bHandled |= mvpGLWindowMenus[i]->HandleClick(button, state, where.x, where.y);
  }

  if( !bHandled ) {
    ostringstream os;
    os << "Mouse.Click " << button << " " << where.x << " " << where.y << endl;
    GUI.ParseLine( os.str() );
  }

  mmMouseButtonPressed[ button ] = true;

}

/**
 * What to do when the user releases a mouse button
 * @param win the window
 * @param where the location
 * @param state the button state
 * @param button the button pressed
 */
void GLWindow2::on_mouse_up(GLWindow& win, CVD::ImageRef where, int state, int button)
{
  mmMouseButtonPressed[ button ] = false;
}

void GLWindow2::on_event(GLWindow& win, int event)
{
  if(event==EVENT_CLOSE)
    GUI.ParseLine("quit");
}

pair<Vector<6>, Vector<6> > GLWindow2::GetMousePoseUpdate()
{
  pair<Vector<6>, Vector<6> > result = make_pair(mvLeftPoseUpdate, mvMCPoseUpdate);
  mvLeftPoseUpdate = Zeros;
  mvMCPoseUpdate = Zeros;
  return result;
}



/**
 * Process the key press, and notify System using the command system
 * @param  glWindow The GL window.
 * @param k the key code.
 */
#ifndef WIN32
#include <X11/keysym.h>
void GLWindow2::on_key_down(GLWindow&, int k)
{
  string s;
  switch(k)
    {
    case XK_a:   case XK_A:  s="a"; break;
    case XK_b:   case XK_B:  s="b"; break;
    case XK_c:   case XK_C:  s="c"; break;
    case XK_d:   case XK_D:  s="d"; break;
    case XK_e:   case XK_E:  s="e"; break;
    case XK_f:   case XK_F:  s="f"; break;
    case XK_g:   case XK_G:  s="g"; break;
    case XK_h:   case XK_H:  s="h"; break;
    case XK_i:   case XK_I:  s="i"; break;
    case XK_j:   case XK_J:  s="j"; break;
    case XK_k:   case XK_K:  s="k"; break;
    case XK_l:   case XK_L:  s="l"; break;
    case XK_m:   case XK_M:  s="m"; break;
    case XK_n:   case XK_N:  s="n"; break;
    case XK_o:   case XK_O:  s="o"; break;
    case XK_p:   case XK_P:  s="p"; break;
    case XK_q:   case XK_Q:  s="q"; break;
    case XK_r:   case XK_R:  s="r"; break;
    case XK_s:   case XK_S:  s="s"; break;
    case XK_t:   case XK_T:  s="t"; break;
    case XK_u:   case XK_U:  s="u"; break;
    case XK_v:   case XK_V:  s="v"; break;
    case XK_w:   case XK_W:  s="w"; break;
    case XK_x:   case XK_X:  s="x"; break;
    case XK_y:   case XK_Y:  s="y"; break;
    case XK_z:   case XK_Z:  s="z"; break;
    case XK_1:   s="1"; break;
    case XK_2:   s="2"; break;
    case XK_3:   s="3"; break;
    case XK_4:   s="4"; break;
    case XK_5:   s="5"; break;
    case XK_6:   s="6"; break;
    case XK_7:   s="7"; break;
    case XK_8:   s="8"; break;
    case XK_9:   s="9"; break;
    case XK_0:   s="0"; break;
    case XK_KP_Prior: case XK_Page_Up:     s="PageUp"; break;
    case XK_KP_Next:  case XK_Page_Down:   s="PageDown"; break;
    case XK_Return: s="Enter"; break;
    case XK_space:  s="Space"; break;
    case XK_BackSpace:  s="BackSpace"; break;
    case XK_Escape:  s="Escape"; break;
    default: break;
    }

  if(s!="")
    GUI.ParseLine("try KeyPress "+s);
}
#else
void GLWindow2::on_key_down(GLWindow&, int k)
{
  string s;
  // ASCI chars can be mapped directly:
  if( (k >= 48 && k <=57) || ( k >=97 && k <= 122) || (k >= 65 && k <= 90))
  {
	char c = k;
	if(c >= 65 && c <= 90)
		c = c + 32;
	s = c;
  }
  else switch (k) // Some special chars are translated:
  {
    case 33: s="PageUp"; break;
    case 34: s="PageDown"; break;
    case 13: s="Enter"; break;
    case 32:  s="Space"; break;
    case 8:  s="BackSpace"; break;
    case 27:  s="Escape"; break;
    default: break;
  }
  
  if(s!="")
    GUI.ParseLine("try KeyPress "+s);
}
#endif


/**
 * Calculate the video image coordinates from the window coordinates.
 * This is needed when the window has been resized
 * @param v2WinCoords the window coordinates
 * @return the video coordinates
 */
Vector<2> GLWindow2::VidFromWinCoords( CVD::ImageRef irWin )
{
  double dXScale = (double) mirVideoSize[0] / (double) size().x;
  double dYScale = (double) mirVideoSize[1] / (double) size().y;
  Vector<2> v2;
  v2[0] = dXScale * (double) irWin.x;
  v2[1] = dYScale * (double) irWin.y;
  return v2;
}



/**
 * Find out if a mouse button is currently pressed.
 * @param button the button id to query
 * @return true if down, false if not or unknown.
 */
const bool GLWindow2::IsMouseButtonPressed( const int button ) const
{
  std::map <int, bool >::const_iterator it = mmMouseButtonPressed.find( button );

  if( it == mmMouseButtonPressed.end() ) {
    return false;
  }
  
  return (*it).second;
  
}


}

