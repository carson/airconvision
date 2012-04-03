// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited


#include <algorithm>
#include "MapViewer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LevelHelpers.h"
#include <iomanip>
#include "math.h"
#include <cvd/gl_helpers.h>

namespace PTAMM {

using namespace CVD;
using namespace std;


/**
 * Map viewer constructor
 * @param maps the vector of all maps
 * @param map the current map
 * @param glw the GL window reference
 */
MapViewer::MapViewer(std::vector<Map*> &maps, Map *map, GLWindow2 &glw):
  mvpMaps(maps),
  mpMap(map),
  mpViewingMap(map),
  mGLWindow(glw),
  mbBrowseMode(false)
{
  mse3ViewerFromWorld =
    SE3<>::exp(makeVector(0,0,2,0,0,0)) * SE3<>::exp(makeVector(0,0,0,0.8 * M_PI,0,0));
}

/**
 * Draw the map dots
 */
void MapViewer::DrawMapDots()
{
  SetupFrustum();
  SetupModelView();

  int nForMass = 0;
  glColor3f(0,1,1);
  //glPointSize(3);//@hack
  glPointSize(1);
  glBegin(GL_POINTS);
  mv3MassCenter = Zeros;
  for(size_t i=0; i<mpViewingMap->vpPoints.size(); i++)
  {
    
    Vector<3> v3Pos = mpViewingMap->vpPoints[i]->v3WorldPos;
    /*
     *@hack
     */
    glColor(mpViewingMap->vpPoints[i]->pColor);

    if( (v3Pos * v3Pos) < 10000)
    {
      nForMass++;
      mv3MassCenter += v3Pos;
    }
    glVertex(v3Pos);
  }
  glEnd();
  mv3MassCenter = mv3MassCenter / (0.1 + nForMass);
}


/**
 * Draw the Grid
 */
void MapViewer::DrawGrid()
{
  SetupFrustum();
  SetupModelView();
  glLineWidth(1);

  glBegin(GL_LINES);

  // Draw a larger grid around the outside..
  double dGridInterval = 0.1;

  double dMin = -100.0 * dGridInterval;
  double dMax =  100.0 * dGridInterval;

  for(int x=-10;x<=10;x+=1)
    {
      if(x==0)
	glColor3f(1,1,1);
      else
	glColor3f(0.3,0.3,0.3);
      glVertex3d((double)x * 10 * dGridInterval, dMin, 0.0);
      glVertex3d((double)x * 10 * dGridInterval, dMax, 0.0);
    }
  for(int y=-10;y<=10;y+=1)
    {
      if(y==0)
	glColor3f(1,1,1);
      else
	glColor3f(0.3,0.3,0.3);
      glVertex3d(dMin, (double)y * 10 *  dGridInterval, 0.0);
      glVertex3d(dMax, (double)y * 10 * dGridInterval, 0.0);
    }

  glEnd();

  glBegin(GL_LINES);
  dMin = -10.0 * dGridInterval;
  dMax =  10.0 * dGridInterval;

  for(int x=-10;x<=10;x++)
    {
      if(x==0)
	glColor3f(1,1,1);
      else
	glColor3f(0.5,0.5,0.5);

      glVertex3d((double)x * dGridInterval, dMin, 0.0);
      glVertex3d((double)x * dGridInterval, dMax, 0.0);
    }
  for(int y=-10;y<=10;y++)
    {
      if(y==0)
	glColor3f(1,1,1);
      else
	glColor3f(0.5,0.5,0.5);
      glVertex3d(dMin, (double)y * dGridInterval, 0.0);
      glVertex3d(dMax, (double)y * dGridInterval, 0.0);
    }

  glColor3f(1,0,0);
  glVertex3d(0,0,0);
  glVertex3d(1,0,0);
  glColor3f(0,1,0);
  glVertex3d(0,0,0);
  glVertex3d(0,1,0);
  glColor3f(1,1,1);
  glVertex3d(0,0,0);
  glVertex3d(0,0,1);
  glEnd();

//   glColor3f(0.8,0.8,0.8);
//   glRasterPos3f(1.1,0,0);
//   mGLWindow.PrintString("x");
//   glRasterPos3f(0,1.1,0);
//   mGLWindow.PrintString("y");
//   glRasterPos3f(0,0,1.1);
//   mGLWindow.PrintString("z");
}

  /**
   * @hack by camparijet
   * Draw Flat grid 
   * @todo repairquick and dirty hack
   *
   */
void MapViewer::DrawFlatGrid(){
  //SetupFrustum();
  //SetupModelView();
  glLineWidth(1);

  glBegin(GL_LINES);

  // Draw a larger grid around the outside..
  double dGridInterval = 0.1;

  double dMin = -100.0 * dGridInterval;
  double dMax =  100.0 * dGridInterval;

  for(int x=-10;x<=10;x+=1)
    {
      if(x==0)
	glColor3f(1,1,1);
      else
	glColor3f(0.3,0.3,0.3);
      glVertex3d((double)x * 10 * dGridInterval, dMin, 0.0);
      glVertex3d((double)x * 10 * dGridInterval, dMax, 0.0);
    }
  for(int y=-10;y<=10;y+=1)
    {
      if(y==0)
	glColor3f(1,1,1);
      else
	glColor3f(0.3,0.3,0.3);
      glVertex3d(dMin, (double)y * 10 *  dGridInterval, 0.0);
      glVertex3d(dMax, (double)y * 10 * dGridInterval, 0.0);
    }

  glEnd();

  glBegin(GL_LINES);
  dMin = -10.0 * dGridInterval;
  dMax =  10.0 * dGridInterval;

  for(int x=-10;x<=10;x++)
    {
      if(x==0)
	glColor3f(1,1,1);
      else
	glColor3f(0.5,0.5,0.5);

      glVertex3d((double)x * dGridInterval, dMin, 0.0);
      glVertex3d((double)x * dGridInterval, dMax, 0.0);
    }
  for(int y=-10;y<=10;y++)
    {
      if(y==0)
	glColor3f(1,1,1);
      else
	glColor3f(0.5,0.5,0.5);
      glVertex3d(dMin, (double)y * dGridInterval, 0.0);
      glVertex3d(dMax, (double)y * dGridInterval, 0.0);
    }

  glColor3f(1,0,0);
  glVertex3d(0,0,0);
  glVertex3d(1,0,0);
  glColor3f(0,1,0);
  glVertex3d(0,0,0);
  glVertex3d(0,1,0);
  glColor3f(1,1,1);
  glVertex3d(0,0,0);
  glVertex3d(0,0,1);
  glEnd();
}


/**
 * Draw the map
 * @param se3CamFromWorld Current camera location
 */
void MapViewer::DrawMap(SE3<> se3CamFromWorld)
{
  mMessageForUser.str(""); // Wipe the user message clean

  // Update viewer position according to mouse input:
  {
    pair<Vector<6>, Vector<6> > pv6 = mGLWindow.GetMousePoseUpdate();
    SE3<> se3CamFromMC;
    se3CamFromMC.get_translation() = mse3ViewerFromWorld * mv3MassCenter;
    mse3ViewerFromWorld = SE3<>::exp(pv6.first) *
      se3CamFromMC * SE3<>::exp(pv6.second) * se3CamFromMC.inverse() * mse3ViewerFromWorld;
  }

  mGLWindow.SetupViewport();
  glClearColor(0,0,0,0);
  glClearDepth(1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glColorMask(1,1,1,1);

  glEnable(GL_DEPTH_TEST);
  //DrawGrid(); @hack by camparijet
  //DrawFlatGrid();
  //DrawMapDots();

  if( mpViewingMap == mpMap ) {
    DrawCamera(se3CamFromWorld);
  }

  for(size_t i=0; i<mpViewingMap->vpKeyFrames.size(); i++){
    TakDrawCameraAndFrame(mpViewingMap->vpKeyFrames[i]->se3CfromW, true,mpViewingMap->texName,mpViewingMap->vpKeyFrames[i]->tIndex,mpViewingMap->vpKeyFrames[i]->Camera);       
  }
  //cout << "Display Keyframe : " << mpViewingMap->vpKeyFrames[i]->tIndex << endl; 
  //TakDrawCameraAndFrame(mpViewingMap->vpKeyFrames[i]->se3CfromW, true,mpViewingMap->texName,0);
  //TakDrawCameraAndFrame(mpViewingMap->vpKeyFrames[i]->se3CfromW, true,mpViewingMap->vpKeyFrames[i]->aLevels[0].im);
    //DrawCamera(mpViewingMap->vpKeyFrames[i]->se3CfromW, true);
  glDisable(GL_DEPTH_TEST);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  mMessageForUser << " Map " << mpViewingMap->MapID() << ": "
		  << mpViewingMap->vpPoints.size() << "P, " << mpViewingMap->vpKeyFrames.size() << "KF";
  mMessageForUser << setprecision(4);
  mMessageForUser << "   Camera Pos: " << se3CamFromWorld.inverse().get_translation();
}

/**
 * Return the status bar message
 * @return the message string
 */
string MapViewer::GetMessageForUser()
{
  return mMessageForUser.str();
}


/**
 * set up the viewer frustrum
 */
void MapViewer::SetupFrustum()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  double zNear = 0.03;
  glFrustum(-zNear, zNear, 0.75*zNear,-0.75*zNear,zNear,50); 
  glScalef(1,1,-1);
  return;
}
/**
 * @hack by camparijet
 * for KF's textures
 
void MapViewer::TakSetupFrustum()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  double zNear = 0.03;
  glMultMatrix(Camera.MakeUFBLinearFrustumMatrix(-zNear,zNear));
  //glFrustum(-zNear, zNear, 0.75*zNear,-0.75*zNear,zNear,50);
  glScalef(1,1,-1);
  return;
}
*/

/**
 * Set up the model view.
 * @param se3WorldFromCurrent se3 that converts from
 * world frame to camera frame
 */
void MapViewer::SetupModelView(SE3<> se3WorldFromCurrent)
{
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glMultMatrix(mse3ViewerFromWorld * se3WorldFromCurrent);
  return;
}


/**
 * Draw the camera / keyframe
 * @param se3CfromW Camera's location in thr world
 * @param bSmall Draw the small camera (keyframe)
 */
void MapViewer::DrawCamera(SE3<> se3CfromW, bool bSmall)
{

  SetupModelView(se3CfromW.inverse());
  SetupFrustum();

  if(bSmall)
    glLineWidth(1);
  else
    glLineWidth(3);
  /**
   * @hack camparijet
   * for drawing camera-coordination
   */
  glBegin(GL_LINES);
  glColor3f(1,0,0);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.1f, 0.0f, 0.0f);
  glColor3f(0,1,0);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.1f, 0.0f);
  glColor3f(1,1,1);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 0.1f);
  glEnd();

  /**
   * @hack camparijet
   * projection camera positiion
   * in the initialized plane.
   */
  if(!bSmall)
  {
    glLineWidth(1);
    glColor3f(0.5,0.5,0.5);
    SetupModelView();
    Vector<2> v2CamPosXY = se3CfromW.inverse().get_translation().slice<0,2>();
    glBegin(GL_LINES);
    glColor3f(1,1,1);
    glVertex2d(v2CamPosXY[0] - 0.04, v2CamPosXY[1] + 0.04);
    glVertex2d(v2CamPosXY[0] + 0.04, v2CamPosXY[1] - 0.04);
    glVertex2d(v2CamPosXY[0] - 0.04, v2CamPosXY[1] - 0.04);
    glVertex2d(v2CamPosXY[0] + 0.04, v2CamPosXY[1] + 0.04);
    glEnd();
  }
  /*
  glBegin(GL_LINES);
  glLineWidth(3);
  glColor3f(1,0,0);
  glMultMatrix(se3CfromW);
  //glVertex3d(se3C);
  glMultMatrix(se3CfromW.inverse());
  glEnd();*/

}

/**
 * @hack added by camparijet
 * Drawing KeyFrame in MapViewer mode...
 * this function is extended "DrawCamera" (SE3<> se3CfromW, bool bSmassll)
 * @param se3CfromW Camera's location in thr world
 * @param bSmall Draw the small camera (keyframe)
 * @param keyFrameImage KeyFrame's Image...
 */
//  void MapViewer::TakDrawCameraAndFrame(SE3<> se3CfromW, bool bSmall,Image<CVD::Rgb<CVD::byte> >keyFrameImage,GLuint *t)
  void MapViewer::TakDrawCameraAndFrame(SE3<> se3CfromW, bool bSmall,GLuint *texName, int tIndex,ATANCamera camera)
  //void MapViewer::TakDrawCameraAndFrame(KeyFrame &k,bool bSmall,GLuint *texName)
{
  //texture matrix
  glMatrixMode(GL_TEXTURE);
  glLoadIdentity();
  /*glTranslated(-0.5,-0.5,0.0);
    glScaled(0.5,0.5,1.0);*/
      
  SetupModelView(se3CfromW.inverse());
  //@hack for texture  
  // glMatrixMode(GL_TEXTURE);
  // glLoadIdentity();
  // TooN::Matrix<4,4> camparams;
  // //cout << camera.GetParams() << endl;
  // Fill(camparams) = camera.mvFocal[0],0,0,-camera.mvCenter[0]*camera.mvInvFocal[0],       
  //   0,camera.mvFocal[1],0,-camera.mvCenter[3]*camera.mvInvFocal[1],
  //   0,0,1,0,
  //   0,0,0,1;
  // // = TooN::Data(camera.GetParams[0],0,0,-camera.GetParams[2]/camera.GetParams[0],
  //    // 	   0,camera.GetParams[1],0,-camera.GetParams[3]/camera.GetParams[1],
  //    // 	   0,0,1,0,
  //    // 	   0,0,0,1);
  
  // //glTranslated()
  // glMultMatrix(camparams);  
  SetupFrustum();
  
  /**
   * @hack camparijet
   * for drawing keyframe
   */
  //glDrawPixels(keyFrameImage);

  if(bSmall)
    glLineWidth(1);
  else
    glLineWidth(3);

  /**
   * @hack camparijet
   * for drawing camera-coordination
   */
  glBegin(GL_LINES);
  glColor3f(1,0,0);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.1f, 0.0f, 0.0f);
  glColor3f(0,1,0);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.1f, 0.0f);
  glColor3f(1,1,1);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0f, 0.0f, 0.1f);
  glEnd();

  /**
   * @hack by camparijet
   * drawing Camera an
   *
   */
  /*glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glMultMatrix(camera.MakeUFBLinearFrustumMatrix(0,1));
  glScalef(1,1,-1);*/
  //glProje()
  //glBlendFunc(GL_SRC_COLOR, GL_DST_COLOR);  
  // static double fx = 1./camera.GetParams()[0];
  // static double fy = 1./camera.GetParams()[1];
  // static double cx = -camera.GetParams()[2]/camera.GetParams()[0];
  // static double cy = -camera.GetParams()[3]/camera.GetParams()[1];
  //glScalef(1.,1.,1.);
  glBlendFunc(GL_SRC_COLOR,GL_ONE_MINUS_SRC_COLOR);
  glEnable(GL_BLEND);
  
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D,texName[tIndex]);
  
  glBegin(GL_QUADS);
  //0.490112 0.680042 0.521819 0.387479 0.285584
  // glTexCoord2d(0.25,0.75);
  // glVertex3d(-1./0.490112,1./0.680042,1);
  
  // glTexCoord2d(0.25,0.25);
  // glVertex3d(-1./0.490112,-1./0.680042,1);
  
  // glTexCoord2d(0.75, 0.25);
  // glVertex3d(1./0.490112,-1./0.680042,1);
  
  // glTexCoord2d(0.75, 0.75);
  // glVertex3d(1./0.490112,1./0.680042,1);
  
  glTexCoord2d(0.0,1.0);
  glVertex3d(-1,1,1);
  
  glTexCoord2d(0.0,0.0);
  glVertex3d(-1,-1,1);
  
  glTexCoord2d(1.0, 0.0);
  glVertex3d(1,-1,1);
  
  glTexCoord2d(1.0, 1.0);
  glVertex3d(1,1,1);
  
  
  // glTexCoord2d(0.0, 1.0);
  // glVertex3d(-1,0.75,1);
  
  // glTexCoord2d(0.0, 0.0);
  // glVertex3d(-1,-0.75,1);
  
  // glTexCoord2d(1.0, 0.0);
  // glVertex3d(1,-0.75,1);
  
  // glTexCoord2d(1.0, 1.0);
  // glVertex3d(1,0.75,1);
  
  glEnd();
  
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_BLEND);
  

  /**
   * @hack camparijet
   * projection camera positiion
   * in the initialized plane.
   */
  if(!bSmall)
  {
    glLineWidth(1);
    glColor3f(0.5,0.5,0.5);
    SetupModelView();
    Vector<2> v2CamPosXY = se3CfromW.inverse().get_translation().slice<0,2>();
    glBegin(GL_LINES);
    glColor3f(1,1,1);
    glVertex2d(v2CamPosXY[0] - 0.04, v2CamPosXY[1] + 0.04);
    glVertex2d(v2CamPosXY[0] + 0.04, v2CamPosXY[1] - 0.04);
    glVertex2d(v2CamPosXY[0] - 0.04, v2CamPosXY[1] - 0.04);
    glVertex2d(v2CamPosXY[0] + 0.04, v2CamPosXY[1] + 0.04);
    glEnd();
  }
  /* camera path?
  glLineWidth(3);
  glColor3f(1,0,0);
  glBegin(GL_LINES);
  glVertex3f(0,0,0);
  glVertex3f(se3CfromW.get_translation()[0],
	     se3CfromW.get_translation()[1],
	     se3CfromW.get_translation()[2]
	     );
  //glVertex3d(10,0,0);

  //glModelView()
  //glMultMatrix(se3CfromW); 
  //glVertex3d(0,0,0);
  //glMultMatrix(se3CfromW.inverse());
  glEnd();*/
}

/**
 * Switch to the specified map.
 * @param map the map to switch to.
 * @param bForce forces the map view to view the specifed map. This is used when deleting a map.
 */
void MapViewer::SwitchMap( Map * map, bool bForce )
{
  if( map != NULL && map != mpMap ) {
    mpMap = map;

    if(!mbBrowseMode || bForce) {
      mpViewingMap = mpMap;
    }
  }

  /*  If this was in a separate thread then
      a switching mechanism such as the one
      in MapMaker would be required
  */
}


/**
 * Switch to the next map in the list.
 */
void MapViewer::ViewNextMap()
{
  vector<Map*>::iterator it = find(mvpMaps.begin(), mvpMaps.end(), mpViewingMap);
  if(it == mvpMaps.end()) {
    return;
  }

  mbBrowseMode = true;

  it++;
  if(it != mvpMaps.end())  {
    mpViewingMap = (*it);
  }
  else {
    mpViewingMap = mvpMaps.front();
  }
}


/**
 * Switch to the previous map in the list.
 */
void MapViewer::ViewPrevMap()
{
  vector<Map*>::iterator it = find(mvpMaps.begin(), mvpMaps.end(), mpViewingMap);
  if(it == mvpMaps.end()) {
    return;
  }

  mbBrowseMode = true;

  int pos = it - mvpMaps.begin();

  if(pos > 0)
  {
    --it;
    mpViewingMap = (*it);
  }
  else {
    mpViewingMap = mvpMaps.back();
  }
}


/**
 * View the current map, and leave browsing mode.
 */
void MapViewer::ViewCurrentMap()
{
  mpViewingMap = mpMap;
  mbBrowseMode = false;
}

}

