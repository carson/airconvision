#include "FrontendRenderer.h"
#include "OpenGL.h"
#include "Utils.h"

#include <gvars3/instances.h>

#include <cvd/image.h>
#include <cvd/image_ref.h>

using namespace TooN;
using namespace GVars3;
using namespace CVD;

namespace PTAMM {

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

  /*
  glBegin(GL_POINTS);
  for (auto it = vDeadTrails.begin(); it != vDeadTrails.end(); ++it) {
    glColor3f(0.5,0.1,0.7);
    glVertex(*it);
  }
  glEnd();
  */
}

void FrontendRenderer::DrawCorners(const std::vector<CVD::ImageRef> &vCorners)
{
  glColor3f(1,0,1);
  glPointSize(4);
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
  Image<Vector<2> > imVertices(ImageRef(nTot,nTot));
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
  Vector<3> avLevelColors[LEVELS];

  for (int i = 0; i < LEVELS; ++i) {
    if (i==0)      avLevelColors[i] = makeVector( 1.0, 0.0, 0.0);
    else if (i==1) avLevelColors[i] = makeVector( 1.0, 1.0, 0.0);
    else if (i==2) avLevelColors[i] = makeVector( 0.0, 1.0, 0.0);
    else if (i==3) avLevelColors[i] = makeVector( 0.0, 0.0, 0.7);
    else           avLevelColors[i] = makeVector( 1.0, 1.0, 0.7); // In case I ever run with LEVELS > 4
  }

  glPointSize(4);
  glDisable(GL_BLEND);
  glBegin(GL_POINTS);
  for(auto it = vMapPoints.begin(); it != vMapPoints.end(); ++it) {
    glColor(avLevelColors[it->first]);
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
  Vector<3> wx2 = se3WorldFromNormWorld * makeVector(.1, 0, 0);
  Vector<3> wy2 = se3WorldFromNormWorld * makeVector(0, .1, 0);
  Vector<3> wz2 = se3WorldFromNormWorld * makeVector(0, 0, .1);

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

  glRasterPos2d(-0.5,-0.5);
  glDrawPixels(mDrawData.imFrame);


  if (mDrawData.bInitialTracking) {
    if (GV3::get<int>("Tracker.DrawFASTCorners",0, SILENT)) {
      DrawCorners(mDrawData.initialTracker.vCorners);
    }

    if (mDrawData.bUseStereo) {
      Vector<3> v3PointOnPlane;
      if (PickPointOnPlane(mCamera, mDrawData.v4GroundPlane,
                               makeVector(320, 240), v3PointOnPlane))
      {
        v3PointOnPlane *= 0.02; // Scale the point to make it possible to use the same DrawGrid funciton

        Vector<3> v3Normal = mDrawData.v4GroundPlane.slice<0, 3>();
        normalize(v3Normal);
        SE3<> se3AlignedPlane = AlignerFromPointAndUp(v3PointOnPlane, v3Normal);
        SE3<> se3CamFromPlane = se3AlignedPlane.inverse();

        glColor3f(1,1,1);
        //DrawGrid(se3CamFromPlane);
      }

      if (PickPointOnPlane(mCamera, mDrawData.v4DispGroundPlane,
                               makeVector(320, 240), v3PointOnPlane))
      {
        v3PointOnPlane *= 0.02; // Scale the point to make it possible to use the same DrawGrid funciton

        Vector<3> v3Normal = mDrawData.v4DispGroundPlane.slice<0, 3>();
        normalize(v3Normal);
        SE3<> se3AlignedPlane = AlignerFromPointAndUp(v3PointOnPlane, v3Normal);
        SE3<> se3CamFromPlane = se3AlignedPlane.inverse();

        glColor3f(0,1,1);
        DrawGrid(se3CamFromPlane);
      }


    } else {
      DrawTrails(mDrawData.initialTracker.vTrails, mDrawData.initialTracker.vDeadTrails);
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
      glColor4f(1.0f, 0.0f, 0.5f, 0.6f);
    }

    DrawGrid(mse3CamFromWorld);

    // Draw all the matched map points
    DrawMapPoints(mDrawData.tracker.vMapPoints);

    if (!mDrawData.bHasDeterminedScale) {
      DrawMarkerPose(mDrawData.se3MarkerPose);
    }
  }

  // Draw the right stereo frame if it exists
  if (mDrawData.imFrameStereo.data()) {
    int nLeftOffset = mDrawData.imFrame.size().x;
    glRasterPos2d(-0.5 + nLeftOffset,-0.5);
    glDrawPixels(mDrawData.imFrameStereo);

    if (mDrawData.bInitialTracking) {
      DrawCorners(mDrawData.vBackProjectedPts);
    }
  }
}


}
