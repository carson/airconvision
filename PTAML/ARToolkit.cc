#include "ARToolkit.h"
#include <cmath>
#include <fstream>

#include <cvd/argb.h>
#include <cvd/image_convert.h>
#include <cvd/image_io.h>

#include <TooN/TooN.h>
#include <TooN/se3.h>

#include <AR/param.h>
#include <AR/ar.h>
#include <AR/gsub.h>

#include <GL/gl.h>


namespace PTAMM {

const char CPARAM_NAME[] = "Data/psEye.dat";
const char PATTERN_NAME[] = "Data/patt.hiro";
const int MARKER_WIDTH = 80.0;
double MARKER_CENTER[2] = {0.0, 0.0}; // cant be const because of the AR toolkit api...
const int THRESH = 100;

int g_patternId = -1;

bool InitARToolkit(int xsize, int ysize)
{
  ARParam wparam, cparam;

  // set the initial camera parameters
  if (arParamLoad(CPARAM_NAME, 1, &wparam) < 0) {
    printf("Camera parameter load error !!\n");
    return false;
  }

  arParamChangeSize(&wparam, xsize, ysize, &cparam);
  arInitCparam(&cparam);
  printf("*** Camera Parameter ***\n");
  arParamDisp(&cparam);

  if ((g_patternId = arLoadPatt(PATTERN_NAME)) < 0) {
    printf("Pattern file load error !! \n");
    return false;
  }

  return true;
}

void DrawLineSeg( double x1, double y1, double x2, double y2 )
{
  glBegin(GL_LINES);
    glVertex2f( x1, y1 );
    glVertex2f( x2, y2 );
  glEnd();
  glFlush();
}

void DrawSquare(double  vertex[4][2])
{
  DrawLineSeg( vertex[0][0], vertex[0][1],
               vertex[1][0], vertex[1][1]);
  DrawLineSeg( vertex[1][0], vertex[1][1],
               vertex[2][0], vertex[2][1]);
  DrawLineSeg( vertex[2][0], vertex[2][1],
               vertex[3][0], vertex[3][1]);
  DrawLineSeg( vertex[3][0], vertex[3][1],
               vertex[0][0], vertex[0][1]);
}

bool DistanceToMarkerPlane(const CVD::Image<CVD::byte> &imFrame, float& dist)
{
  // Check if AR toolkit was initialized
  if (g_patternId < 0) {
    const CVD::ImageRef& imSize = imFrame.size();
    if (!InitARToolkit(imSize.x, imSize.y)) {
      return false;
    }
  }

  // Convert the input image from BW to RGB (a bit stupid because AR toolkit
  // later converts it to BW anyway)
  CVD::Image<CVD::Rgb<CVD::byte> > rgbImage(imFrame.size());
  CVD::convert_image(imFrame, rgbImage);
  ARUint8* dataPtr = (ARUint8*)rgbImage.data();

  ARMarkerInfo* markerInfo;
  int markerNum;

  // Detect markers in the image
  if (arDetectMarker(dataPtr, THRESH, &markerInfo, &markerNum) < 0) {
    return false;
  }

  // Select the best matched marker
  int k = -1;
  for (int i = 0; i < markerNum; i++) {
    if (markerInfo[i].id  == g_patternId) {
      if( k == -1 ) {
        k = i;
      } else { // make sure you have the best pattern (highest confidence factor)
        if (markerInfo[k].cf < markerInfo[i].cf) k = i;
      }
    }
  }

  if (k == -1) {
    return false;
  }

  // Draw the found squares
  glColor3f(1.0, 0.0, 0.0);
  glLineWidth(6.0);
  for (int i = 0; i < markerNum; ++i) {
    DrawSquare(markerInfo[i].vertex);
  }

  // get the transformation between the marker and the real camera
  double markerTrans[3][4];
  arGetTransMat(&markerInfo[k], MARKER_CENTER, MARKER_WIDTH, markerTrans);

  // Convert the matrix markerTrans into a SE3 object
  TooN::Matrix<3,4,double,TooN::Reference::RowMajor> markerTransMat(*markerTrans);
  TooN::SO3<> rot(markerTransMat.slice<0, 0, 3, 3>());
  TooN::SE3<> mt(rot, markerTransMat.slice<0, 3, 3, 1>().T()[0]);

  // Caluclate the camers position relative to the marker, assuming the
  // marker is located at the world origin.
  TooN::Vector<> camPos = mt.inverse().get_translation();

  // find the range (eqals the Z coordinate of the camera)
  dist = camPos[2];

  //printf(" X: %3.2f Y: %3.2f Z: %3.2f Range: %3.2f \n",Xpos,Ypos,Zpos,range);

  return true;
}

}
