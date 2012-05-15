// Copyright 2008 Isis Innovation Limited
#include "KeyFrame.h"
#include "ShiTomasi.h"
#include "SmallBlurryImage.h"
#include "LevelHelpers.h"
#include "MapPoint.h"

#include <cvd/vision.h>
#include <cvd/fast_corner.h>

namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;

/**
 * Constructer.
 * @param cam the camera that made this keyframe
 */
KeyFrame::KeyFrame(const ATANCamera &cam)
  : pSBI( NULL ),
    Camera(cam)
{

}

/**
 * Destructor
 */
KeyFrame::~KeyFrame()
{
  if( pSBI != NULL )  {
    delete pSBI;
    pSBI = NULL;
  }
}


/**
 * Keyframe copy constructor
 * @param rhs
 */
KeyFrame::KeyFrame(const KeyFrame &rhs)
  : Camera( rhs.Camera )
{
  *this = rhs;
}


/**
 * Keyframe assingment operater.
 * Copies one keyframe from another
 * @param rhs Keyframe to copy from
 * @return the resulting copy
 */
KeyFrame& KeyFrame::operator=(const KeyFrame &rhs)
{
  if(this == &rhs) {
    return *this;
  }

  se3CfromW          =    rhs.se3CfromW;
  bFixed             =    rhs.bFixed;
  mMeasurements      =    rhs.mMeasurements;
  dSceneDepthMean    =    rhs.dSceneDepthMean;
  dSceneDepthSigma   =    rhs.dSceneDepthSigma;
  Camera             =    rhs.Camera;

  for( int i = 0; i < LEVELS; ++i ) {
    aLevels[i] = rhs.aLevels[i];
  }

  //should actually copy this, but is cheap to make.
  if( rhs.pSBI ) {
    pSBI = new SmallBlurryImage(rhs);
  }
  else {
    pSBI = NULL;
  }

  im_cl = rhs.im_cl;//@hack by camparijet for adding keyframe to color
  return *this;
}

// ThinCandidates() Thins out a key-frame's candidate list.
// Candidates are those salient corners where the mapmaker will attempt
// to make a new map point by epipolar search. We don't want to make new points
// where there are already existing map points, this routine erases such candidates.
// Operates on a single level of a keyframe.
void KeyFrame::ThinCandidates(int nLevel)
{
  vector<Candidate> &vCSrc = aLevels[nLevel].vCandidates;
  vector<Candidate> vCGood;
  vector<ImageRef> irBusyLevelPos;
  // Make a list of `busy' image locations, which already have features at the same level
  // or at one level higher.
  for(meas_it it = mMeasurements.begin(); it != mMeasurements.end(); ++it)
  {
    if(!(it->second.nLevel == nLevel || it->second.nLevel == nLevel + 1))
      continue;
    irBusyLevelPos.push_back(ir_rounded(it->second.v2RootPos / LevelScale(nLevel)));
  }

  // Only keep those candidates further than 10 pixels away from busy positions.
  unsigned int nMinMagSquared = 10*10;
  for(size_t i=0; i<vCSrc.size(); ++i)
  {
    const ImageRef& irC = vCSrc[i].irLevelPos;
    bool bGood = true;
    for(size_t j=0; j<irBusyLevelPos.size(); ++j)
    {
      const ImageRef& irB = irBusyLevelPos[j];
      if((irB - irC).mag_squared() < nMinMagSquared)
      {
        bGood = false;
        break;
      }
    }
    if(bGood)
      vCGood.push_back(vCSrc[i]);
  }
  vCSrc = vCGood;
}

// Calculates the depth(z-) distribution of map points visible in a keyframe
// This function is only used for the first two keyframes - all others
// get this filled in by the tracker
void KeyFrame::RefreshSceneDepth()
{
  double dSumDepth = 0.0;
  double dSumDepthSquared = 0.0;
  int nMeas = 0;
  for(meas_it it = mMeasurements.begin(); it != mMeasurements.end(); ++it)
  {
    MapPoint &point = *it->first;
    Vector<3> v3PosK = se3CfromW * point.v3WorldPos;
    dSumDepth += v3PosK[2];
    dSumDepthSquared += v3PosK[2] * v3PosK[2];
    nMeas++;
  }

  assert(nMeas > 2); // If not then something is seriously wrong with this KF!!

  // Update scene depth variables
  dSceneDepthMean = dSumDepth / nMeas;
  dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (dSceneDepthMean) * (dSceneDepthMean));
}



/**
 * Perpares a Keyframe from an image. Generates pyramid levels, does FAST detection, etc.
 * Does not fully populate the keyframe struct, but only does the bits needed for the tracker;
 * e.g. does not perform FAST nonmax suppression. Things like that which are needed by the
 * mapmaker but not the tracker go in MakeKeyFrame_Rest();
 * @param im image to make keyframe from
 */
void KeyFrame::MakeKeyFrame_Lite(BasicImage<CVD::byte> &im)
{

  // First, copy out the image data to the pyramid's zero level.
  aLevels[0].im.resize(im.size());
  copy(im, aLevels[0].im);

  // Then, for each level...
  for(int i=0; i<LEVELS; i++)
    {
      Level &lev = aLevels[i];
      if(i!=0)
        {  // .. make a half-size image from the previous level..
          lev.im.resize(aLevels[i-1].im.size() / 2);
          halfSample(aLevels[i-1].im, lev.im);
        }

      // .. and detect and store FAST corner points.
      // I use a different threshold on each level; this is a bit of a hack
      // whose aim is to balance the different levels' relative feature densities.
      lev.vCandidates.clear();
      lev.vMaxCorners.clear();
      lev.vCorners.clear();

      // Removed the if-cases and replaced it with a array lookup @dhenell
      int barrierPerLevel[] = { 10, 15, 15, 10 };
      int barrier = barrierPerLevel[i];

      // Sometimes several 100 000 corners are found. This amount of corners make the
      // program slow and freeze if it is during the stereo initialization. Thus,
      // this code aims to keep the amount of corners below a certain number.
      // 50 000 is chosen quite arbitrary but that number seems to work without
      // freezing the computer.
      // @dhenell

      vector<ImageRef> vTmpCorners;

      do {
        vTmpCorners.clear();
        fast_corner_detect_10(lev.im, vTmpCorners, barrier);
        barrier += 5;
      } while (vTmpCorners.size() > 50000);

      // Added a ROI for corners just for testing if it had any effects.
      // @dhenell
      double border = 0.10f;
      int minBorderX = border * lev.im.size().x;
      int maxBorderX = (1.0 - border) * lev.im.size().x;
      int minBorderY = border * lev.im.size().y;
      int maxBorderY = (1.0 - border) * lev.im.size().y;

      for (auto it = vTmpCorners.begin(); it != vTmpCorners.end(); ++it) {
        //if (it->x > minBorderX && it->y > minBorderY && it->x < maxBorderX && it->y < maxBorderY)
        {
          lev.vCorners.push_back(*it);
        }
      }

     // cout << "Level " << i << " : " << lev.vCorners.size() << "  " << barrier << endl;


      // Generate row look-up-table for the FAST corner points: this speeds up
      // finding close-by corner points later on.
      lev.vCornerRowLUT.clear();
      unsigned int v=0;
      size_t numCorners = lev.vCorners.size();
      for(int y=0; y<lev.im.size().y; y++)
        {
          while( (v < numCorners) && (y > lev.vCorners[v].y) )
            v++;
          lev.vCornerRowLUT.push_back(v);
        }
    }
}

  /**
   * @hack by camparijet
   * for adding Image to KeyFrame
   * Method for adding KeyFrame...
   */
  void KeyFrame::AddRgbToKeyFrame(BasicImage<CVD::Rgb<CVD::byte> > &im_color){
     im_cl.resize(im_color.size());
     copy(im_color, im_cl);
     //cout << "Raw :" << im_cl[0][0]  << endl; @hack for debugging
     //cout << "Raw :" << im_cl.size()  << endl; //@hack @todo for debugging
  }
  /*void KeyFrame::MakeKeyFrame_Lite(BasicImage<CVD::byte> &im, BasicImage<CVD::Rgb<CVD::byte> > &im_color)
{
  im_cl.resize(im_color.size());
  copy(im_color, im_cl);
  MakeKeyFrame_Lite(im);
  }*/

/**
 * Fills the rest of the keyframe structure needed by the mapmaker:
 * FAST nonmax suppression, generation of the list of candidates for further map points,
 * creation of the relocaliser's SmallBlurryImage.
 */
void KeyFrame::MakeKeyFrame_Rest()
{
  static gvar3<double> gvdCandidateMinSTScore("MapMaker.CandidateMinShiTomasiScore", 70, SILENT);
  // For each level...
  for(int l=0; l<LEVELS; l++)
    {
      Level &lev = aLevels[l];
      // .. find those FAST corners which are maximal..
      fast_nonmax(lev.im, lev.vCorners, 10, lev.vMaxCorners);
      // .. and then calculate the Shi-Tomasi scores of those, and keep the ones with
      // a suitably high score as Candidates, i.e. points which the mapmaker will attempt
      // to make new map points out of.
      for(vector<ImageRef>::iterator i=lev.vMaxCorners.begin(); i!=lev.vMaxCorners.end(); i++)
        {
          if(!lev.im.in_image_with_border(*i, 10))
            continue;
          double dSTScore = FindShiTomasiScoreAtPoint(lev.im, 3, *i);
          if(dSTScore > *gvdCandidateMinSTScore)
            {
              Candidate c;
              c.irLevelPos = *i;
              c.dSTScore = dSTScore;
              lev.vCandidates.push_back(c);
            }
        }
    }

  // Also, make a SmallBlurryImage of the keyframe: The relocaliser uses these.
  pSBI = new SmallBlurryImage(*this);
  // Relocaliser also wants the jacobians..
  pSBI->MakeJacs();
}


/**
 * Level needs its own operator= to override CVD's reference-counting behaviour.
 * @param rhs the level to copy
 * @return the copied level
 */
Level& Level::operator=(const Level &rhs)
{
  // Operator= should physically copy pixels, not use CVD's reference-counting image copy.
  im.resize(rhs.im.size());
  copy(rhs.im, im);

  vCorners = rhs.vCorners;
  vMaxCorners = rhs.vMaxCorners;
  vCornerRowLUT = rhs.vCornerRowLUT;
  return *this;
}

// -------------------------------------------------------------
// Some useful globals defined in LevelHelpers.h live here:
Vector<3> gavLevelColors[LEVELS];

// These globals are filled in here. A single static instance of this struct is run before main()
struct LevelHelpersFiller // Code which should be initialised on init goes here; this runs before main()
{
  LevelHelpersFiller()
  {
    for(int i=0; i<LEVELS; i++)
      {
        if(i==0)  gavLevelColors[i] = makeVector( 1.0, 0.0, 0.0);
        else if(i==1)  gavLevelColors[i] = makeVector( 1.0, 1.0, 0.0);
        else if(i==2)  gavLevelColors[i] = makeVector( 0.0, 1.0, 0.0);
        else if(i==3)  gavLevelColors[i] = makeVector( 0.0, 0.0, 0.7);
        else gavLevelColors[i] =  makeVector( 1.0, 1.0, 0.7); // In case I ever run with LEVELS > 4
      }
  }
};
static LevelHelpersFiller foo;


}
