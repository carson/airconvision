// Copyright 2008 Isis Innovation Limited
#include "KeyFrame.h"
#include "SmallBlurryImage.h"
#include "LevelHelpers.h"
#include "MapPoint.h"
#include "Utils.h"
#include "Timing.h"

#include <cvd/image.h>
#include <cvd/vision.h>

#include <cassert>

namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;

int g_nNumFeaturesFound[LEVELS] = { 0 };

Level::Level()
  : bImplaneCornersCached(false)
  , mpFeatureGrid(NULL)
  , mbHasAllFeatures(false)
  , mbHasBestFeatures(false)
{
}

Level::Level(const Level& rhs)
  : bImplaneCornersCached(false)
  , mpFeatureGrid(NULL)
  , mbHasAllFeatures(false)
  , mbHasBestFeatures(false)
{
  *this = rhs;
}


Level::~Level()
{
  delete mpFeatureGrid;
}

/**
 * Level needs its own operator= to override CVD's reference-counting behaviour.
 * @param rhs the level to copy
 * @return the copied level
 */
Level& Level::operator=(const Level &rhs)
{
  if (this == &rhs) {
    return *this;
  }

  // Operator= should physically copy pixels, not use CVD's reference-counting image copy.
  im.copy_from(rhs.im);

  delete mpFeatureGrid;
  mpFeatureGrid = NULL;

  if (rhs.mpFeatureGrid) {
    mpFeatureGrid = new FeatureGrid(*rhs.mpFeatureGrid);
  }

  bImplaneCornersCached = rhs.bImplaneCornersCached;
  vImplaneCorners = rhs.vImplaneCorners;

  mvAllFeatures = rhs.mvAllFeatures;
  mvBestFeatures = rhs.mvBestFeatures;

  mbHasAllFeatures = rhs.mbHasAllFeatures;
  mbHasBestFeatures = rhs.mbHasBestFeatures;

  return *this;
}

void Level::Init(size_t nWidth, size_t nHeight, size_t nGridRows, size_t nGridCols)
{
  assert(mpFeatureGrid == NULL);
  mpFeatureGrid = new FeatureGrid(nWidth, nHeight, nGridRows, nGridCols, 100, 50, 15);
}

void Level::Clear()
{
  bImplaneCornersCached = false;
  vImplaneCorners.clear();
  mpFeatureGrid->Clear();
  mvAllFeatures.clear();
  mvBestFeatures.clear();
  mbHasAllFeatures = false;
  mbHasBestFeatures = false;
}

void Level::FindFeatures()
{
  if (!mbHasAllFeatures) {
    mvAllFeatures.clear();
    mpFeatureGrid->Clear();
    mpFeatureGrid->FindFeatures(im);
    mpFeatureGrid->GetAllFeatures(mvAllFeatures);
    mbHasAllFeatures = true;
  }
}

const std::vector<CVD::ImageRef>& Level::Features()
{
  if (!mbHasAllFeatures) {
    FindFeatures();
  }

  return mvAllFeatures;
}

void Level::FindBestFeatures()
{
  if (!mbHasBestFeatures) {

    if (!mbHasAllFeatures) {
      FindFeatures();
    }

    mvBestFeatures.clear();
    mpFeatureGrid->FindBestFeatures(im);
    mbHasBestFeatures = true;
  }
}

void Level::GetBestFeatures(size_t nMaxFeatures, std::vector<CVD::ImageRef>& vFeatures)
{
  if (!mbHasBestFeatures) {
    FindBestFeatures();
  }

  mpFeatureGrid->GetBestFeatures(nMaxFeatures, vFeatures);
}

void Level::GetFeaturesInsideCircle(const CVD::ImageRef &irPos, int nRadius, std::vector<CVD::ImageRef> &vFeatures) const
{
  mpFeatureGrid->GetFeaturesInsideCircle(irPos, nRadius, vFeatures);
}

/**
 * Constructer.
 * @param cam the camera that made this keyframe
 */
KeyFrame::KeyFrame(const ATANCamera &cam)
  : pSBI( NULL ),
    Camera(cam)
{
  int width = cam.GetImageSize()[0];
  int height = cam.GetImageSize()[1];
  aLevels[0].Init(width, height,        10, 8);
  aLevels[1].Init(width / 2, height / 2, 6, 6);
  aLevels[2].Init(width / 4, height / 4, 2, 2);
  aLevels[3].Init(width / 8, height / 8, 1, 1);
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

  return *this;
}

// ThinCandidates() Thins out a key-frame's candidate list.
// Candidates are those salient corners where the mapmaker will attempt
// to make a new map point by epipolar search. We don't want to make new points
// where there are already existing map points, this routine erases such candidates.
// Operates on a single level of a keyframe.
void KeyFrame::ThinCandidates(int nLevel, std::vector<CVD::ImageRef>& vCandidates)
{
  vector<ImageRef> vCGood;
  vector<ImageRef> irBusyLevelPos;

  // Make a list of `busy' image locations, which already have features at the same level
  // or at one level higher.
  for(auto it = mMeasurements.begin(); it != mMeasurements.end(); ++it)
  {
    if(!(it->second.nLevel == nLevel || it->second.nLevel == nLevel + 1))
      continue;
    irBusyLevelPos.push_back(ir_rounded(it->second.v2RootPos / LevelScale(nLevel)));
  }

  // Only keep those candidates further than 10 pixels away from busy positions.
  unsigned int nMinMagSquared = 10*10;
  for(size_t i=0; i<vCandidates.size(); ++i)
  {
    const ImageRef& irC = vCandidates[i];
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
      vCGood.push_back(vCandidates[i]);
  }
  vCandidates = vCGood;
}

// Calculates the depth(z-) distribution of map points visible in a keyframe
// This function is only used for the first two keyframes - all others
// get this filled in by the tracker
void KeyFrame::RefreshSceneDepth()
{
  double dSumDepth = 0.0;
  double dSumDepthSquared = 0.0;
  int nMeas = 0;
  for(auto it = mMeasurements.begin(); it != mMeasurements.end(); ++it)
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
void KeyFrame::MakeKeyFrame_Lite(const BasicImage<CVD::byte> &im, int* aFastCornerBarriers)
{
  // First, copy out the image data to the pyramid's zero level.
  aLevels[0].im.copy_from(im);

  for (int i = 1; i < LEVELS; ++i) {
    // .. make a half-size image from the previous level..
    aLevels[i].im.resize(aLevels[i-1].im.size() / 2);
    halfSample(aLevels[i-1].im, aLevels[i].im);
  }

  gFeatureTimer.Start();

  for (int i = 0; i < LEVELS; ++i) {
    aLevels[i].Clear();
    aLevels[i].FindFeatures();
  }

  gFeatureTimer.Stop();
}

/**
 * Fills the rest of the keyframe structure needed by the mapmaker:
 * FAST nonmax suppression, generation of the list of candidates for further map points,
 * creation of the relocaliser's SmallBlurryImage.
 */
void KeyFrame::MakeKeyFrame_Rest()
{
  // Also, make a SmallBlurryImage of the keyframe: The relocaliser uses these.
  pSBI = new SmallBlurryImage(*this);
  // Relocaliser also wants the jacobians..
  pSBI->MakeJacs();
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
