// Copyright 2009 Isis Innovation Limited

/****************************************
  PTAMM: The relocalizer has been modified
  to allow it to search all maps for a match.
  If it recovers to the current map then
  behaviour is as before.
  If it recovers to another map then it signals
  for the Tracker, MapMaker, MapViewer and
  System to change maps.

****************************************/

#include "Relocaliser.h"
#include "SmallBlurryImage.h"
#include <cvd/utility.h>
#include <gvars3/instances.h>
#include <TooN/se2.h>

namespace PTAMM {

using namespace CVD;
using namespace std;
using namespace GVars3;

/**
 * Create the relocalizer
 * @param maps The reference to all of the maps for searching.
 * @param camera The camera model
 */
Relocaliser::Relocaliser(const ATANCamera &camera)
  : mCamera(camera)
{
}

/**
 * Attempt to recover the camera pose.
 * Searches all maps for a best match.
 * @param currentMap The current map
 * @param kCurrent The current camera image
 * @return sucess or failure
 */
bool Relocaliser::AttemptRecovery(const Map &currentMap, KeyFrame &kCurrent)
{
  // Ensure the incoming frame has a SmallBlurryImage attached
  SmallBlurryImage sbi(kCurrent);

  // Find the best ZMSSD match from all keyframes in all maps
  ScoreKFs(currentMap, sbi);
  //mnBest = currentMap.GetKeyFrames().size() - 1;

  if (mnBest < 0) {
    return false;
  }

  const vector<KeyFrame*>& vKeyFrames = currentMap.GetKeyFrames();

  assert(mnBest < (int)vKeyFrames.size());

  // And estimate a camera rotation from a 3DOF image alignment
  SmallBlurryImage *pOtherSbi = vKeyFrames[mnBest]->pSBI;
  assert(pOtherSbi != NULL);
  pair<SE2<>, double> result_pair = sbi.IteratePosRelToTarget(*pOtherSbi, 6);
  SE2<> mse2 = result_pair.first;
  double dScore = result_pair.second;

  SE3<> se3KeyFramePos = currentMap.GetKeyFrames()[mnBest]->se3CfromW;
  mse3Best = SmallBlurryImage::SE3fromSE2(mse2, mCamera) * se3KeyFramePos;

  //cout << "Reloc score: " << dScore << endl;

  return dScore < GV3::get<double>("Reloc2.MaxScore", 9e6, SILENT);
}

/**
 * Compare current KF to all KFs stored in map by
 * Zero-mean SSD
 * @param pMap the map to search
 * @param kCurrent the current camera frame
 */
void Relocaliser::ScoreKFs(const Map &map, const SmallBlurryImage &currentSBI)
{
  mdBestScore = 9e6;
  mnBest = -1;

  const std::vector<KeyFrame*>& keyFrames = map.GetKeyFrames();
  for (size_t i = 0; i < keyFrames.size(); ++i) {
    double dSSD = currentSBI.ZMSSD(*keyFrames[i]->pSBI);
    if (dSSD < mdBestScore) {
      mdBestScore = dSSD;
      mnBest = i;
    }
  }
}

}
