// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited
//
// SmallBlurryImage-based relocaliser
//
// Each KF stores a small, blurred version of itself;
// Just compare a small, blurred version of the input frame to all the KFs,
// choose the closest match, and then estimate a camera rotation by direct image
// minimisation.
//
// This has been modified to search the keyframes of all maps to allow map switching


#ifndef __RELOCALISER_H
#define __RELOCALISER_H

#include "ATANCamera.h"
#include "SmallBlurryImage.h"

#include "Map.h"

namespace PTAMM {


class Relocaliser
{
public:
  Relocaliser(const ATANCamera &camera);
  bool AttemptRecovery(const Map & currentMap, KeyFrame &k);

  const SE3<>& BestPose() const { return mse3Best; }

protected:
  void ScoreKFs(const Map &map, const SmallBlurryImage &currentSBI);

  ATANCamera mCamera;
  int mnBest;
  double mdBestScore;
  SE3<> mse3Best;
};

}

#endif

