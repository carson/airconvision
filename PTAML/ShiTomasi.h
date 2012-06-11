// Copyright 2008 Isis Innovation Limited
#ifndef __SHI_TOMASI__H
#define __SHI_TOMASI__H

#include <cvd/image.h>
#include <cvd/byte.h>

namespace PTAMM {

double FindShiTomasiScoreAtPoint(const CVD::BasicImage<CVD::byte> &image,
                                 int nHalfBoxSize,
                                 const CVD::ImageRef &irCenter);


}

#endif
