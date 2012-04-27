#ifndef __ARTOOLKIT_H
#define __ARTOOLKIT_H

#include <cvd/image.h>
#include <cvd/byte.h>

namespace PTAMM {

bool InitARToolkit(const CVD::ImageRef& imSize);
bool DistanceToMarkerPlane(const CVD::Image<CVD::byte> &imFrame, float& dist);

}

#endif
