#include "FPSCounter.h"
#include "Timing.h"

#include <cvd/image_io.h>
#include <cvd/fast_corner.h>

#include <cstdlib>
#include <iostream>

using namespace std;
using namespace CVD;
using namespace PTAMM;

int main(int argc, char* argv[])
{
  CVD::Image<CVD::byte> imDifficult;
  img_load(imDifficult, "difficult_fast.png");

  CVD::Image<CVD::byte> imEasy;
  img_load(imEasy, "easy_fast.png");


  vector<ImageRef> vCorners;

  while (true) {
    cout << "Difficult: ";
    Tic();
    fast_corner_detect_10(imDifficult, vCorners, 15);
    Toc();
    vCorners.clear();
    cout << "Easy: ";
    Tic();
    fast_corner_detect_10(imEasy, vCorners, 15);
    Toc();
    vCorners.clear();
  }

  return 0;
}
