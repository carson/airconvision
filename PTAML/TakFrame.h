#ifndef __TAK_FRAME__
#define __TAK_FRAME__

/*#include "MapMaker.h"*/
#include <TooN/se3.h>
#include <cvd/image.h>
#include <cvd/rgb.h>

namespace PTAMM{

  struct TakFrame{
    int _id;
    TooN::SE3<> CamFromWorld;   
    int isKeyFrame; // 0 : false, 1 : true, 2 : initial
    //CVD::Image<CVD::Rgb<CVD::byte> > im; //@hack by camparijet for track frame    
  };
  /*{
    //TakFrame();
    //TakFrame(const TakFrame& takFrame);
    TakFrame(int id);
    TakFrame(int id, CVD::Image<CVD::Rgb<CVD::byte> > image, SE3<> pose);
    int _id;
    CVD::Image< CVD::Rgb < CVD::byte> > im; //@hack by camparijet for track frame
    //SE3<> mse3CamFromWorld;   
    //bool isKeyFrame;
      
  };*/
}

#endif
