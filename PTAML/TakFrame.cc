/*#include "TakFrame.h"

namespace PTAMM {

using namespace CVD;
using namespace std;

  TakFrame::TakFrame():
    _id(9)
  {
    std::cout << "hoge" << std::endl;
  }
  
  TakFrame::TakFrame(const TakFrame& takFrame):
    _id(takFrame._id)
  {
    
  }


  /*  TakFrame::TakFrame(int id, Image<Rgb<CVD::byte> > image, SE3<> pose):
    _id(id),
    im(image),
    mse3CamFromWorld(pose),
    isKeyFrame(false)
  {
    
  }
}*/

