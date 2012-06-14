#ifndef __RECTANGLE_H
#define __RECTANGLE_H

#include <cvd/image_ref.h>

namespace PTAMM {

using namespace CVD;

class Rectangle {
  public:
    Rectangle();
//    Rectangle(int x, int y, int width, int height);
    Rectangle(double x, double y, double width, double height);
    Rectangle(const ImageRef &irPosition, const ImageRef &irSize);

    bool Contains(const ImageRef &irPoint) const;

    double x, y, width, height;
};

}

#endif
