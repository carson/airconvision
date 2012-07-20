#include "Rectangle.h"

namespace PTAMM {

Rectangle::Rectangle()
  : x(0)
  , y(0)
  , width(0)
  , height(0)
{
}

//Rectangle::Rectangle(int x, int y, int width, int height)
Rectangle::Rectangle(double x, double y, double width, double height)
: x(x)
, y(y)
, width(width)
, height(height)
{
}

Rectangle::Rectangle(const ImageRef &irPosition, const ImageRef &irSize)
  : x(irPosition.x)
  , y(irPosition.y)
  , width(irSize.x)
  , height(irSize.y)
{
}

bool Rectangle::Contains(const ImageRef &irPoint) const
{
  return irPoint.x >= x && irPoint.y >= y &&
         irPoint.x < x + width && irPoint.y < y + height;
}

}

