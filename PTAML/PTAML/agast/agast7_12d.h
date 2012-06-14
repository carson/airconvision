//
//    agast7d - AGAST, an adaptive and generic corner detector based on the
//              accelerated segment test for a 12 pixel mask in diamond format
//
//    Copyright (C) 2010  Elmar Mair
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef AGAST7_12D_H
#define AGAST7_12D_H

#include <cvd/image_ref.h>
#include <vector>

void agast7_12d(const unsigned char* im, int xsize, int ysize, int stride, int b, std::vector<CVD::ImageRef> &corners);
void agast7_12d_nms(const unsigned char* im, int stride, int b, const std::vector<CVD::ImageRef>& corners_all, std::vector<CVD::ImageRef>& corners_max);

#endif /* AGAST7_12D_H */
