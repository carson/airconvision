//
//    agast7s - AGAST, an adaptive and generic corner detector based on the
//              accelerated segment test for a 12 pixel mask in square format
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

#ifndef AGAST7_12S_H
#define AGAST7_12S_H

#include <cvd/image_ref.h>
#include <vector>

void agast7_12s(const unsigned char* im, int xsize, int ysize, int pitch, int b, std::vector<CVD::ImageRef> &corners);
void agast7_12s_nms(const unsigned char* im, int stride, int b, const std::vector<CVD::ImageRef>& corners_all, std::vector<CVD::ImageRef>& corners_max);
void agast7_12s_nms_with_scores(const unsigned char* im, int stride, int b,
                                const std::vector<CVD::ImageRef>& corners_all,
                                std::vector<CVD::ImageRef>& corners_max,
                                std::vector<int>& scores);

#endif /* AGAST7_12S_H */
