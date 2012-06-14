//
//    agast9 - OAST, an optimal corner detector based on the
//              accelerated segment test for a 16 pixel mask
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

#ifndef OAST9_16_H
#define OAST9_16_H

#include <cvd/image_ref.h>
#include <vector>

void oast9_16(const unsigned char* im, int xsize, int ysize, int stride, int b, std::vector<CVD::ImageRef> &corners);
void oast9_16_nms(const unsigned char* im, int stride, int b, const std::vector<CVD::ImageRef>& corners_all, std::vector<CVD::ImageRef>& corners_max);
void oast9_16_nms_with_scores(const unsigned char* im, int stride, int b,
                              const std::vector<CVD::ImageRef>& corners_all,
                              std::vector<CVD::ImageRef>& corners_max,
                              std::vector<int>& scores);

#endif /* OAST9_16_H */
