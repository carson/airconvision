//
//    nonMaximumSuppression - this nonMaximumSuppression is a re-implementation
//                            of the NMS as proposed by Rosten. However, this
//                            implementation is complete and about 40% faster
//                            than the OpenCV-version
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

#ifndef NONMAXIMUMSUPPRESSION_H_
#define NONMAXIMUMSUPPRESSION_H_

#include <cvd/image_ref.h>
#include <vector>

void nonMaximumSuppression(const std::vector<CVD::ImageRef>& corners_all, const std::vector<int>& scores, std::vector<CVD::ImageRef>& corners_max);

#endif /* NONMAXIMUMSUPPRESSION_H_ */
