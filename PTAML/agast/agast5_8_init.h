//
//    agast5 - AGAST, an adaptive and generic corner detector based on the
//              accelerated segment test for a 8 pixel mask
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

#ifndef AGAST5_8_INIT_H
#define AGAST5_8_INIT_H

#include <cstdint>

static int s_width=-1;
static int_fast16_t s_offset0;
static int_fast16_t s_offset1;
static int_fast16_t s_offset2;
static int_fast16_t s_offset3;
static int_fast16_t s_offset4;
static int_fast16_t s_offset5;
static int_fast16_t s_offset6;
static int_fast16_t s_offset7;

static __inline void init5_8_pattern(int image_width)
{
	if(image_width==s_width)
		return;

	s_width=image_width;

	s_offset0=(-1)+(0)*s_width;
	s_offset1=(-1)+(-1)*s_width;
	s_offset2=(0)+(-1)*s_width;
	s_offset3=(1)+(-1)*s_width;
	s_offset4=(1)+(0)*s_width;
	s_offset5=(1)+(1)*s_width;
	s_offset6=(0)+(1)*s_width;
	s_offset7=(-1)+(1)*s_width;
}


#endif /* AGAST5_8_INIT_H */
