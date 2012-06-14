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

#ifndef AGAST7_12D_INIT_H
#define AGAST7_12D_INIT_H

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
static int_fast16_t s_offset8;
static int_fast16_t s_offset9;
static int_fast16_t s_offset10;
static int_fast16_t s_offset11;

static __inline void init7_12d_pattern(int image_width)
{
	if(image_width==s_width)
		return;

	s_width=image_width;

	s_offset0=(-3)+(0)*s_width;
	s_offset1=(-2)+(-1)*s_width;
	s_offset2=(-1)+(-2)*s_width;
	s_offset3=(0)+(-3)*s_width;
	s_offset4=(1)+(-2)*s_width;
	s_offset5=(2)+(-1)*s_width;
	s_offset6=(3)+(0)*s_width;
	s_offset7=(2)+(1)*s_width;
	s_offset8=(1)+(2)*s_width;
	s_offset9=(0)+(3)*s_width;
	s_offset10=(-1)+(2)*s_width;
	s_offset11=(-2)+(1)*s_width;
}

#endif /* AGAST7_12D_INIT_H */
