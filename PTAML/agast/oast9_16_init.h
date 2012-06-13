//
//    oast9 - OAST, an optimal corner detector based on the
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

#ifndef OAST9_16_INIT_H
#define OAST9_16_INIT_H

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
static int_fast16_t s_offset12;
static int_fast16_t s_offset13;
static int_fast16_t s_offset14;
static int_fast16_t s_offset15;

static __inline void init9_16_pattern(int image_width)
{
	if(image_width==s_width)
		return;

	s_width=image_width;

	s_offset0=(-3)+(0)*s_width;
	s_offset1=(-3)+(-1)*s_width;
	s_offset2=(-2)+(-2)*s_width;
	s_offset3=(-1)+(-3)*s_width;
	s_offset4=(0)+(-3)*s_width;
	s_offset5=(1)+(-3)*s_width;
	s_offset6=(2)+(-2)*s_width;
	s_offset7=(3)+(-1)*s_width;
	s_offset8=(3)+(0)*s_width;
	s_offset9=(3)+(1)*s_width;
	s_offset10=(2)+(2)*s_width;
	s_offset11=(1)+(3)*s_width;
	s_offset12=(0)+(3)*s_width;
	s_offset13=(-1)+(3)*s_width;
	s_offset14=(-2)+(2)*s_width;
	s_offset15=(-3)+(1)*s_width;
}

#endif /* OAST9_16_INIT_H */
