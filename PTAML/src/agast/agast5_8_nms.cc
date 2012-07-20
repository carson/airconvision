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

#include <cstdint>
#include <cstdlib>
#include "nonMaximumSuppression.h"
#include "agast5_8_init.h"
#include "agast5_8.h"

//using also bisection as propsed by Edward Rosten in FAST,
//but it is based on the OAST
int agast5_8_cornerScore(const unsigned char* p, int bstart)
{
    int bmin = bstart;
    int bmax = 255;
    int b = (bmax + bmin)/2;

	register int_fast16_t offset0=s_offset0;
	register int_fast16_t offset1=s_offset1;
	register int_fast16_t offset2=s_offset2;
	register int_fast16_t offset3=s_offset3;
	register int_fast16_t offset4=s_offset4;
	register int_fast16_t offset5=s_offset5;
	register int_fast16_t offset6=s_offset6;
	register int_fast16_t offset7=s_offset7;

	while(1)
	{
		register const int cb = *p + b;
		register const int c_b = *p - b;
		if(p[offset0] > cb)
		  if(p[offset2] > cb)
		    if(p[offset3] > cb)
		      if(p[offset5] > cb)
		        if(p[offset1] > cb)
		          if(p[offset4] > cb)
		            goto is_a_corner;
		          else
		            if(p[offset7] > cb)
		              goto is_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          if(p[offset4] > cb)
		            if(p[offset6] > cb)
		              goto is_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		      else
		        if(p[offset1] > cb)
		          if(p[offset4] > cb)
		            goto is_a_corner;
		          else
		            if(p[offset7] > cb)
		              goto is_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		    else
		      if(p[offset7] > cb)
		        if(p[offset6] > cb)
		          if(p[offset5] > cb)
		            if(p[offset1] > cb)
		              goto is_a_corner;
		            else
		              if(p[offset4] > cb)
		                goto is_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            if(p[offset1] > cb)
		              goto is_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        if(p[offset5] < c_b)
		          if(p[offset3] < c_b)
		            if(p[offset7] < c_b)
		              if(p[offset4] < c_b)
		                if(p[offset6] < c_b)
		                  goto is_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		  else
		    if(p[offset5] > cb)
		      if(p[offset7] > cb)
		        if(p[offset6] > cb)
		          if(p[offset1] > cb)
		            goto is_a_corner;
		          else
		            if(p[offset4] > cb)
		              goto is_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		    else
		      if(p[offset5] < c_b)
		        if(p[offset3] < c_b)
		          if(p[offset2] < c_b)
		            if(p[offset1] < c_b)
		              if(p[offset4] < c_b)
		                goto is_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              if(p[offset4] < c_b)
		                if(p[offset6] < c_b)
		                  goto is_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            if(p[offset7] < c_b)
		              if(p[offset4] < c_b)
		                if(p[offset6] < c_b)
		                  goto is_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		else if(p[offset0] < c_b)
		  if(p[offset2] < c_b)
		    if(p[offset7] > cb)
		      if(p[offset3] < c_b)
		        if(p[offset5] < c_b)
		          if(p[offset1] < c_b)
		            if(p[offset4] < c_b)
		              goto is_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            if(p[offset4] < c_b)
		              if(p[offset6] < c_b)
		                goto is_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          if(p[offset1] < c_b)
		            if(p[offset4] < c_b)
		              goto is_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		      else
		        if(p[offset5] > cb)
		          if(p[offset3] > cb)
		            if(p[offset4] > cb)
		              if(p[offset6] > cb)
		                goto is_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		    else
		      if(p[offset7] < c_b)
		        if(p[offset3] < c_b)
		          if(p[offset5] < c_b)
		            if(p[offset1] < c_b)
		              goto is_a_corner;
		            else
		              if(p[offset4] < c_b)
		                if(p[offset6] < c_b)
		                  goto is_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            if(p[offset1] < c_b)
		              goto is_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          if(p[offset6] < c_b)
		            if(p[offset5] < c_b)
		              if(p[offset1] < c_b)
		                goto is_a_corner;
		              else
		                if(p[offset4] < c_b)
		                  goto is_a_corner;
		                else
		                  goto is_not_a_corner;
		            else
		              if(p[offset1] < c_b)
		                goto is_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		      else
		        if(p[offset3] < c_b)
		          if(p[offset5] < c_b)
		            if(p[offset1] < c_b)
		              if(p[offset4] < c_b)
		                goto is_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              if(p[offset4] < c_b)
		                if(p[offset6] < c_b)
		                  goto is_a_corner;
		                else
		                  goto is_not_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            if(p[offset1] < c_b)
		              if(p[offset4] < c_b)
		                goto is_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		  else
		    if(p[offset5] > cb)
		      if(p[offset3] > cb)
		        if(p[offset2] > cb)
		          if(p[offset1] > cb)
		            if(p[offset4] > cb)
		              goto is_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            if(p[offset4] > cb)
		              if(p[offset6] > cb)
		                goto is_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          if(p[offset7] > cb)
		            if(p[offset4] > cb)
		              if(p[offset6] > cb)
		                goto is_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		    else
		      if(p[offset5] < c_b)
		        if(p[offset7] < c_b)
		          if(p[offset6] < c_b)
		            if(p[offset1] < c_b)
		              goto is_a_corner;
		            else
		              if(p[offset4] < c_b)
		                goto is_a_corner;
		              else
		                goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		else
		  if(p[offset3] > cb)
		    if(p[offset5] > cb)
		      if(p[offset2] > cb)
		        if(p[offset1] > cb)
		          if(p[offset4] > cb)
		            goto is_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          if(p[offset4] > cb)
		            if(p[offset6] > cb)
		              goto is_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		      else
		        if(p[offset7] > cb)
		          if(p[offset4] > cb)
		            if(p[offset6] > cb)
		              goto is_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		        else
		          goto is_not_a_corner;
		    else
		      goto is_not_a_corner;
		  else
		    if(p[offset3] < c_b)
		      if(p[offset5] < c_b)
		        if(p[offset2] < c_b)
		          if(p[offset1] < c_b)
		            if(p[offset4] < c_b)
		              goto is_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            if(p[offset4] < c_b)
		              if(p[offset6] < c_b)
		                goto is_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		        else
		          if(p[offset7] < c_b)
		            if(p[offset4] < c_b)
		              if(p[offset6] < c_b)
		                goto is_a_corner;
		              else
		                goto is_not_a_corner;
		            else
		              goto is_not_a_corner;
		          else
		            goto is_not_a_corner;
		      else
		        goto is_not_a_corner;
		    else
		      goto is_not_a_corner;

		is_a_corner:
			bmin=b;
			goto end;

		is_not_a_corner:
			bmax=b;
			goto end;

		end:

		if(bmin == bmax - 1 || bmin == bmax)
			return bmin;
		b = (bmin + bmax) / 2;
	}
}


void agast5_8_score(const unsigned char* i, int stride, const std::vector<CVD::ImageRef>& corners_all, int b, std::vector<int>& scores)
{
	init5_8_pattern(stride);

	for(size_t n = 0; n < corners_all.size(); ++n)
        scores[n] = agast5_8_cornerScore(i + corners_all[n].y*stride + corners_all[n].x, b);
}

void agast5_8_nms(const unsigned char* im, int stride, int b, const std::vector<CVD::ImageRef>& corners_all, std::vector<CVD::ImageRef>& corners_max)
{
  std::vector<int> scores(corners_all.size(), 0);
  agast5_8_score(im, stride, corners_all, b, scores);
  nonMaximumSuppression(corners_all, scores, corners_max);
}

void agast5_8_nms_with_scores(const unsigned char* im, int stride, int b,
                              const std::vector<CVD::ImageRef>& corners_all,
                              std::vector<CVD::ImageRef>& corners_max,
                              std::vector<int>& scores)
{
  scores.resize(corners_all.size());
  agast5_8_score(im, stride, corners_all, b, scores);
  nonMaximumSuppression(corners_all, scores, corners_max);
}

