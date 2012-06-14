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

#include <cstdlib>
#include "nonMaximumSuppression.h"

static std::vector<int> nmsFlags; //-1 max, else index to max

void nonMaximumSuppression(const std::vector<CVD::ImageRef>& corners_all,
                           const std::vector<int>& scores,
                           std::vector<CVD::ImageRef>& corners_max)
{
	int lastRow=0, next_lastRow=0;
	int lastRowCorner_ind=0, next_lastRowCorner_ind=0;

	nmsFlags.resize(corners_all.size());

	//set all flags to MAXIMUM
	std::fill(nmsFlags.begin(), nmsFlags.end(), -1);

	for (int currCorner_ind=0; currCorner_ind<corners_all.size(); currCorner_ind++)
	{
	        const CVD::ImageRef *currCorner = &corners_all[currCorner_ind];

		int t;

		//check above
		if(lastRow+1 < currCorner->y)
		{
			lastRow=next_lastRow;
			lastRowCorner_ind=next_lastRowCorner_ind;
		}
		if(next_lastRow!=currCorner->y)
		{
			next_lastRow=currCorner->y;
			next_lastRowCorner_ind=currCorner_ind;
		}
		if(lastRow+1==currCorner->y)
		{
			//find the corner above the current one
			while((corners_all[lastRowCorner_ind].x < currCorner->x) && (corners_all[lastRowCorner_ind].y == lastRow))
				lastRowCorner_ind++;

			if( (corners_all[lastRowCorner_ind].x == currCorner->x) && (lastRowCorner_ind!=currCorner_ind) )
			{
				int t=lastRowCorner_ind;
				while(nmsFlags[t]!=-1) //find the maximum in this block
					t=nmsFlags[t];

				if( scores[currCorner_ind] < scores[t] )
				{
					nmsFlags[currCorner_ind]=t;
				}
				else
					nmsFlags[t]=currCorner_ind;
			}
		}

		//check left
		t=currCorner_ind-1;
		if( (currCorner_ind!=0) && (corners_all[t].y == currCorner->y) && (corners_all[t].x+1 == currCorner->x) )
		{
			int currCornerMaxAbove_ind=nmsFlags[currCorner_ind];

			while(nmsFlags[t]!=-1) //find the maximum in that area
				t=nmsFlags[t];

			if(currCornerMaxAbove_ind==-1) //no maximum above
			{
				if(t!=currCorner_ind)
				{
					if( scores[currCorner_ind] < scores[t] )
						nmsFlags[currCorner_ind]=t;
					else
						nmsFlags[t]=currCorner_ind;
				}
			}
			else	//maximum above
			{
				if(t!=currCornerMaxAbove_ind)
				{
					if(scores[currCornerMaxAbove_ind] < scores[t])
					{
						nmsFlags[currCornerMaxAbove_ind]=t;
						nmsFlags[currCorner_ind]=t;
					}
					else
					{
						nmsFlags[t]=currCornerMaxAbove_ind;
						nmsFlags[currCorner_ind]=currCornerMaxAbove_ind;
					}
				}
			}
		}

		currCorner++;
	}

        corners_max.clear();
        corners_max.reserve(corners_all.size());

	//collecting maximum corners
	for(int currCorner_ind=0; currCorner_ind<corners_all.size(); currCorner_ind++)
	{
		if(nmsFlags[currCorner_ind] == -1)
		{
		        corners_max.push_back(corners_all[currCorner_ind]);
		}
	}
}

