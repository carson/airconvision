// -*- c++ -*-
// Copyright 2009 Isis Innovation Limited

/********************************************************************

  A set of utility functions
  Author: Robert Castle, 2009, bob@robots.ox.ac.uk

********************************************************************/

#ifndef __PTAMM_UTILS__
#define __PTAMM_UTILS__

#include <cvd/image_ref.h>

#include <string>
#include <iostream>
#include <iterator>
#include <vector>
#include <algorithm>
#include <numeric>

namespace PTAMM {

/**
 * Output a vector as a stream that is space separated.
 * @param os Output stream
 * @param v Vector to output
 * @return the stream output
 */
template<class T>
  std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
  std::copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " "));
  return os;
}

/**
 * Returns the indices of the sorted vector.
 *
 * Taken from http://stackoverflow.com/questions/10580982/c-sort-keeping-track-of-indices
 */
template <typename T>
std::vector<size_t> ordered(std::vector<T> const& values) {
    std::vector<size_t> indices(values.size());
    std::iota(begin(indices), end(indices), static_cast<size_t>(0));

    std::sort(
        begin(indices), end(indices),
        [&](size_t a, size_t b) { return values[a] > values[b]; }
    );
    return indices;
}


void PruneWhiteSpace(std::string & str);

bool PointInsideRect(const CVD::ImageRef &pt, const CVD::ImageRef &start, const CVD::ImageRef &size);

}

#endif
