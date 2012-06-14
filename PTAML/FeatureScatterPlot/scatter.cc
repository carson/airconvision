#include "Timing.h"
#include "ShiTomasi.h"

#include "agast/agast5_8.h"
#include "agast/agast7_12s.h"
#include "agast/agast7_12d.h"
#include "agast/oast9_16.h"

#include <cvd/fast_corner.h>
#include <cvd/image_io.h>
#include <cvd/fast_corner.h>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <algorithm>

namespace CVD {
  void fast_corner_detect_plain_7(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
  void fast_corner_detect_plain_8(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
  void fast_corner_detect_plain_9(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
  void fast_corner_detect_plain_10(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
  void fast_corner_detect_plain_11(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
  void fast_corner_detect_plain_12(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
}

enum FeatureDetector {
  PLAIN_FAST7,
  PLAIN_FAST8,
  PLAIN_FAST9,
  PLAIN_FAST10,
  PLAIN_FAST11,
  PLAIN_FAST12,
  FAST10,
  OAST9_16,
  AGAST7_12d,
  AGAST7_12s,
  AGAST5_8,
  NUM_FEATURE_DETECTORS
};

using namespace std;
using namespace CVD;
using namespace PTAMM;

template <typename T1, typename T2>
void unzip(vector<pair<T1,T2>>& v, vector<T1>& v1, vector<T2>& v2)
{
  v1.clear();
  v1.reserve(v.size());
  v2.clear();
  v2.reserve(v.size());

  for (auto it = v.begin(); it != v.end(); ++it) {
    v1.push_back(it->first);
    v2.push_back(it->second);
  }
}

int main(int argc, char* argv[])
{
  CVD::Image<CVD::byte> im;
  img_load(im, "image.pnm");

  ImageRef irSize = im.size();
  int nStride = irSize.x;
  const unsigned char* pData = im.data();


  cout << "Image size: " << irSize << endl;

  for (int i = 0; i < NUM_FEATURE_DETECTORS; ++i) {
    FeatureDetector fd = (FeatureDetector)i;
    cout << "FD: " << i << " ";

    Tic();
    int nBarrier = 15;
    vector<ImageRef> vCorners;

    do {
      vCorners.clear();

      switch (fd) {
      case PLAIN_FAST7:
        fast_corner_detect_plain_7(im, vCorners, nBarrier);
        break;
      case PLAIN_FAST8:
        fast_corner_detect_plain_8(im, vCorners, nBarrier);
        break;
      case PLAIN_FAST9:
        fast_corner_detect_plain_9(im, vCorners, nBarrier);
        break;
      case PLAIN_FAST10:
        fast_corner_detect_plain_10(im, vCorners, nBarrier);
        break;
      case PLAIN_FAST11:
        fast_corner_detect_plain_11(im, vCorners, nBarrier);
        break;
      case PLAIN_FAST12:
        fast_corner_detect_plain_12(im, vCorners, nBarrier);
        break;
      case FAST10:
        fast_corner_detect_10(im, vCorners, nBarrier);
        break;
      case OAST9_16:
        oast9_16(pData, irSize.x, irSize.y, nStride, nBarrier, vCorners);
        break;
      case AGAST7_12d:
        agast7_12d(pData, irSize.x, irSize.y, nStride, nBarrier, vCorners);
        break;
      case AGAST7_12s:
        agast7_12s(pData, irSize.x, irSize.y, nStride, nBarrier, vCorners);
        break;
      case AGAST5_8:
        agast5_8(pData, irSize.x, irSize.y, nStride, nBarrier, vCorners);
        break;
      default:
        break;
      }

      ++nBarrier;

    } while (vCorners.size() > 10000);

    vector<ImageRef> vMaxCorners;
    vector<int> vScores;
    vector<pair<ImageRef,int>> vMaxCornersWithScore;

    switch (fd) {
    case PLAIN_FAST7:
    case PLAIN_FAST8:
    case PLAIN_FAST9:
    case PLAIN_FAST10:
    case PLAIN_FAST11:
    case PLAIN_FAST12:
    case FAST10:
//      fast_nonmax(im, vCorners, nBarrier, vMaxCorners);
      fast_nonmax_with_scores(im, vCorners, nBarrier, vMaxCornersWithScore);
      unzip(vMaxCornersWithScore, vMaxCorners, vScores);
      break;
    case OAST9_16:
      oast9_16_nms_with_scores(pData, nStride, nBarrier, vCorners, vMaxCorners, vScores);
      break;
    case AGAST7_12d:
      agast7_12d_nms_with_scores(pData, nStride, nBarrier, vCorners, vMaxCorners, vScores);
      break;
    case AGAST7_12s:
      agast7_12s_nms_with_scores(pData, nStride, nBarrier, vCorners, vMaxCorners, vScores);
      break;
    case AGAST5_8:
      agast5_8_nms_with_scores(pData, nStride, nBarrier, vCorners, vMaxCorners, vScores);
      break;
    default:
      throw std::runtime_error("Invalid feature detector");
    }

    Toc();

    vector<pair<ImageRef,int>> vInnerMaxCornersWithScore;
    for (size_t i = 0; i < vMaxCorners.size(); ++i) {
      if (im.in_image_with_border(vMaxCorners[i], 10)) {

        if (vScores.size() != vMaxCorners.size()) { // agast detecors returns the scores for every point
          auto it = std::find(vCorners.begin(), vCorners.end(), vMaxCorners[i]);
          assert(it != vCorners.end());
          size_t index = std::distance(vCorners.begin(), it);
          vInnerMaxCornersWithScore.emplace_back(vMaxCorners[i], vScores[index]);
        } else {
          vInnerMaxCornersWithScore.emplace_back(vMaxCorners[i], vScores[i]);
        }
      }
    }

    cout << vInnerMaxCornersWithScore.size() << endl;

    vector<int> vShiTomasiScores(vMaxCorners.size(), 0);

    stringstream ss;
    ss << "fd" << i << ".txt";
    ofstream fs(ss.str(), ios::out | ios::trunc);

    for (size_t i = 0; i < vInnerMaxCornersWithScore.size(); ++i) {
      double dScore = FindShiTomasiScoreAtPoint(im, 3, vInnerMaxCornersWithScore[i].first);
      fs << vInnerMaxCornersWithScore[i].second << " " << dScore << endl;
    }

    fs.close();

  }

  return 0;
}
