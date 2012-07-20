#include "FeatureGrid.h"
#include "ShiTomasi.h"

#include "agast/agast5_8.h"
#include "agast/agast7_12s.h"
#include "agast/agast7_12d.h"
#include "agast/oast9_16.h"

#include <cvd/fast_corner.h>
#include <gvars3/instances.h>

#include <limits>
#include <cmath>
#include <stdexcept>

namespace CVD {
  void fast_corner_detect_plain_7(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
  void fast_corner_detect_plain_8(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
  void fast_corner_detect_plain_9(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
  void fast_corner_detect_plain_10(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
  void fast_corner_detect_plain_11(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
  void fast_corner_detect_plain_12(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
}

using namespace CVD;
using namespace GVars3;

namespace PTAMM {

FeatureGrid::FeatureGrid(size_t nWidth, size_t nHeight, size_t nRows, size_t nCols,
                         int nInitialBarrier)
  : mnRows(nRows)
  , mnCols(nCols)
  , mnMinFeaturesPerCell(2000)
  , mnMaxFeaturesPerCell(3000)
  , mFeatureDetector(PLAIN_FAST10)
{
  int nBorder = 0;

  nWidth -= 2 * nBorder;
  nHeight -= 2 * nBorder;

  mirCellSize = ImageRef(nWidth / nCols, nHeight / nRows);

  size_t nCells = mnCols * mnRows;
  mvCells.resize(nCells);


  for (size_t i = 0; i < mnRows; ++i) {
    for (size_t j = 0; j < mnCols; ++j) {
      size_t idx = i * mnCols + j;
      mvCells[idx].irPosition = ImageRef(j * mirCellSize.x + nBorder, i * mirCellSize.y + nBorder);
      mvCells[idx].nBarrier = nInitialBarrier;
    }
  }
}

void FeatureGrid::DetectFeatures(const CVD::BasicImage<CVD::byte> &im,
                                 const GridCell &cell,
                                 std::vector<ImageRef> &vCorners)
{
  vCorners.clear();

  ImageRef irSize;
  int nStride = im.row_stride();
  int nBarrier = cell.nBarrier;
  const unsigned char *pData = im.data() + cell.irPosition.y * nStride + cell.irPosition.x;

  Image<byte> im2;

  switch (mFeatureDetector) {
  case PLAIN_FAST7:
    irSize = CellSize(im.size(), cell, ImageRef(6, 6));
    fast_corner_detect_plain_7(im.sub_image(cell.irPosition, irSize), vCorners, nBarrier);
    break;
  case PLAIN_FAST8:
    irSize = CellSize(im.size(), cell, ImageRef(6, 6));
    fast_corner_detect_plain_8(im.sub_image(cell.irPosition, irSize), vCorners, nBarrier);
    break;
  case PLAIN_FAST9:
    irSize = CellSize(im.size(), cell, ImageRef(6, 6));
    fast_corner_detect_plain_9(im.sub_image(cell.irPosition, irSize), vCorners, nBarrier);
    break;
  case PLAIN_FAST10:
    irSize = CellSize(im.size(), cell, ImageRef(6, 6));
    fast_corner_detect_plain_10(im.sub_image(cell.irPosition, irSize), vCorners, nBarrier);
    break;
  case PLAIN_FAST11:
    irSize = CellSize(im.size(), cell, ImageRef(6, 6));
    fast_corner_detect_plain_11(im.sub_image(cell.irPosition, irSize), vCorners, nBarrier);
    break;
  case PLAIN_FAST12:
    irSize = CellSize(im.size(), cell, ImageRef(6, 6));
    fast_corner_detect_plain_12(im.sub_image(cell.irPosition, irSize), vCorners, nBarrier);
    break;
  case FAST10:
    irSize = CellSize(im.size(), cell, ImageRef(6, 6));
    im2.copy_from(im.sub_image(cell.irPosition, irSize));
    fast_corner_detect_10(im2, vCorners, cell.nBarrier);
    break;
  case OAST9_16:
    irSize = CellSize(im.size(), cell, ImageRef(8, 6));
    oast9_16(pData, irSize.x, irSize.y, nStride, nBarrier, vCorners);
    break;
  case AGAST7_12d:
    irSize = CellSize(im.size(), cell, ImageRef(8, 6));
    agast7_12d(pData, irSize.x, irSize.y, nStride, nBarrier, vCorners);
    break;
  case AGAST7_12s:
    irSize = CellSize(im.size(), cell, ImageRef(6, 4));
    agast7_12s(pData, irSize.x, irSize.y, nStride, nBarrier, vCorners);
    break;
  case AGAST5_8:
    irSize = CellSize(im.size(), cell, ImageRef(4, 2));
    agast5_8(pData, irSize.x, irSize.y, nStride, nBarrier, vCorners);
    break;
  default:
    throw std::runtime_error("Invalid feature detector");
  }
}

void FeatureGrid::NonMaximumSuppression(const CVD::BasicImage<CVD::byte> &im,
                                        const std::vector<CVD::ImageRef> &vAllFeatures,
                                        int nMinBarrier,
                                        std::vector<CVD::ImageRef> &vMaxFeatures)
{
  vMaxFeatures.clear();

  switch (mFeatureDetector) {
  case PLAIN_FAST7:
  case PLAIN_FAST8:
  case PLAIN_FAST9:
  case PLAIN_FAST10:
  case PLAIN_FAST11:
  case PLAIN_FAST12:
  case FAST10:
    fast_nonmax(im, vAllFeatures, nMinBarrier, vMaxFeatures);
    break;
  case OAST9_16:
    oast9_16_nms(im.data(), im.row_stride(), nMinBarrier, vAllFeatures, vMaxFeatures);
    break;
  case AGAST7_12d:
    agast7_12d_nms(im.data(), im.row_stride(), nMinBarrier, vAllFeatures, vMaxFeatures);
    break;
  case AGAST7_12s:
    agast7_12s_nms(im.data(), im.row_stride(), nMinBarrier, vAllFeatures, vMaxFeatures);
    break;
  case AGAST5_8:
    agast5_8_nms(im.data(), im.row_stride(), nMinBarrier, vAllFeatures, vMaxFeatures);
    break;
  default:
    throw std::runtime_error("Invalid feature detector");
  }
}

void FeatureGrid::SetTargetFeatureCount(size_t nMinFeaturesPerCell, size_t nMaxFeaturesPerCell)
{
  mnMinFeaturesPerCell = nMinFeaturesPerCell;
  mnMaxFeaturesPerCell = nMaxFeaturesPerCell;
}

void FeatureGrid::Clear()
{
  for (size_t i = 0; i < mvCells.size(); ++i) {
    mvCells[i].vFeatures.clear();
    mvCells[i].vBestFeatures.clear();
  }
}

void FeatureGrid::FindFeatures(const CVD::BasicImage<CVD::byte> &im)
{
  for (size_t i = 0; i < mvCells.size(); ++i) {
    GridCell& cell = mvCells[i];
    cell.vFeatures.clear();

    std::vector<ImageRef> vTmpCorners;
    DetectFeatures(im, cell, vTmpCorners);

    for (auto it = vTmpCorners.begin(); it != vTmpCorners.end(); ++it) {
      cell.vFeatures.push_back(cell.irPosition + *it);
    }

    size_t nFoundCorners = vTmpCorners.size();

    if (nFoundCorners > mnMaxFeaturesPerCell) {
      cell.nBarrier = std::min(cell.nBarrier + 1, 100);
    } else if (nFoundCorners < mnMinFeaturesPerCell) {
      cell.nBarrier = std::max(cell.nBarrier - 1, 5);
    }
  }
}

void FeatureGrid::FindBestFeatures(const CVD::BasicImage<CVD::byte> &im)
{
  static gvar3<int> gvnShiTomasiThreshold("Tracker.ShiTomasiThreshold", 50, SILENT);

  for (size_t i = 0; i < mvCells.size(); ++i) {
    mvCells[i].vBestFeatures.clear();
  }

  std::vector<ImageRef> vMaxFeatures;
  GetNonMaxSuppressed(im, vMaxFeatures);

  int nShiTomasiThreshold = *gvnShiTomasiThreshold;

  for (size_t i = 0; i < vMaxFeatures.size(); ++i) {
    if (im.in_image_with_border(vMaxFeatures[i], 5)) {
      int idx = CellIndex(vMaxFeatures[i]);
      assert(idx >= 0);
      double dScore = FindShiTomasiScoreAtPoint(im, 3, vMaxFeatures[i]);

      if (dScore > nShiTomasiThreshold)
      { // Just a small threshold to not add really bad points
        mvCells[idx].vBestFeatures.insert(ScoredPoint(vMaxFeatures[i], dScore));
      }
    }
  }
}

void FeatureGrid::GetFeaturesInsideCircle(const CVD::ImageRef &irCenter, int nRadius, std::vector<CVD::ImageRef> &vFeatures) const
{
  int sqRadius = nRadius * nRadius;

  int colStart = std::max(0, (irCenter.x - nRadius) / mirCellSize.x);
  int rowStart = std::max(0, (irCenter.y - nRadius) / mirCellSize.y);
  int colEnd = std::min((int)mnCols, (int)std::ceil((double)(irCenter.x + nRadius) / mirCellSize.x));
  int rowEnd = std::min((int)mnRows, (int)std::ceil((double)(irCenter.y + nRadius) / mirCellSize.y));

  for (int i = rowStart; i < rowEnd; ++i) {
    for (int j = colStart; j < colEnd; ++j) {
      size_t idx = i * mnCols + j;
      const GridCell &cell = mvCells[idx];

      for (auto it = cell.vFeatures.begin(); it != cell.vFeatures.end(); ++it) {
        int dx = it->x - irCenter.x;
        int dy = it->y - irCenter.y;

        int sqDist = dx * dx + dy * dy;

        if (sqDist <= sqRadius) {
          vFeatures.push_back(*it);
        }
      }
    }
  }
}

void FeatureGrid::GetAllFeatures(std::vector<CVD::ImageRef> &vFeatures) const
{
  vFeatures.clear();
  for (size_t i = 0; i < mvCells.size(); ++i) {
    const std::vector<ImageRef>& fts = mvCells[i].vFeatures;
    for (auto it = fts.begin(); it != fts.end(); ++it) {
      vFeatures.push_back(*it);
    }
  }
}

void FeatureGrid::GetAllFeaturesSorted(std::vector<CVD::ImageRef> &vFeatures) const
{
  vFeatures.clear();

  int currentRow = 0;

  for (size_t i = 0; i < mnRows; ++i) {
    std::vector<size_t> vOffsets(mnCols, 0);
    int nRemainingCols = 0;

    do {
      nRemainingCols = mnCols;

      for (size_t j = 0; j < mnCols; ++j) {
        const GridCell& cell = mvCells[i * mnCols + j];
        const std::vector<ImageRef>& fts = cell.vFeatures;
        size_t& offset = vOffsets[j];

        while (offset < fts.size() && fts[offset].y == currentRow) {
          vFeatures.push_back(fts[offset]);
          ++offset;
        }

        if (offset == fts.size()) {
          nRemainingCols--;
        }
      }

      ++currentRow;
    } while (nRemainingCols > 0);
  }
}

void FeatureGrid::GetBestFeatures(size_t nNumFeatures, std::vector<CVD::ImageRef> &vFeatures) const
{
  vFeatures.clear();
  size_t nFeaturesPerCell = nNumFeatures / mvCells.size();
  // Select the best N/num_cells features per cell
  for (size_t i = 0; i < mvCells.size(); ++i) {
    const std::multiset<ScoredPoint>& fts = mvCells[i].vBestFeatures;
    if (fts.size() > nFeaturesPerCell) {
      auto it = fts.begin();
      for (size_t i = 0; i < nFeaturesPerCell; ++i) {
        vFeatures.push_back(it->irPoint);
        ++it;
      }
    } else {
      for (auto it = fts.begin(); it != fts.end(); ++it) {
        vFeatures.push_back(it->irPoint);
      }
    }
  }
}

ImageRef FeatureGrid::CellSize(const ImageRef &irImSize, const GridCell &cell, const ImageRef &irMargin) const
{
  ImageRef irSize = mirCellSize + irMargin;

  ImageRef irBottomRight = cell.irPosition + irSize;
  if (irBottomRight.x > irImSize.x) {
    irSize.x -= irBottomRight.x - irImSize.x;
  }
  if (irBottomRight.y > irImSize.y) {
    irSize.y -= irBottomRight.y - irImSize.y;
  }

  return irSize;
}

int FeatureGrid::CellIndex(const CVD::ImageRef &irPoint) const
{
  int col = irPoint.x / mirCellSize.x;
  int row = irPoint.y / mirCellSize.y;

  if (col < 0 || (size_t)col >= mnCols || row < 0 || (size_t)row >= mnRows) {
    return -1;
  }

  return row * mnCols + col;
}

int FeatureGrid::MinBarrier() const
{
  int nMinBarrier = std::numeric_limits<int>::max();
  for (size_t i = 0; i < mvCells.size(); ++i) {
    nMinBarrier = std::min(nMinBarrier, mvCells[i].nBarrier);
  }
  return nMinBarrier;
}

void FeatureGrid::GetNonMaxSuppressed(const CVD::BasicImage<CVD::byte> &im, std::vector<CVD::ImageRef> &vMaxFeatures)
{
  std::vector<ImageRef> vSortedFeatures;
  GetAllFeaturesSorted(vSortedFeatures);
  int nMinBarrier = MinBarrier();
  NonMaximumSuppression(im, vSortedFeatures, nMinBarrier, vMaxFeatures);
}

}
