#include "FeatureGrid.h"
#include "ShiTomasi.h"

#include <cvd/fast_corner.h>

#include <limits>
#include <cmath>

namespace CVD
{
void fast_corner_detect_plain_10(const SubImage<byte>& i, std::vector<ImageRef>& corners, int b);
}

using namespace CVD;

namespace PTAMM {

FeatureGrid::FeatureGrid(size_t nWidth, size_t nHeight, size_t nRows, size_t nCols,
                         size_t nMinFeaturesPerCell, size_t nMaxFeaturesPerCell, int nInitialBarrier)
  : mnRows(nRows)
  , mnCols(nCols)
{
  mirCellSize = ImageRef(nWidth / nCols, nHeight / nRows);

  size_t nCells = mnCols * mnRows;
  mvCells.resize(nCells);

  mnMinFeaturesPerCell = nMinFeaturesPerCell;
  mnMaxFeaturesPerCell = nMaxFeaturesPerCell;

  for (size_t i = 0; i < mnRows; ++i) {
    for (size_t j = 0; j < mnCols; ++j) {
      size_t idx = i * mnCols + j;
      mvCells[idx].irPosition = ImageRef(j * mirCellSize.x, i * mirCellSize.y);
      mvCells[idx].nBarrier = nInitialBarrier;
    }
  }
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

    ImageRef irSize = mirCellSize + ImageRef(6, 6);

    ImageRef irBottomRight = cell.irPosition + irSize;
    if (irBottomRight.x > im.size().x - 3) {
      irSize.x -= irBottomRight.x - (im.size().x - 3);
    }
    if (irBottomRight.y > im.size().y - 3) {
      irSize.y -= irBottomRight.y - (im.size().y - 3);
    }

    std::vector<ImageRef> vTmpCorners;

#if 1
    CVD::fast_corner_detect_plain_10(im.sub_image(cell.irPosition, irSize), vTmpCorners, cell.nBarrier);
#else
    Image<byte> im2;
    im2.copy_from(im.sub_image(cell.irPosition, irSize));
    fast_corner_detect_10(im2, vTmpCorners, cell.nBarrier);
#endif

    for (auto it = vTmpCorners.begin(); it != vTmpCorners.end(); ++it) {
      cell.vFeatures.push_back(cell.irPosition + *it);
    }

    size_t nFoundCorners = vTmpCorners.size();

    if (nFoundCorners > mnMaxFeaturesPerCell) {
      cell.nBarrier = std::min(cell.nBarrier + 1, 100);
    } else if (nFoundCorners < mnMinFeaturesPerCell) {
      cell.nBarrier = std::max(cell.nBarrier - 1, 10);
    }
  }
}

void FeatureGrid::FindBestFeatures(const CVD::BasicImage<CVD::byte> &im)
{
  for (size_t i = 0; i < mvCells.size(); ++i) {
    mvCells[i].vBestFeatures.clear();
  }

  std::vector<ImageRef> vMaxFeatures;
  GetNonMaxSuppressed(im, vMaxFeatures);

  for (size_t i = 0; i < vMaxFeatures.size(); ++i) {
    if (im.in_image_with_border(vMaxFeatures[i], 5)) {
      int idx = GetCellIndex(vMaxFeatures[i]);
      assert(idx >= 0);
      double dScore = FindShiTomasiScoreAtPoint(im, 3, vMaxFeatures[i]);

      if (dScore > 50) { // Just a small threshold to not add really bad points
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

int FeatureGrid::GetCellIndex(const CVD::ImageRef &irPoint) const
{
  int col = irPoint.x / mirCellSize.x;
  int row = irPoint.y / mirCellSize.y;

  if (col < 0 || (size_t)col >= mnCols || row < 0 || (size_t)row >= mnRows) {
    return -1;
  }

  return row * mnCols + col;
}

int FeatureGrid::GetMinBarrier() const
{
  int nMinBarrier = std::numeric_limits<int>::max();
  for (size_t i = 0; i < mvCells.size(); ++i) {
    nMinBarrier = std::min(nMinBarrier, mvCells[i].nBarrier);
  }
  return nMinBarrier;
}

void FeatureGrid::GetNonMaxSuppressed(const CVD::BasicImage<CVD::byte> &im, std::vector<CVD::ImageRef> &vMaxFeatures) const
{
  std::vector<ImageRef> vSortedFeatures;
  GetAllFeaturesSorted(vSortedFeatures);

  int nMinBarrier = GetMinBarrier();
  fast_nonmax(im, vSortedFeatures, nMinBarrier, vMaxFeatures);
}

}
