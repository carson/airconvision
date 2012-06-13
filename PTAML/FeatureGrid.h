#ifndef __FEATURE_GRID_H
#define __FEATURE_GRID_H

#include <cvd/image_ref.h>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <gvars3/instances.h>

#include <set>
#include <vector>

namespace PTAMM {

enum FeatureDetector {
  PLAIN_FAST10,
  FAST10,
  OAST9_16,
  AGAST7_12d,
  AGAST7_12s,
  AGAST5_8
};

class FeatureGrid {
  public:
    FeatureGrid(size_t nWidth, size_t nHeight, size_t nRows, size_t nCols, int nInitialBarrier = 15);

    void SetTargetFeatureCount(size_t nMinFeaturesPerCell, size_t nMaxFeaturesPerCell);

    void Clear();

    void FindFeatures(const CVD::BasicImage<CVD::byte> &im);
    void GetAllFeatures(std::vector<CVD::ImageRef> &vFeatures) const;
    void GetFeaturesInsideCircle(const CVD::ImageRef &irCenter, int nRadius, std::vector<CVD::ImageRef> &vFeatures) const;

    void FindBestFeatures(const CVD::BasicImage<CVD::byte> &im);
    void GetBestFeatures(size_t nNumFeatures, std::vector<CVD::ImageRef> &vFeatures) const;

    size_t Rows() const { return mnRows; }
    size_t Cols() const { return mnCols; }

  private:
    struct ScoredPoint {
      ScoredPoint() {}
      ScoredPoint(const CVD::ImageRef &irPoint, double dScore)
       : irPoint(irPoint), dScore(dScore) {}

      bool operator<(const ScoredPoint &rhs) const
      {
        return dScore > rhs.dScore; // Sort descending
      }

      CVD::ImageRef irPoint;
      double dScore;
    };

    struct GridCell {
      CVD::ImageRef irPosition;
      int nBarrier;
      std::vector<CVD::ImageRef> vFeatures;
      std::multiset<ScoredPoint> vBestFeatures;
    };

    void DetectFeatures(const CVD::BasicImage<CVD::byte> &im,
                        const GridCell &cell,
                        std::vector<CVD::ImageRef> &vCorners);

    void NonMaximumSuppression(const CVD::BasicImage<CVD::byte> &im,
                               const std::vector<CVD::ImageRef> &vAllFeatures,
                               int nMinBarrier,
                               std::vector<CVD::ImageRef> &vMaxFeatures);

    void GetAllFeaturesSorted(std::vector<CVD::ImageRef> &vFeatures) const;
    void GetNonMaxSuppressed(const CVD::BasicImage<CVD::byte> &im, std::vector<CVD::ImageRef> &vFeatures);

    int GetCellIndex(const CVD::ImageRef &irPoint) const;
    int GetMinBarrier() const;

  private:
    size_t mnRows;
    size_t mnCols;
    CVD::ImageRef mirCellSize;
    std::vector<GridCell> mvCells;
    size_t mnMinFeaturesPerCell;
    size_t mnMaxFeaturesPerCell;

    GVars3::gvar3<int> mgvnFeatureDectecor;
};

}

#endif
