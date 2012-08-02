#ifndef __STEREO_PLANE_FINDER_H
#define __STEREO_PLANE_FINDER_H

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/LU.h>
#include <cvd/image.h>

namespace PTAMM {

using namespace TooN;

template <int StateSize, int ObsSize, int CtrlSize>
class KalmanFilter {
  public:
    KalmanFilter()
    {
    }

    KalmanFilter(const Matrix<StateSize>& A, const Matrix<StateSize, CtrlSize>& B,
                 const Matrix<StateSize>& R, const Matrix<ObsSize, StateSize>& C,
                 const Matrix<StateSize>& Q)
      : A_(A), B_(B), R_(R), C_(C), Q_(Q)
    {
    }

    void init(const TooN::Vector<StateSize>& mu, const Matrix<StateSize>& sigma)
    {
      mu_ = mu;
      sigma_ = sigma;
    }

    void predict(const TooN::Vector<CtrlSize>& ctrl)
    {
      muBar_ = A_ * mu_ + B_ * ctrl;
      sigmaBar_ = A_ * sigma_ * A_.T() + R_;
    }

    void observe(const TooN::Vector<ObsSize>& obs)
    {
      TooN::LU<StateSize> lu(C_ * sigmaBar_ * C_.T() + Q_);
      Matrix<> K = sigmaBar_ * C_.T() * lu.get_inverse();
      mu_ = muBar_ + K * (obs - C_ * muBar_);
      sigma_ = (Identity - K * C_) * sigmaBar_;
    }

    const TooN::Vector<StateSize>& mu() const { return mu_; }

  private:
    Matrix<StateSize> A_;
    Matrix<StateSize, CtrlSize> B_;
    Matrix<StateSize> R_;
    Matrix<ObsSize, StateSize> C_;
    Matrix<StateSize> Q_;

    TooN::Vector<StateSize> mu_;
    TooN::Vector<StateSize> muBar_;
    Matrix<StateSize> sigma_;
    Matrix<StateSize> sigmaBar_;
};

class StereoPlaneFinder {
  public:
    StereoPlaneFinder();

    void Update(const std::vector<TooN::Vector<3> >& pointCloud);
    void Update(const CVD::SubImage<float> &imDisp, double f, double L);
    const TooN::Vector<4>& GetPlane() const;

  private:
    void UpdateFilter(const TooN::Vector<4>& v4Plane);

  private:
    bool mbFirstUpdate;
    KalmanFilter<4, 4, 1> mFilter;

    TooN::Vector<4> mv4Plane;
};

}

#endif
