#include <glog/logging.h>

#include "kalman_filter.h"

namespace SORT {

KalmanFilter::KalmanFilter() {
  x_ = Vector7f::Zero();
  LOG(INFO) << "x_:\n" << x_;
  P_ = Matrix7f::Identity();
  P_.block<3, 3>(4, 4) *= 1000; // give high uncertainty to the unobservable initial velocities
  P_ *= 10;
  LOG(INFO) << "P_:\n" << P_;

  F_ << 1, 0, 0, 0, 1, 0, 0,
        0, 1, 0, 0, 0, 1, 0,
        0, 0, 1, 0, 0, 0, 1,
        0, 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 1;
  LOG(INFO) << "F_:\n" << F_;
  Q_ = Matrix7f::Identity();
  Q_(6, 6) *= 0.01;
  Q_.block<3, 3>(4, 4) *= 0.01;
  LOG(INFO) << "Q_:\n" << Q_;
  
  H_ << 1, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 1, 0, 0, 0;
  LOG(INFO) << "H_:\n" << H_;
  R_ = Matrix4f::Identity();
  R_.block<2, 2>(2, 2) *= 10;
  LOG(INFO) << "R_:\n" << R_;

  I7_ = Matrix7f::Identity();
}

void KalmanFilter::Predict() {
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose()+Q_;
}

void KalmanFilter::Update(Vector4f z) {
  // y = z - Hx
  Vector4f y = z-H_*x_;
  // common subexpression for accelerating
  Matrix74f PHT = P_*H_.transpose();
  // S = HPH' + R
  Matrix4f S = H_*PHT+R_;
  // K = PH'inv(S)
  Matrix74f K = PHT*S.inverse();
  // x = x + Ky
  x_ = x_ + K*y;
  // P = (I-KH)P(I-KH)' + KRK'
  // This is more numerically stable
  // and works for non-optimal K vs the equation
  // P = (I-KH)P usually seen in the literature.
  Matrix7f I_KH = I7_-K*H_;
  P_ = I_KH*P_*I_KH.transpose()+K*R_*K.transpose();
}


}  // namespace SORT