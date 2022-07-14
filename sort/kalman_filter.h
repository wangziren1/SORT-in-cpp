#ifndef SORT_KALMAN_FILTER_H_
#define SORT_KALMAN_FILTER_H_

#include <Eigen/Dense>

namespace SORT {

using Vector7f = Eigen::Matrix<float, 7, 1>;
using Matrix7f = Eigen::Matrix<float, 7, 7>;
using Matrix47f = Eigen::Matrix<float, 4, 7>;
using Matrix74f = Eigen::Matrix<float, 7, 4>;
using Matrix4f = Eigen::Matrix<float, 4, 4>;
using Vector4f = Eigen::Matrix<float, 4, 1>;

class KalmanFilter {
 public:
  KalmanFilter();

  void Predict();
  
  void Update(Vector4f z);

  Vector7f& state() { return x_; }

 private:
  Vector7f x_; // Current state estimate
  Matrix7f P_; // Current state covariance matrix

  Matrix7f F_; // State Transition matrix
  Matrix7f Q_; // Process noise matrix
  
  Matrix47f H_; // Measurement function
  Matrix4f R_; // Measurement noise matrix

  Matrix7f I7_; // 7*7 identity matrix
};

}  // namespace SORT

#endif  // SORT_KALMAN_FILTER_H_