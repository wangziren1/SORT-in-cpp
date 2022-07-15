#ifndef SORT_KALMAN_BOX_TRACKER_H_
#define SORT_KALMAN_BOX_TRACKER_H_

#include <iostream>
#include <vector>

#include "kalman_filter.h"

namespace SORT{

struct Bbox;

class KalmanBoxTracker{
 public:
  KalmanBoxTracker(const Bbox& bbox);
  Bbox Predict();
  void Update(Bbox bbox);
  Bbox state();
  int predict_times_since_update() { return predict_times_since_update_; }
  int hits_continuous() { return hits_continuous_; } 
  int id() { return id_; }

 private:
  static int count;
  KalmanFilter kalman_filter_;
  int predict_times_since_update_;
  int id_;
  int age_;
  int hits_;
  int hits_continuous_;
  std::vector<Bbox> history_;
};

}

#endif  // SORT_KALMAN_BOX_TRACKER_H_