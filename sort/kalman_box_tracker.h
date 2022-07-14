#ifndef SORT_KALMAN_BOX_TRACKER_H_
#define SORT_KALMAN_BOX_TRACKER_H_

#include <iostream>
#include <vector>

#include "kalman_filter.h"

namespace SORT{

struct Bbox {
  Bbox(float x1, float y1, float x2, float y2) 
    : x1(x1), y1(y1), x2(x2), y2(y2) {} 
  
  float x1;
  float y1;
  float x2;
  float y2;
};
std::ostream& operator<<(std::ostream& os, const Bbox& bbox);

class KalmanBoxTracker{
 public:
  KalmanBoxTracker(const Bbox& bbox);
  void Predict();
  void Update(Bbox bbox);
  Bbox state();

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