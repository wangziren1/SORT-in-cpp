#include <cmath>

#include <glog/logging.h>

#include "kalman_box_tracker.h"

namespace SORT{

std::ostream& operator<<(std::ostream& os, const Bbox& bbox) {
  os << "Bbox:" << bbox.x1 << " " << bbox.y1 << " " 
     << bbox.x2 << " " << bbox.y2;
  return os;
}

Vector4f Bbox2Z(const Bbox& bbox) {
  float w = bbox.x2 - bbox.x1;
  float h = bbox.y2 - bbox.y1;
  float x = bbox.x1 + w / 2.;
  float y = bbox.y1 + h / 2.;
  float s = w * h;
  float r = w / h;
  return Vector4f(x, y, s, r); 
}

Bbox X2Bbox(const Vector7f& x) {
  float w = sqrt(x[2]*x[3]);
  float h = x[2]/w;
  Bbox bbox(x[0]-w/2, x[1]-h/2, x[0]+w/2, x[1]+h/2);
  return bbox;
}

int KalmanBoxTracker::count = 0;

KalmanBoxTracker::KalmanBoxTracker(const Bbox& bbox) 
  : predict_times_since_update_(0), 
    id_(count),
    age_(0),
    hits_(0),
    hits_continuous_(0) {
  count++;
  kalman_filter_.state().head<4>() = Bbox2Z(bbox);
}

void KalmanBoxTracker::Predict() {
  Vector7f& state = kalman_filter_.state();
  if (state[6] + state[2] <= 0)
    state[6] *= 0;
  kalman_filter_.Predict();
  age_++;
  if (predict_times_since_update_ > 0)
    hits_continuous_ = 0;
  predict_times_since_update_++;
}

void KalmanBoxTracker::Update(Bbox bbox) {
  predict_times_since_update_ = 0;
  hits_++;
  hits_continuous_++;
  kalman_filter_.Update(Bbox2Z(bbox));
  history_.push_back(state());
}

Bbox KalmanBoxTracker::state() {
  return X2Bbox(kalman_filter_.state());
}

}  // namespace SORT