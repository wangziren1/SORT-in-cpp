#include <glog/logging.h>

#include "kalman_box_tracker.h"
#include "sort.h"

using namespace SORT;

int main(int argc, char* argv[]) {
  // Initialize Google’s logging library.
  google::InitGoogleLogging(argv[0]);
  
  Bbox bbox(10, 10, 20, 20);
  KalmanBoxTracker kalman_box_tracker(bbox);
  LOG(INFO) << kalman_box_tracker.state();

  kalman_box_tracker.Predict();
  LOG(INFO) << kalman_box_tracker.state();
  
  kalman_box_tracker.Update(Bbox(14, 16, 26, 23));
  LOG(INFO) << kalman_box_tracker.state();
  
  kalman_box_tracker.Predict();
  LOG(INFO) << kalman_box_tracker.state();
  
  kalman_box_tracker.Update(Bbox(21, 19, 31, 29.));
  LOG(INFO) << kalman_box_tracker.state();
}