#include "sort.h"

#include "kalman_box_tracker.h"
#include "linear_assignment.h"

namespace SORT {

std::ostream& operator<<(std::ostream& os, const Bbox& bbox) {
  os << "Bbox:" << bbox.x1 << " " << bbox.y1 << " " 
     << bbox.x2 << " " << bbox.y2;
  return os;
}

Sort::Sort(int max_age_for_lost, int min_hits, float iou_threshold)
  : max_age_for_lost_(max_age_for_lost), min_hits_(min_hits), 
    iou_threshold_(iou_threshold), frame_num_(0) {
}

std::vector<BboxID> Sort::Update(const std::vector<Bbox>& detections) {
  frame_num_++;
  // get predicted locations from existing trackers.
  std::vector<Bbox> predictions;
  for (auto& tracker : trackers_) {
    predictions.push_back(tracker->Predict());
  }
  // associate detections to trackers
  MatchResult match_result = LinearAssignment(detections, predictions, 
                                              iou_threshold_);
  // update matched trackers with assigned detections
  for (const auto& match : match_result.matches) {
    trackers_[match.second]->Update(detections[match.first]);
  }
  // create and initialise new trackers for unmatched detections
  for (const auto& unmatched_detection : match_result.unmatched_detections) {
    trackers_.push_back(std::make_shared<KalmanBoxTracker>(
        detections[unmatched_detection]));
  }
  
  std::vector<BboxID> sort_results;
  for (int i = 0; i < trackers_.size();) {
    auto tracker = trackers_[i];
    if (tracker->predict_times_since_update() < 1 && 
        (tracker->hits_continuous() >= min_hits_ ||
         frame_num_ <= min_hits_)) {
      // id +1 as MOT benchmark requires positive
      sort_results.push_back(BboxID{tracker->state(), tracker->id()+1});
    }
    // remove dead tracklet
    if (trackers_[i]->predict_times_since_update() > max_age_for_lost_) {
      auto temp = trackers_[i];
      trackers_[i] = trackers_.back();
      trackers_.back() = temp;
      trackers_.pop_back();
    } else {
      i++;
    }
  }
  return sort_results;
}

}  // namespace SORT