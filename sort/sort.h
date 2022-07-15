#ifndef SORT_SORT_H_
#define SORT_SORT_H_

#include <iostream>
#include <memory>
#include <vector>

namespace SORT {

class KalmanBoxTracker;

struct Bbox {
  Bbox(float x1, float y1, float x2, float y2) 
    : x1(x1), y1(y1), x2(x2), y2(y2) {} 
  // top left corner(x1, y1) and bottom right corner(x2, y2) 
  float x1; 
  float y1;
  float x2;
  float y2;
};
std::ostream& operator<<(std::ostream& os, const Bbox& bbox);

struct BboxID {
  Bbox bbox;
  int id;
};

class Sort {
 public:
  Sort(int max_lost_age=1, int min_hits=3, float iou_threshold=0.3);
  std::vector<BboxID> Update(const std::vector<Bbox>& detections);

 private:
  // if the tracker has track lost for more than max_age_for_lost_, 
  // delete the tracker.
  int max_age_for_lost_;
  // if the tracker has updated for min_hits_, the tracker's state is reliable.
  int min_hits_; 
  float iou_threshold_;
  std::vector<std::shared_ptr<KalmanBoxTracker>> trackers_;
  int frame_num_;
};

}  // namespace SORT

#endif  // SORT_SORT_H_