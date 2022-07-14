#ifndef SORT_ASSIGNMENT_H_
#define SORT_ASSIGNMENT_H_ 

#include <vector>

namespace SORT {

struct Bbox;

float IOU(const Bbox& b1, const Bbox& b2);

std::vector<std::vector<float>> ComputeIOUs(const std::vector<Bbox>& detections, 
                                            const std::vector<Bbox>& tracks);

struct MatchResult{
  std::vector<std::pair<int, int>> matches;
  std::vector<int> unmatched_detections;
  std::vector<int> unmatched_tracks;
};

bool IsSimpleAssignment(const std::vector<std::vector<float>>& ious, 
                        float iou_threshold, 
                        std::vector<std::pair<int, int>>& matched_indices);

MatchResult LinearAssignment(const std::vector<Bbox>& detections, 
                             const std::vector<Bbox>& tracks, 
                             float iou_threshold=0.3);

}  // namespace SORT

#endif  // SORT_ASSIGNMENT_H_