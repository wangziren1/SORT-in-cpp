#include "linear_assignment.h"

#include "hungarian.h"
#include "sort.h"

namespace SORT {

float IOU(const Bbox& b1, const Bbox& b2) {
  float area1 = (b1.x2 - b1.x1) * (b1.y2 - b1.y1);
  float area2 = (b2.x2 - b2.x1) * (b2.y2 - b2.y1);
  float x1 = std::max(b1.x1, b2.x1);
  float y1 = std::max(b1.y1, b2.y1);
  float x2 = std::min(b1.x2, b2.x2);
  float y2 = std::min(b1.y2, b2.y2);
  float w = std::max(0.0f, x2 - x1);
  float h = std::max(0.0f, y2 - y1);
  return w * h / (area1 + area2 - w * h);
}

std::vector<std::vector<float>> ComputeIOUs(const std::vector<Bbox>& detections, 
                                            const std::vector<Bbox>& tracks) {
  std::vector<std::vector<float>> ious(detections.size(), 
                                       std::vector<float>(tracks.size(), 0));
  for (int i = 0; i < detections.size(); ++i)
    for (int j = 0; j < tracks.size(); ++j)
      ious[i][j] = IOU(detections[i], tracks[j]);
  return ious;
}

bool IsSimpleAssignment(const std::vector<std::vector<float>>& ious, 
                        float iou_threshold, 
                        std::vector<std::pair<int, int>>& matched_indices) {
  int max = -1;
  for (int i = 0; i < ious.size(); ++i) {
    int sum = 0;
    for (int j = 0; j < ious[0].size(); ++j) {
      if (ious[i][j] > iou_threshold) {
        matched_indices.push_back(std::make_pair(i, j));
        sum++;
      }
    }
    if (sum > max)
      max = sum;
  }
  if (max == 1) {
    int max = -1;
    for (int j = 0; j < ious[0].size(); ++j) {
      int sum = 0;
      for (int i = 0; i < ious.size(); ++i) {
        if (ious[i][j] > iou_threshold) 
          sum++;
      }
      if (sum > max)
        max = sum;
    }
    if (max == 1)
      return true;
  }
  return false;
}

inline std::vector<std::vector<float>> NegativeIOUs(
    const std::vector<std::vector<float>>& ious) {
  std::vector<std::vector<float>> negative_ious(
      ious.size(), std::vector<float>(ious[0].size(), 0));
  for (int i = 0; i < ious.size(); ++i)
    for (int j = 0; j < ious[0].size(); ++j)
      negative_ious[i][j] = 1-ious[i][j];
  return negative_ious;
}

MatchResult LinearAssignment(const std::vector<Bbox>& detections, 
                         const std::vector<Bbox>& tracks,
                         float iou_threshold) {
  MatchResult match_result;
  if (tracks.empty()) {
    for (int i = 0; i < detections.size(); ++i)
      match_result.unmatched_detections.push_back(i);
    return match_result;
  }

  std::vector<std::vector<float>> ious = ComputeIOUs(detections, tracks);
  std::vector<std::pair<int, int>> matched_indices;
  if (!ious.empty()) {
    std::vector<std::pair<int, int>> matched_indices_temp;
    if (IsSimpleAssignment(ious, iou_threshold, matched_indices_temp))
      matched_indices = matched_indices_temp;
    else {
      HungarianAlgorithm hungarian;
      std::vector<int> assignment;
      hungarian.Solve(NegativeIOUs(ious), assignment);
      for (int i = 0; i < assignment.size(); ++i) {
        // Filter out matched with low IOU
        if (assignment[i] != -1 && ious[i][assignment[i]] >= iou_threshold)
        matched_indices.push_back(std::make_pair(i, assignment[i]));
      }
    }
  }

  std::vector<int> unmatched_detections;
  for (int i = 0; i < detections.size(); ++i) {
    bool is_match = false;
    for (const auto& match : matched_indices) {
      if (i == match.first) {
        is_match = true;
        break;
      }
    }
    if (!is_match)
      unmatched_detections.push_back(i);
  }
  std::vector<int> unmatched_tracks;
  for(int i = 0; i < tracks.size(); ++i) {
    bool is_match = false;
    for (const auto& match : matched_indices) {
      if (i == match.second) {
        is_match = true;
        break;
      }
    }
    if (!is_match)
      unmatched_tracks.push_back(i);
  }

  match_result.matches = matched_indices;
  match_result.unmatched_detections = unmatched_detections;
  match_result.unmatched_tracks = unmatched_tracks;
  return match_result;
}

}  // namespace SORT