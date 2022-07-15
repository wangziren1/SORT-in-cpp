#include <gtest/gtest.h>

#include <iostream>

#include "sort.h"
#include "linear_assignment.h"

using namespace SORT;

TEST(linear_assignment, iou_test) {
  Bbox b1(10, 10, 20, 20);
  Bbox b2(15, 15, 25, 25);
  float iou = IOU(b1, b2);
  EXPECT_NEAR(0.142857, iou, 0.00001);

  Bbox b3(20, 20, 30, 30);
  iou = IOU(b1, b3);
  EXPECT_EQ(0, iou);
}

TEST(linear_assignment, IsSimpleAssignment_test) {
  std::vector<std::vector<float>> ious{{1, 0, 0, 0},
                                       {0, 1, 0, 0},
                                       {0, 0, 1, 0}};
  std::vector<std::pair<int, int>> matched_indices;                        
  bool is_simple_assignment = IsSimpleAssignment(ious, 0.3, matched_indices);
  EXPECT_EQ(true, is_simple_assignment);
  std::cout << matched_indices[0].first << " " << matched_indices[0].second << std::endl
            << matched_indices[1].first << " " << matched_indices[1].second << std::endl
            << matched_indices[2].first << " " << matched_indices[2].second << std::endl;

  ious = {{1, 0, 0, 0},
          {0, 1, 0, 0},
          {0, 0, 1, 1}};
  is_simple_assignment = IsSimpleAssignment(ious, 0.3, matched_indices);
  EXPECT_EQ(false, is_simple_assignment);

  ious = {{1, 0, 0, 0},
          {0, 1, 0, 0},
          {1, 0, 1, 0}};
  is_simple_assignment = IsSimpleAssignment(ious, 0.3, matched_indices);
  EXPECT_EQ(false, is_simple_assignment);
}

TEST(linear_assignment, LinearAssignment_test) {
  std::vector<Bbox> detections{Bbox(95, 95, 119, 122), Bbox(105, 105, 125, 125)};
  std::vector<Bbox> tracks{Bbox(100, 100, 120, 120)};
  float iou0 = IOU(detections[0], tracks[0]);
  std::cout << iou0 << std::endl;
  float iou1 = IOU(detections[1], tracks[0]);
  std::cout << iou1 << std::endl;
  MatchResult match_result = LinearAssignment(detections, tracks);
  EXPECT_EQ(match_result.matches.size(), 1);
  EXPECT_EQ(match_result.matches[0].first, 0);
  EXPECT_EQ(match_result.matches[0].second, 0);
  EXPECT_EQ(match_result.unmatched_detections.size(), 1);
  EXPECT_EQ(match_result.unmatched_detections[0], 1);
  EXPECT_EQ(match_result.unmatched_tracks.size(), 0);
}