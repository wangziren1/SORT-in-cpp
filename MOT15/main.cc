#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include "sort.h"

using namespace SORT;
using namespace std;

vector<string> split(const string& s, char delim) {
    vector<string> result;
    istringstream ss(s);
    string item;

    while (getline(ss, item, delim)) {
        result.push_back (item);
    }
    return result;
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);

  std::string file_path("../data/train/ADL-Rundle-6/det/det.txt");
  std::fstream f(file_path);
  if (!f.is_open()) {
    std::cout << "Can't open " << file_path << std::endl;
    return -1;
  }
  vector<string> lines;
  std::string line;
  while (getline(f, line)) {
    lines.push_back(line);
  }
  int frames = stoi(split(lines.back(), ',')[0]);
  cout << "total frames: " << frames << std::endl;

  vector<vector<Bbox>> data(frames);
  for (auto line : lines) {
    vector<string> words = split(line, ',');
    int frame_id = stoi(words[0]);
    float x1 = stof(words[2]);
    float y1 = stof(words[3]);
    float w = stof(words[4]);
    float h = stof(words[5]);
    // cout << frame_id << " " << x1 << " " << y1 << " " << w << " " << h << std::endl;
    float x2 = x1 + w;
    float y2 = y2 + h;
    data[frame_id-1].push_back(Bbox(x1, y1, x2, y2));
  }
  
  cout << "start" << std::endl;
  Sort mot_tracker;
  int frame = 0;
  for (int i = 0; i < data.size(); ++i) {
    frame = i + 1;
    // cout << frame << std::endl;
    vector<BboxID> bboxids = mot_tracker.Update(data[i]);
    string img_path("~/data/MOT15/train/ADL-Rundle-6/img1/");
    // cv::Mat img = cv::imread(img_path);
    // cv::imshow("frame", img);
    // cv::waitKey(1);
  }

}