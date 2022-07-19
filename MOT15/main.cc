#include <chrono>
#include <iostream>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>

#include "sort.h"

using namespace SORT;
using namespace std;

static vector<cv::Scalar> GenerateColours() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> distrib(0, 255);
  vector<cv::Scalar> colours;
  for (int i = 0; i < 32; ++i) {
    colours.push_back(cv::Scalar(distrib(gen), distrib(gen), distrib(gen)));
  }
  return colours;
}
static vector<cv::Scalar> colours = GenerateColours();

vector<string> split(const string& s, char delim) {
    vector<string> result;
    istringstream ss(s);
    string item;

    while (getline(ss, item, delim)) {
        result.push_back (item);
    }
    return result;
}

vector<vector<Bbox>> ReadData(string file_path) {
  std::fstream f(file_path);
  CHECK(f.is_open() == true) << "can't open " << file_path;
  // read data
  vector<string> lines;
  std::string line;
  while (getline(f, line)) {
    lines.push_back(line);
  }
  int frames = stoi(split(lines.back(), ',')[0]);
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
    float y2 = y1 + h;
    data[frame_id-1].push_back(Bbox(x1, y1, x2, y2));
  }
  return data;
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);

  // create output directory
  string output_dir("../output/");
  mkdir(output_dir.c_str(), S_IRUSR | S_IWUSR | S_IXUSR);

  vector<string> file_paths{
    "../data/train/ADL-Rundle-6/det/det.txt",
    "../data/train/ADL-Rundle-8/det/det.txt", 
    "../data/train/ETH-Bahnhof/det/det.txt",
    "../data/train/ETH-Pedcross2/det/det.txt",
    "../data/train/ETH-Sunnyday/det/det.txt",
    "../data/train/KITTI-13/det/det.txt",
    "../data/train/KITTI-17/det/det.txt",
    "../data/train/PETS09-S2L1/det/det.txt",
    "../data/train/TUD-Campus/det/det.txt",
    "../data/train/TUD-Stadtmitte/det/det.txt",
    "../data/train/Venice-2/det/det.txt",
  };

  std::chrono::duration<double> total_time;
  int total_frames = 0;
  for (auto file_path : file_paths) {
    LOG(INFO) << "Processing " << file_path;
    // read data
    vector<vector<Bbox>> data = ReadData(file_path);
    // output file
    string video(split(file_path, '/')[3]);
    ofstream of_stream(output_dir+video+".txt");
    of_stream << fixed << setprecision(2);
    
    Sort mot_tracker;
    int frame = 0;
    for (int i = 0; i < data.size(); ++i) {
      frame = i + 1;
      // update
      auto start = chrono::steady_clock::now();
      vector<BboxID> bboxids = mot_tracker.Update(data[i]);
      auto end = chrono::steady_clock::now();
      std::chrono::duration<double> elapsed_seconds = end-start;
      total_time += elapsed_seconds;
      // show image and save reslt
      ostringstream ss;
      ss << setw(6) << right << setfill('0') << frame;
      string img_path = "../mot_benchmark/train/" + video + "/img1/" + 
                        ss.str() + ".jpg";
      cv::Mat img = cv::imread(img_path);
      for (auto& bboxid : bboxids) {
        int id = bboxid.id;
        const Bbox& bbox = bboxid.bbox;
        cv::rectangle(img, cv::Point(bbox.x1, bbox.y1), 
                      cv::Point(bbox.x2, bbox.y2), colours[id%32], 2);
        cv::putText(img, to_string(id), cv::Point(bbox.x1, bbox.y1-2), 
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        of_stream << frame << "," 
                  << id << "," 
                  << bbox.x1 << "," 
                  << bbox.y1 << ","
                  << bbox.x2 - bbox.x1 << ","
                  << bbox.y2 - bbox.y1 << ","
                  << -1 << ","
                  << -1 << ","
                  << -1 << "\n";
      }
      cv::namedWindow("frame", cv::WINDOW_NORMAL);
      cv::imshow("frame", img);
      cv::waitKey(1);
      total_frames++;
    }
  }
  LOG(INFO) << "Total tracking took: " << fixed << setprecision(3) 
            << total_time.count() << " seconds for " 
            << total_frames << " frames or " << setprecision(0) 
            << total_frames / total_time.count() << "FPS";
}