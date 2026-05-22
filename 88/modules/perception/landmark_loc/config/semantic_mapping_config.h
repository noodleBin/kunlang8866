#pragma once
#include "yaml-cpp/yaml.h"
#include <unordered_set>
#include <array>
namespace semantic_mapping{
class SemanticMappingConfig {
public:
static SemanticMappingConfig* GetInstance() {
  static SemanticMappingConfig cfg;
  return &cfg;
}

std::string ResFolder() const {
  return res_folder_;
}

std::string DebugFolder() const {
  return debug_folder_;
}

int MapWidth() const {
  return map_width_;
}

int KeyFrameInterval() const {
  return key_frame_interval_;
}

double KeyFramePosThes() const {
  return key_frame_pos_thres_;
}

double KeyFrameAngThes() const {
  return key_frame_ang_thres_;
}

double SegThres() const {
  return seg_thres_;
}

bool LoopClosing() const {
  return loop_closing_;
}

int Mode() const {
  return mode_;
}
private:
SemanticMappingConfig();
int mode_ = 0;
std::string res_folder_ = "/century/data/data/";
std::string debug_folder_ = "/century/data/log/";
int map_width_= 50;
int key_frame_interval_ = 50;
double key_frame_pos_thres_ = 0.2;
double key_frame_ang_thres_ =  5 * M_PI / 180;
double seg_thres_ = 0.5;
bool loop_closing_ = true;


const std::string config_file_ = "/century/modules/perception/landmark_loc/res/semantic_mapping.yaml";
};

}