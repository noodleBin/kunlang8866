/******************************************************************************
 * Copyright 2018 The Century Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <getopt.h>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/common/time_conversion.h"
#include "cyber/init.h"
#include "cyber/tools/cyber_recorder/info.h"
#include "cyber/tools/cyber_recorder/player/player.h"
#include "cyber/tools/cyber_recorder/recorder.h"
#include "cyber/tools/cyber_recorder/recoverer.h"
#include "cyber/tools/cyber_recorder/spliter.h"

using century::cyber::common::GetFileName;
using century::cyber::common::StringToUnixSeconds;
using century::cyber::common::UnixSecondsToString;
using century::cyber::record::HeaderBuilder;
using century::cyber::record::Info;
using century::cyber::record::Player;
using century::cyber::record::PlayParam;
using century::cyber::record::Recorder;
using century::cyber::record::Recoverer;
using century::cyber::record::Spliter;

const char INFO_OPTIONS[] = "h";
const char RECORD_OPTIONS[] = "o:ac:k:i:m:h";
const char PLAY_OPTIONS[] = "f:ac:k:lr:b:e:s:d:p:h";
const char SPLIT_OPTIONS[] = "f:o:c:k:b:e:h";
const char RECOVER_OPTIONS[] = "f:o:h";

void DisplayUsage(const std::string& binary);
void DisplayUsage(const std::string& binary, const std::string& command);
void DisplayUsage(const std::string& binary, const std::string& command,
                  const std::string& options);

void DisplayUsage(const std::string& binary) {
  std::cout << "usage: " << binary << " <command> [<args>]\n"
            << "The " << binary << " commands are:\n"
            << "\tinfo\tShow information of an exist record.\n"
            << "\tplay\tPlay an exist record.\n"
            << "\trecord\tRecord same topic.\n"
            << "\tsplit\tSplit an exist record.\n"
            << "\trecover\tRecover an exist record.\n"
            << std::endl;
}

void DisplayUsage(const std::string& binary, const std::string& command) {
  if (command == "info") {
    std::cout << "usage: cyber_recorder info file" << std::endl;
    std::cout << "usage: " << binary << " " << command << " [options]"
              << std::endl;
    DisplayUsage(binary, command, INFO_OPTIONS);
    return;
  }

  std::cout << "usage: " << binary << " " << command << " [options]"
            << std::endl;
  if (command == "record") {
    DisplayUsage(binary, command, RECORD_OPTIONS);
  } else if (command == "play") {
    DisplayUsage(binary, command, PLAY_OPTIONS);
  } else if (command == "split") {
    DisplayUsage(binary, command, SPLIT_OPTIONS);
  } else if (command == "recover") {
    DisplayUsage(binary, command, RECOVER_OPTIONS);
  } else {
    std::cout << "Unknown command: " << command << std::endl;
    DisplayUsage(binary);
  }
}

void DisplayUsage(const std::string& binary, const std::string& command,
                  const std::string& options) {
  for (char option : options) {
    switch (option) {
      case 'f':
        std::cout << "\t-f, --file <file>\t\t\tinput record file" << std::endl;
        break;
      case 'o':
        std::cout << "\t-o, --output <file>\t\t\toutput record file"
                  << std::endl;
        break;
      case 'a':
        std::cout << "\t-a, --all\t\t\t\t" << command << " all" << std::endl;
        break;
      case 'c':
        std::cout << "\t-c, --white-channel <name>\t\tonly " << command
                  << " the specified channel" << std::endl;
        break;
      case 'k':
        std::cout << "\t-k, --black-channel <name>\t\tnot " << command
                  << " the specified channel" << std::endl;
        break;
      case 'l':
        std::cout << "\t-l, --loop\t\t\t\tloop " << command << std::endl;
        break;
      case 'r':
        std::cout << "\t-r, --rate <1.0>\t\t\tmultiply the " << command
                  << " rate by FACTOR" << std::endl;
        break;
      case 'b':
        std::cout << "\t-b, --begin 2018-07-01-00:00:00\t" << command
                  << " the record begin at" << std::endl;
        break;
      case 'e':
        std::cout << "\t-e, --end 2018-07-01-00:01:00\t\t" << command
                  << " the record end at" << std::endl;
        break;
      case 's':
        std::cout << "\t-s, --start <seconds>\t\t\t" << command
                  << " started at n seconds" << std::endl;
        break;
      case 'd':
        std::cout << "\t-d, --delay <seconds>\t\t\t" << command
                  << " delayed n seconds" << std::endl;
        break;
      case 'p':
        std::cout << "\t-p, --preload <seconds>\t\t\t" << command
                  << " after trying to preload n second(s)" << std::endl;
        break;
      case 'i':
        std::cout << "\t-i, --segment-interval <seconds>\t" << command
                  << " segmented every n second(s)" << std::endl;
        break;
      case 'm':
        std::cout << "\t-m, --segment-size <MB>\t\t\t" << command
                  << " segmented every n megabyte(s)" << std::endl;
        break;
      case 'h':
        std::cout << "\t-h, --help\t\t\t\tshow help message" << std::endl;
        break;
      case ':':
        break;
      default:
        std::cout << "unknown option: -" << option;
        break;
    }
  }
}

struct Options {
  std::vector<std::string> opt_file_vec;
  std::vector<std::string> opt_output_vec;
  std::vector<std::string> opt_white_channels;
  std::vector<std::string> opt_black_channels;
  bool opt_all = false;
  bool opt_loop = false;
  float opt_rate = 1.0f;
  uint64_t opt_begin = 0;
  uint64_t opt_end = std::numeric_limits<uint64_t>::max();
  uint64_t opt_start = 0;
  uint64_t opt_delay = 0;
  uint32_t opt_preload = 3;
  ::century::cyber::proto::Header opt_header = HeaderBuilder::GetHeader();
};

bool InitBasicInfo(int argc, char** argv, std::string& binary,
                   std::string& command, std::string& file_path) {
  binary = GetFileName(std::string(argv[0]));
  if (argc < 2) {
    DisplayUsage(binary);
    return false;
  }
  command = std::string(argv[1]);
  if (argc >= 3) {
    file_path = std::string(argv[2]);
  }
  return true;
}

bool ParseOptionFiles(int argc, char** argv,
                      std::vector<std::string>& file_vec) {
  file_vec.emplace_back(std::string(optarg));
  for (int i = optind; i < argc; ++i) {
    if (*argv[i] != '-') {
      file_vec.emplace_back(std::string(argv[i]));
    } else {
      break;
    }
  }
  return true;
}

bool ParseOptionChannels(int argc, char** argv,
                         std::vector<std::string>& channels) {
  channels.emplace_back(std::string(optarg));
  for (int i = optind; i < argc; ++i) {
    if (*argv[i] != '-') {
      channels.emplace_back(std::string(argv[i]));
    } else {
      break;
    }
  }
  return true;
}

bool ParseOptionRate(const std::string& binary, const std::string& command,
                     float& opt_rate) {
  try {
    opt_rate = std::stof(optarg);
  } catch (const std::invalid_argument& ia) {
    std::cout << "Invalid argument: -r/--rate " << std::string(optarg)
              << std::endl;
    return false;
  } catch (const std::out_of_range& e) {
    std::cout << "Argument is out of range: -r/--rate "
              << std::string(optarg) << std::endl;
    return false;
  }
  return true;
}

bool ParseOptionInt(const std::string& opt_name, uint64_t& opt_value) {
  try {
    opt_value = std::stoi(optarg);
  } catch (const std::invalid_argument& ia) {
    std::cout << "Invalid argument: " << opt_name << " " << std::string(optarg)
              << std::endl;
    return false;
  } catch (const std::out_of_range& e) {
    std::cout << "Argument is out of range: " << opt_name << " "
              << std::string(optarg) << std::endl;
    return false;
  }
  return true;
}

bool ParseOptionPreload(const std::string& opt_name, uint32_t& opt_value) {
  try {
    opt_value = std::stoi(optarg);
  } catch (const std::invalid_argument& ia) {
    std::cout << "Invalid argument: " << opt_name << " " << std::string(optarg)
              << std::endl;
    return false;
  } catch (const std::out_of_range& e) {
    std::cout << "Argument is out of range: " << opt_name << " "
              << std::string(optarg) << std::endl;
    return false;
  }
  return true;
}

bool ParseOptionSegmentInterval(::century::cyber::proto::Header& opt_header) {
  try {
    int interval_s = std::stoi(optarg);
    if (interval_s < 0) {
      std::cout << "Argument is less than zero: -i/--segment-interval "
                << std::string(optarg) << std::endl;
      return false;
    }
    opt_header.set_segment_interval(interval_s * 1000000000ULL);
  } catch (const std::invalid_argument& ia) {
    std::cout << "Invalid argument: -i/--segment-interval "
              << std::string(optarg) << std::endl;
    return false;
  } catch (const std::out_of_range& e) {
    std::cout << "Argument is out of range: -i/--segment-interval "
              << std::string(optarg) << std::endl;
    return false;
  }
  return true;
}

bool ParseOptionSegmentSize(::century::cyber::proto::Header& opt_header) {
  try {
    int size_mb = std::stoi(optarg);
    if (size_mb < 0) {
      std::cout << "Argument is less than zero: -m/--segment-size "
                << std::string(optarg) << std::endl;
      return false;
    }
    opt_header.set_segment_raw_size(size_mb * 1024 * 1024ULL);
  } catch (const std::invalid_argument& ia) {
    std::cout << "Invalid argument: -m/--segment-size " << std::string(optarg)
              << std::endl;
    return false;
  } catch (const std::out_of_range& e) {
    std::cout << "Argument is out of range: -m/--segment-size "
              << std::string(optarg) << std::endl;
    return false;
  }
  return true;
}

bool ParseOptions(int argc, char** argv, const std::string& binary,
                  const std::string& command, Options& opt) {
  int long_index = 0;
  const std::string short_opts = "f:c:k:o:alr:b:e:s:d:p:i:m:h";
  static const struct option long_opts[] = {
      {"files", required_argument, nullptr, 'f'},
      {"white-channel", required_argument, nullptr, 'c'},
      {"black-channel", required_argument, nullptr, 'k'},
      {"output", required_argument, nullptr, 'o'},
      {"all", no_argument, nullptr, 'a'},
      {"loop", no_argument, nullptr, 'l'},
      {"rate", required_argument, nullptr, 'r'},
      {"begin", required_argument, nullptr, 'b'},
      {"end", required_argument, nullptr, 'e'},
      {"start", required_argument, nullptr, 's'},
      {"delay", required_argument, nullptr, 'd'},
      {"preload", required_argument, nullptr, 'p'},
      {"segment-interval", required_argument, nullptr, 'i'},
      {"segment-size", required_argument, nullptr, 'm'},
      {"help", no_argument, nullptr, 'h'}};

  do {
    int opt_char =
        getopt_long(argc, argv, short_opts.c_str(), long_opts, &long_index);
    if (-1 == opt_char) {
      break;
    }

    switch (opt_char) {
      case 'f':
        ParseOptionFiles(argc, argv, opt.opt_file_vec);
        break;
      case 'c':
        ParseOptionChannels(argc, argv, opt.opt_white_channels);
        break;
      case 'k':
        ParseOptionChannels(argc, argv, opt.opt_black_channels);
        break;
      case 'o':
        opt.opt_output_vec.push_back(std::string(optarg));
        break;
      case 'a':
        opt.opt_all = true;
        break;
      case 'l':
        opt.opt_loop = true;
        break;
      case 'r':
        if (!ParseOptionRate(binary, command, opt.opt_rate)) {
          return false;
        }
        break;
      case 'b':
        opt.opt_begin =
            StringToUnixSeconds(std::string(optarg)) * 1000 * 1000 * 1000ULL;
        break;
      case 'e':
        opt.opt_end =
            StringToUnixSeconds(std::string(optarg)) * 1000 * 1000 * 1000ULL;
        break;
      case 's':
        if (!ParseOptionInt("-s/--start", opt.opt_start)) {
          return false;
        }
        break;
      case 'd':
        if (!ParseOptionInt("-d/--delay", opt.opt_delay)) {
          return false;
        }
        break;
      case 'p':
        if (!ParseOptionPreload("-p/--preload", opt.opt_preload)) {
          return false;
        }
        break;
      case 'i':
        if (!ParseOptionSegmentInterval(opt.opt_header)) {
          return false;
        }
        break;
      case 'm':
        if (!ParseOptionSegmentSize(opt.opt_header)) {
          return false;
        }
        break;
      case 'h':
        DisplayUsage(binary, command);
        return false;
      default:
        break;
    }
  } while (true);

  return true;
}

int ProcInfoCommand(const std::string& file_path, char* argv0) {
  if (file_path.empty()) {
    std::cout << "usage: cyber_recorder info file" << std::endl;
    return -1;
  }
  ::century::cyber::Init(argv0);
  Info info;
  bool info_result = info.Display(file_path);
  return info_result ? 0 : -1;
}

int ProcRecoverCommand(const Options& opt, char* argv0) {
  if (opt.opt_file_vec.empty()) {
    std::cout << "MUST specify file option (-f)." << std::endl;
    return -1;
  }
  if (opt.opt_file_vec.size() > 1 || opt.opt_output_vec.size() > 1) {
    std::cout << "TOO many input/output file option (-f/-o)." << std::endl;
    return -1;
  }
  std::string output_file;
  if (opt.opt_output_vec.empty()) {
    output_file = opt.opt_file_vec[0] + ".recover";
  } else {
    output_file = opt.opt_output_vec[0];
  }
  ::century::cyber::Init(argv0);
  Recoverer recoverer(opt.opt_file_vec[0], output_file);
  bool recover_result = recoverer.Proc();
  return recover_result ? 0 : -1;
}

int ProcPlayCommand(const Options& opt, char* argv0) {
  if (opt.opt_file_vec.empty()) {
    std::cout << "MUST specify file option (-f)." << std::endl;
    return -1;
  }
  ::century::cyber::Init(argv0);
  PlayParam play_param;
  play_param.is_play_all_channels = opt.opt_all || opt.opt_white_channels.empty();
  play_param.is_loop_playback = opt.opt_loop;
  play_param.play_rate = opt.opt_rate;
  play_param.begin_time_ns = opt.opt_begin;
  play_param.end_time_ns = opt.opt_end;
  play_param.start_time_s = opt.opt_start;
  play_param.delay_time_s = opt.opt_delay;
  play_param.preload_time_s = opt.opt_preload;
  play_param.files_to_play.insert(opt.opt_file_vec.begin(),
                                   opt.opt_file_vec.end());
  play_param.black_channels.insert(opt.opt_black_channels.begin(),
                                   opt.opt_black_channels.end());
  play_param.channels_to_play.insert(opt.opt_white_channels.begin(),
                                     opt.opt_white_channels.end());
  Player player(play_param);
  const bool play_result = player.Init() && player.Start();
  return play_result ? 0 : -1;
}

int ProcRecordCommand(Options& opt, char* argv0) {
  if (opt.opt_white_channels.empty() && !opt.opt_all) {
    std::cout
        << "MUST specify channels option (-c) or all channels option (-a)."
        << std::endl;
    return -1;
  }
  if (opt.opt_output_vec.size() > 1) {
    std::cout << "TOO many output file option (-o)." << std::endl;
    return -1;
  }
  if (opt.opt_output_vec.empty()) {
    std::string default_output_file =
        UnixSecondsToString(time(nullptr), "%Y%m%d%H%M%S") + ".record";
    opt.opt_output_vec.push_back(default_output_file);
  }
  ::century::cyber::Init(argv0);
  auto recorder = std::make_shared<Recorder>(opt.opt_output_vec[0], opt.opt_all,
                                             opt.opt_white_channels,
                                             opt.opt_black_channels,
                                             opt.opt_header);
  bool record_result = recorder->Start();
  if (record_result) {
    while (!::century::cyber::IsShutdown()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    record_result = recorder->Stop();
  }
  return record_result ? 0 : -1;
}

int ProcSplitCommand(Options& opt, char* argv0) {
  if (opt.opt_file_vec.empty()) {
    std::cout << "Must specify file option (-f)." << std::endl;
    return -1;
  }
  if (opt.opt_file_vec.size() > 1 || opt.opt_output_vec.size() > 1) {
    std::cout << "Too many input/output file option (-f/-o)." << std::endl;
    return -1;
  }
  if (opt.opt_output_vec.empty()) {
    std::string default_output_file = opt.opt_file_vec[0] + ".split";
    opt.opt_output_vec.push_back(default_output_file);
  }
  ::century::cyber::Init(argv0);
  Spliter spliter(opt.opt_file_vec[0], opt.opt_output_vec[0],
                  opt.opt_white_channels, opt.opt_black_channels, opt.opt_begin,
                  opt.opt_end);
  bool split_result = spliter.Proc();
  return split_result ? 0 : -1;
}

int main(int argc, char** argv) {
  std::string binary;
  std::string command;
  std::string file_path;

  if (!InitBasicInfo(argc, argv, binary, command, file_path)) {
    return -1;
  }

  Options opt;
  if (!ParseOptions(argc, argv, binary, command, opt)) {
    return -1;
  }

  if ("info" == command) {
    return ProcInfoCommand(file_path, argv[0]);
  }

  if ("recover" == command) {
    return ProcRecoverCommand(opt, argv[0]);
  }

  if ("play" == command) {
    return ProcPlayCommand(opt, argv[0]);
  }

  if ("record" == command) {
    return ProcRecordCommand(opt, argv[0]);
  }

  if ("split" == command) {
    return ProcSplitCommand(opt, argv[0]);
  }

  DisplayUsage(binary, command);
  return -1;
}
