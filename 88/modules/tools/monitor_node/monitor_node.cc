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

// #include "cyber/examples/proto/examples.pb.h"
#include "cyber/message/raw_message.h"

#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "nlohmann/json.hpp"
#include "cyber/timer/timer.h"

#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <unordered_map>
#include <optional>

using nlohmann::json;
using century::cyber::message::RawMessage;


constexpr const double start_need_time = 10.0;

std::optional<long> GetProcessStartTimeByCmd(const std::string& processCmd)
{
    std::string cmd = "pgrep -f '" + processCmd + "' | head -n 1";
    
    std::array<char, 128> buf{};
    std::string pid_str;
    
    std::unique_ptr<FILE, decltype(&pclose)> pipe_pid(popen(cmd.c_str(), "r"), pclose);
    if (!pipe_pid) return std::nullopt;
    
    while (fgets(buf.data(), buf.size(), pipe_pid.get()) != nullptr) {
        pid_str += buf.data();
    }
    
    if (pid_str.empty()) {
        return 0L;
    }
    
    pid_str.erase(0, pid_str.find_first_not_of(" \t\n\r"));
    pid_str.erase(pid_str.find_last_not_of(" \t\n\r") + 1);
    
    std::string ps_cmd = "ps -p " + pid_str + " -o lstart=";
    std::string output;
    
    std::unique_ptr<FILE, decltype(&pclose)> pipe_ps(popen(ps_cmd.c_str(), "r"), pclose);
    if (!pipe_ps) return std::nullopt;
    
    buf.fill('\0');
    while (fgets(buf.data(), buf.size(), pipe_ps.get()) != nullptr) {
        output += buf.data();
    }
    
    if (output.empty()) {
        return 0L;
    }
    
    output.erase(output.find_last_not_of(" \t\n\r") + 1);
    
    struct tm tm_time = {};
    if (strptime(output.c_str(), "%a %b %d %H:%M:%S %Y", &tm_time) == nullptr) {
        return 0L;
    }
    
    time_t start_time = mktime(&tm_time);
    return static_cast<long>(start_time);
}
std::optional<long> GetProcessRunningSecondsSimple(const std::string& processCmd)
{
    std::string cmd = "ps -ef | grep -E '" + processCmd + "' | grep -v grep | head -n 1";
    cmd += " | xargs -I {} sh -c \"echo {} | awk '{print \\$2}'\"";
    cmd += " | xargs -I {} ps -p {} -o etimes= 2>/dev/null";
    
    std::array<char, 128> buf{};
    std::string output;
    
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) {
        return std::nullopt;
    }
    
    while (fgets(buf.data(), buf.size(), pipe.get()) != nullptr) {
        output += buf.data();
    }
    
    output.erase(0, output.find_first_not_of(" \t\n\r"));
    output.erase(output.find_last_not_of(" \t\n\r") + 1);
    
    if (output.empty()) {
        return 0L;
    }
    
    try {
        long seconds = std::stol(output);
        return seconds;
    } catch (...) {
        return 0L;
    }
}
bool KillProcessByName(const std::string& processCmd)
{
    std::string pid_cmd = "pgrep -f '" + processCmd + "' 2>/dev/null | head -n 1";
    std::array<char, 128> buf{};
    std::string pid_str;
    
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(pid_cmd.c_str(), "r"), pclose);
    if (!pipe) {
        AERROR << "Failed to get PID for: " << processCmd;
        return false;
    }
    
    while (fgets(buf.data(), buf.size(), pipe.get()) != nullptr) {
        pid_str += buf.data();
    }
    
    pid_str.erase(0, pid_str.find_first_not_of(" \t\n\r"));
    pid_str.erase(pid_str.find_last_not_of(" \t\n\r") + 1);
    
    if (pid_str.empty()) {
        AINFO << "Process not found: " << processCmd;
        return false;
    }
    
    std::string kill_cmd = "kill -9 " + pid_str + " 2>/dev/null";
    AINFO << "Killing PID " << pid_str << " for command: " << processCmd;
    
    int ret = system(kill_cmd.c_str());
    return (ret == 0);
}

void MessageCallback(
    const std::shared_ptr<RawMessage>& msg) {

}


std::map<std::string, std::string> kill_process_;
std::map<std::string, double> channel_recv_time_;

std::unordered_map<std::string, std::shared_ptr<century::cyber::Reader<RawMessage>>> channel_;

int main(int argc, char* argv[]) {

  std::ifstream file("/century/monitor_config.json");
  century::cyber::Init(argv[0]);

  nlohmann::json config = nlohmann::json::parse(file);

  // AINFO<< "json : " << config.dump(4) ;
  auto listener_node = century::cyber::CreateNode("node_monitor_freq");


  for (auto channel : config["channel"]) {
    auto name = channel["name"];
    kill_process_[name] = channel["process"];
    channel_recv_time_[name] = century::cyber::Time::Now().ToSecond();

    channel_[name] = listener_node->CreateReader<RawMessage>(name, [&, name](const std::shared_ptr<RawMessage>& message){
      channel_recv_time_[name] = century::cyber::Time::Now().ToSecond();
    });
  }

  century::cyber::Timer timer(
      1000, [&] { 
        auto now = century::cyber::Time::Now().ToSecond();
        for (auto channel : channel_recv_time_) {

            auto process = kill_process_[channel.first];
            auto run_sec = GetProcessRunningSecondsSimple(process);
            AINFO<< "channel name : " << channel.first << "  run_sec : " << *run_sec <<"    diff : " << now - channel_recv_time_[channel.first] ;
            if (run_sec && *run_sec > 10.0 && now - channel_recv_time_[channel.first] > 3.0 ) {
              channel_recv_time_[channel.first] = now;
              KillProcessByName(process);
            }
        }
       }, false);

  timer.Start();
  century::cyber::WaitForShutdown();
  return 0;
}
