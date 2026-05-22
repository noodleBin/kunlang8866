#include "cyber/component/component.h"
namespace century {
namespace perception {
namespace lidar {
namespace{
const std::string process_path = "/century/bazel-bin/modules/perception/lidar_tracking/";
const std::string process_name = "sample_lidar_tracking";
}
class ShellComponent : public century::cyber::Component<> {
 public:
  bool Init() override { 
    std::cout << "ShellComponent Init" << std::endl;
    const std::string command = "pkill -f " + process_name;
    system(command.c_str());
    pid_t pid = fork();
    if (pid == -1) {
      std::cerr << "Fork failed!" << std::endl;
      return false;
    } else if (pid == 0) { 
      execl((process_path + process_name).c_str(), process_name.c_str(), nullptr);   
      std::cerr << "Failed to execute binary!" << std::endl;
    } else {
      std::cout << "Child process PID: " << pid << std::endl;
    }
    std::cout << "ShellComponent Init Exit" << std::endl;
    return true;
  }
  virtual ~ShellComponent() {
    std::cout << "ShellComponent DeInit" << std::endl;
    const std::string command = "pkill -f " + process_name;
    int result = system(command.c_str());
    if (result != -1) {
      std::cout << process_name << " process killed successfully." << std::endl;
    } else {
      std::cerr << process_name <<" process NOT killed." << std::endl;
    }
  }
};
CYBER_REGISTER_COMPONENT(ShellComponent)
  
  
}
}
}