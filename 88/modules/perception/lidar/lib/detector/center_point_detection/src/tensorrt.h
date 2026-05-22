#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace century {
namespace perception {
namespace lidar {
namespace centerpoint {
namespace TensorRT {

class Engine {
 public:
  virtual int64_t getBindingNumel(const std::string& name) = 0;
  virtual std::vector<int64_t> getBindingDims(const std::string& name) = 0;
  virtual bool forward(const std::initializer_list<void*>& buffers, void* stream = nullptr) = 0;
  virtual void print() = 0;
};

std::shared_ptr<Engine> load_plan(const std::string& file);

}  // namespace TensorRT
}  // namespace centerpoint
}  // namespace lidar
}  // namespace perception
}  // namespace century
