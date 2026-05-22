#pragma once

#include <functional>
#include <string>
#include <vector>

namespace century {
namespace drivers {
namespace camera {

using PlatformConfigWarnFn = std::function<void(const std::string&)>;

std::string ResolveHbJ5devPath(const std::string& platform_horizon_path,
                               const std::string& legacy_hb_j5dev_path);

std::vector<std::string> DefaultCameraPowerValuePaths();
std::vector<std::string> ResolveCameraPowerValuePaths(
    const std::vector<std::string>& configured_paths,
    const PlatformConfigWarnFn& warn_fn = PlatformConfigWarnFn());

}  // namespace camera
}  // namespace drivers
}  // namespace century
