/******************************************************************************
 * Copyright 2022 The Century Authors. All Rights Reserved.
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

/**
 * @file speed_limit.cc
 **/

#include "modules/planning/common/speed_limit.h"

#include <algorithm>

#include "cyber/common/log.h"

namespace century {
namespace planning {

void SpeedLimit::AppendSpeedLimit(const double s, const double v) {
  if (!speed_limit_points_.empty()) {
    DCHECK_GE(s, speed_limit_points_.back().first);
  }
  speed_limit_points_.emplace_back(s, v);
}

void SpeedLimit::SetSpeedLimitByIndex(uint32_t index, const double s,
                                      double v) {
  if (!speed_limit_points_.empty()) {
    DCHECK_GE(speed_limit_points_.size(), index);
  }
  if (index < speed_limit_points_.size()) {
    speed_limit_points_[index].first = s;
    speed_limit_points_[index].second = v;
  }
}

void SpeedLimit::SetSpeedLimitByS(const double s, const double v) {
  CHECK_GE(speed_limit_points_.size(), 1U);
  auto compare_s = [](const std::pair<double, double>& point, const double s) {
    return point.first < s;
  };

  auto lower = std::lower_bound(speed_limit_points_.begin(),
                                speed_limit_points_.end(), s, compare_s);
  if (speed_limit_points_.begin() == lower) {
    (*lower).second = v;
    return;
  }

  if (speed_limit_points_.end() == lower) {
    (*(lower - 1)).second = v;
    return;
  }
  if (s - (*(lower - 1)).first > (*lower).first - s) {
    (*lower).first = s;
    (*lower).second = v;

  } else {
    (*(lower - 1)).first = s;
    (*(lower - 1)).second = v;
  }
}

void SpeedLimit::UpdateSpeedLimitByS(const double s, const double v) {
  CHECK_GE(speed_limit_points_.size(), 1U);
  auto compare_s = [](const std::pair<double, double>& point, const double s) {
    return point.first < s;
  };

  auto lower = std::lower_bound(speed_limit_points_.begin(),
                                speed_limit_points_.end(), s, compare_s);
  if (speed_limit_points_.begin() == lower) {
    (*lower).second = v;
    return;
  }

  if (speed_limit_points_.end() == lower) {
    (*(lower - 1)).second = v;
    return;
  }
  if (s - (*(lower - 1)).first > (*lower).first - s) {
    (*lower).second = v;
  } else {
    (*(lower - 1)).second = v;
  }
}

void SpeedLimit::UpdateSpeedLimitWithRawDataBySRange(const double start_s,
                                                     const double end_s,
                                                     const double v) {
  if (speed_limit_points_.size() < 1U) {
    return;
  }
  double s1 = start_s, s2 = end_s;
  if (s1 > s2) {
    std::swap(s1, s2);
  }
  auto compare_s = [](const std::pair<double, double>& point, const double s) {
    return point.first < s;
  };

  auto lower = std::lower_bound(speed_limit_points_.begin(),
                                speed_limit_points_.end(), s1, compare_s);
  auto upper = std::lower_bound(speed_limit_points_.begin(),
                                speed_limit_points_.end(), s2, compare_s);
  if (speed_limit_points_.begin() == upper ||
      speed_limit_points_.end() == lower) {
    return;
  }
  if (lower == upper) {
    (*lower).second = std::min(v, (*lower).second);
  } else {
    // DON'T contain upper item.
    for (auto it = lower; it != upper; ++it) {
      (*it).second = std::min(v, (*it).second);
    }
  }
}

void SpeedLimit::InitSpeedLimit(const double start_s, const double end_s,
                                const double step, const double v) {
  speed_limit_points_.clear();
  if (start_s > end_s) {
    return;
  }
  for (double s = start_s; s <= end_s; s += step) {
    speed_limit_points_.emplace_back(s, v);
  }
}
const std::vector<std::pair<double, double>>& SpeedLimit::speed_limit_points()
    const {
  return speed_limit_points_;
}

double SpeedLimit::GetSpeedLimitByS(const double s) const {
  if (speed_limit_points_.size() < 1U) {
    return 0.0;
  }
  if (s < speed_limit_points_.front().first) {
    return speed_limit_points_.front().second;
  }
  CHECK_GE(speed_limit_points_.size(), 1U);
  DCHECK_GE(s, speed_limit_points_.front().first);

  auto compare_s = [](const std::pair<double, double>& point, const double s) {
    return point.first < s;
  };

  auto it_lower = std::lower_bound(speed_limit_points_.begin(),
                                   speed_limit_points_.end(), s, compare_s);

  if (it_lower == speed_limit_points_.end()) {
    return (it_lower - 1)->second;
  }
  return it_lower->second;
}

double SpeedLimit::GetMaxSpeedLimitBySRange(const double s1,
                                            const double s2) const {
  CHECK_GE(speed_limit_points_.size(), 1U);
  double s_min = std::min(s1, s2);
  double s_max = std::max(s1, s2);
  s_min = std::max(s_min, speed_limit_points_.front().first);
  auto compare_s = [](const std::pair<double, double>& point, const double s) {
    return point.first < s;
  };
  auto compare_speed_limit = [](const std::pair<double, double>& point1,
                                const std::pair<double, double>& point2) {
    return point1.second < point2.second;
  };

  auto it_s_min = std::lower_bound(speed_limit_points_.begin(),
                                   speed_limit_points_.end(), s_min, compare_s);
  auto it_s_max = std::lower_bound(speed_limit_points_.begin(),
                                   speed_limit_points_.end(), s_max, compare_s);
  if (speed_limit_points_.end() == it_s_min) {
    std::advance(it_s_min, -1);
  }
  if (speed_limit_points_.end() == it_s_max) {
    std::advance(it_s_max, -1);
  }
  std::advance(it_s_max, 1);
  auto it_max_speed_limit =
      std::max_element(it_s_min, it_s_max, compare_speed_limit);
  return it_max_speed_limit->second;
}

void SpeedLimit::Clear() { speed_limit_points_.clear(); }

}  // namespace planning
}  // namespace century
