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

#pragma once

#include <string>

#include "modules/planning/proto/planning.pb.h"

namespace century {
namespace planning {

enum class LaneBorrowStatus : uint8_t {
  DEFAULT = 0,
  PREPARE,
  LEFTBORROW,
  RIGHTBORROW,
  RETURN,
  FINISH,
};

class LaneBorrowTickCheck {
 public:
  static void Set(const LaneBorrowStatus &state) {
    if (state_ != state) {
      state_ = state;
      tick_ = 0;
    } else {
      ++tick_;
    }
  }

  static uint64_t Tick(const LaneBorrowStatus &state) {
    return (state_ != state) ? 0 : tick_;
  }

 private:
  static LaneBorrowStatus state_;
  static uint64_t tick_;
};

std::string Status2String(const LaneBorrowStatus &status) {
  std::string msg = "";
  switch (status) {
    case LaneBorrowStatus::DEFAULT:
      msg = "'default'";
      break;
    case LaneBorrowStatus::PREPARE:
      msg = "'prepare'";
      break;
    case LaneBorrowStatus::LEFTBORROW:
      msg = "'left_borrow'";
      break;
    case LaneBorrowStatus::RIGHTBORROW:
      msg = "'right_borrow'";
      break;
    case LaneBorrowStatus::RETURN:
      msg = "'return'";
      break;
    case LaneBorrowStatus::FINISH:
      msg = "'finish'";
      break;

    default:
      msg = "'default'";
      break;
  }
  return msg;
}

}  // namespace planning
}  // namespace century
