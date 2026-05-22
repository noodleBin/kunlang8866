/******************************************************************************
 * Copyright 2023 The Century Authors. All Rights Reserved.
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
 *  @brief  A standard container automatically sorting its contents.
 *
 *  @author ShenWeiHua 20230427
 *
 */

/*
 * @file
 */

#pragma once

#include <functional>
#include <iostream>
#include <queue>
#include <utility>
#include <vector>

namespace century {
namespace planning {

// Customize the c++ priority queue,
// and the implementation can delete the elements that are not top.
template <typename _Tp, typename _Sequence = std::vector<_Tp>,
          typename _Compare = std::less<typename _Sequence::value_type>>
class custom_priority_queue
    : public std::priority_queue<_Tp, _Sequence, _Compare> {
 public:
  // remove not top value by swh 20230427
  bool remove(const _Tp& value) {
    auto it = std::find(this->c.begin(), this->c.end(), value);
    if (it != this->c.end()) {
      this->c.erase(it);
      std::make_heap(this->c.begin(), this->c.end(), this->comp);
      return true;
    } else {
      return false;
    }
  }

  // update src to dst by swh 20230427,equals first remove and second push back
  bool update(const _Tp& src, const _Tp& dst) {
    auto it = std::find(this->c.begin(), this->c.end(), src);
    if (it != this->c.end()) {
      // First remove
      this->c.erase(it);
      std::make_heap(this->c.begin(), this->c.end(), this->comp);
      // Second push back
      this->c.push_back(std::move(dst));
      std::push_heap(this->c.begin(), this->c.end(), this->comp);
      return true;
    } else {
      return false;
    }
  }

  // get and remove the index _Tp by swh 20230427
  // Gets and deletes the element of the specified index 0 to size-1
  bool get_and_remove(unsigned int index, _Tp* value) {
    unsigned int e_size = this->c.size();
    if (index >= e_size) {
      std::cout << "index: " << index << " >= size: " << e_size << "\n";
      return false;
    } else {
      std::vector<_Tp> tempQ;
      unsigned int i = 0;
      _Tp t;
      // First pop out
      while (i < index) {
        tempQ.push_back(this->c.front());
        std::pop_heap(this->c.begin(), this->c.end(), this->comp);
        this->c.pop_back();
        i++;
      }
      // Second get element
      value = &this->c.front();
      std::pop_heap(this->c.begin(), this->c.end(), this->comp);
      this->c.pop_back();
      // Third push back
      i = 0;
      while (i < index) {
        t = tempQ[i++];
        this->c.push_back(std::move(t));
        std::push_heap(this->c.begin(), this->c.end(), this->comp);
      }
      return true;
    }
  }

  // get the index _Tp by swh 20230427
  // Gets the element of the specified index
  bool get(unsigned int index, _Tp* value) {
    unsigned int e_size = this->c.size();
    if (index >= e_size) {
      std::cout << "index: " << index << " >= size: " << e_size << "\n";
      return false;
    } else {
      std::vector<_Tp> tempQ;
      unsigned int i = 0;
      _Tp t;
      // First pop out
      while (i < index) {
        tempQ.push_back(this->c.front());
        std::pop_heap(this->c.begin(), this->c.end(), this->comp);
        this->c.pop_back();
        i++;
      }
      // Second get element
      value = &this->c.front();
      // Third push back
      i = 0;
      while (i < index) {
        t = tempQ[i++];
        this->c.push_back(std::move(t));
        std::push_heap(this->c.begin(), this->c.end(), this->comp);
      }
      return true;
    }
  }
};

}  // namespace planning
}  // namespace century
