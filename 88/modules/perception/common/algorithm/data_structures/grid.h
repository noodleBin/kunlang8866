#pragma once

#ifndef MODULES_PERCEPTION_COMMON_ALGORITHM_DATA_STRUCTURES_GRID_H_
#define MODULES_PERCEPTION_COMMON_ALGORITHM_DATA_STRUCTURES_GRID_H_

#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <iostream>


namespace century {
namespace perception {
namespace algorithm {

// 3d Point coord to grid coord 
template <typename T, size_t N>
class GridId {
 public:
  GridId(const T points[N], T grid_size) {
    for (size_t i = 0; i < N; ++i) {
      id_[i] = std::floor(points[i] / grid_size);
      center_[i] = (id_[i] + 0.5) * grid_size;
    }
  };

  size_t hash() const {
    size_t hash_v = 0;
    for (size_t i = 0; i < N; ++i) {
      hash_v ^= std::hash<T>{}(id_[i]);
    }
    return hash_v;
  }

  bool equals(const GridId& other) const {
    for (size_t i = 0; i < N; ++i) {
      if (id_[i] != other.id_[i]) {
        return false;
      }
    }
    return true;
  }

  T getCenter(size_t index) const {
    return center_[index];
  }

  T center_[N];
  int32_t id_[N];

};

using GridId3d = GridId<double, 3>;

struct GridIdHash {
  template <typename T, size_t N>
  size_t operator()(const GridId<T, N>& id) const {
    return id.hash();
  }
};

struct GridIdEqual {
  template <typename T, size_t N>
  bool operator()(const GridId<T, N>& id1, const GridId<T, N>& id2) const {
    return id1.equals(id2);
  }
};

template <typename T, typename V, size_t N>
using GridMapNd = std::unordered_map<GridId<T, N>, V, GridIdHash, GridIdEqual>;

using GridMap3d = GridMapNd<double, std::vector<double>, 3>;

template <typename T, typename V, size_t N>
class Container {
 public:
  Container(T grid_size) : grid_size_(grid_size) {}

  void emplace(const T points[N], V value) {
    GridId<T, N> id(points, grid_size_);
    auto iter = grid_map_.find(id);
    if (iter == grid_map_.end()) {
      std::vector<V> values;
      values.emplace_back(value);
      grid_map_.emplace(id, values);
    } else {
      iter->second.emplace_back(value);
    }
  }
  std::vector<V> operator[](GridId<T, N> index) {
    return grid_map_[index];
  }

  size_t size() {
    return grid_map_.size();
  }
  
  auto begin() {
    return grid_map_.begin();
  }
  auto end() {
    return grid_map_.end();
  }
 private:
  T grid_size_;
  GridMapNd<T, std::vector<V>, N> grid_map_;
};

template <typename V>
using Container3d = Container<double, V, 3>;


} // namespace algorithm
} // namespace perception
} // namespace century


#endif  // MODULES_PERCEPTION_COMMON_ALGORITHM_DATA_STRUCTURES_GRID_H_