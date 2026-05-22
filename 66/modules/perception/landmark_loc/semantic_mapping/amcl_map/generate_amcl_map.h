#pragma once
#include "semantic_mapping/map_param.h"
#include "semantic_mapping/object_occupancy_map.h"
#include "amcl/include/amcl/map/map.h"
#include <array>
namespace semantic_mapping {

class AmclMapGenerator{

  public:
   AmclMapGenerator() = default;
    ~AmclMapGenerator() {
      for(auto& map : map_layers_) {
        if (map != nullptr) {
          if (map->cells != nullptr) {
            free(map->cells);
            map->cells = nullptr;
          }
          free(map);
          map = nullptr;
        }
      }
    }
    explicit AmclMapGenerator(const MapParam para, const Eigen::Vector3d& origin,  const Eigen::Vector3d& base, const std::string& map_name)
        : origin_(origin), base_(base), para_(para), map_name_(map_name) {
      for(auto& map : map_layers_) {
        map = (map_t*) malloc(sizeof(map_t));
        map->origin_x = origin_(0);
        map->origin_y = origin_(1);
        map->scale = para.resolution_;
        map->size_x = para.grid_size_[0];
        map->size_y = para.grid_size_[1];
        map->cells = (map_cell_t*)calloc(map->size_x * map->size_y, sizeof(map->cells[0]));
       
      }
    }
    bool AmclMapBuild(const std::vector<ObjectCell>& occupancy_grid, const bool need_save = true);

    const std::array<map_t*,MapLayerSize>& GetAmclMap() const;
   private:
    bool LoadOccupancyGrid(const std::vector<ObjectCell>& occupancy_grid);
    bool DoBuilding();
    const double max_occ_distance = 100.0;
    const double resolution_ = 0.1;
   
    Eigen::Vector3d origin_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d base_ = Eigen::Vector3d(0.0, 0.0, 0.0);
    MapParam para_;
    std::string map_name_;
    std::array<map_t *, MapLayerSize> map_layers_{nullptr};
};

}