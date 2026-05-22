#include "generate_amcl_map.h"
#include "modules/perception/landmark_loc/proto/amcl_map.pb.h"
#include "cyber/common/file.h"
#include "amcl/include/amcl/map/map.h"
namespace semantic_mapping {


bool AmclMapGenerator::AmclMapBuild(const std::vector<ObjectCell>& occupancy_grid, const bool need_save ) {
  
  if(!LoadOccupancyGrid(occupancy_grid)) {
    return false;
  }
  if(!DoBuilding()) {
    return false;
  }

  if(!need_save) {
    return true;
  }

  
  century::perception::landmark_loc::AmclMap amcl_map;
  const std::string map_pb = "amcl_map.pb.bin";
  amcl_map.set_origin_x(origin_[0]);
  amcl_map.set_origin_y(origin_[1]);
  amcl_map.set_base_x(base_[0]);
  amcl_map.set_base_y(base_[1]);
  amcl_map.set_base_yaw(base_[2]);
  amcl_map.set_scale(para_.resolution_);
  amcl_map.set_width(para_.grid_size_[0]);
  amcl_map.set_height(para_.grid_size_[1]);
  amcl_map.set_max_occ_distance(max_occ_distance);

  for(uint32_t i=0; i < map_layers_.size(); ++i) {
    const auto& map = map_layers_.at(i);
    if (map == nullptr || map->cells == nullptr) {
      return false;
    }
    auto* map_layer = amcl_map.mutable_layer()->Add();
    map_layer->set_type(static_cast<century::perception::landmark_loc::SemanticType>(i));
    for(int j=0 ; j < occupancy_grid.size(); j++) {
      const auto* cell = &map->cells[j];
      if(!cell) {
        return false;
      }
      auto* map_cell = map_layer->mutable_cells()->Add();
      map_cell->set_occ_distance(cell->occ_dist);
      map_cell->set_occ_state(cell->occ_state);
    }
  }
  
  century::cyber::common::SetProtoToBinaryFile(amcl_map, map_name_ + "/" + map_pb);

  return true;
}

bool AmclMapGenerator::LoadOccupancyGrid(const std::vector<ObjectCell>& occupancy_grid) {
  for(uint32_t i=0; i < static_cast<uint32_t>(ObjectClass::ObjectSize)-1; ++i) {
    auto* map_layer = map_layers_.at(i);
    if(map_layer == nullptr || map_layer->cells == nullptr) {
      return false;
    }
    for(size_t j = 0; j < occupancy_grid.size(); j++) {
      auto* map_cell = &map_layer->cells[j];
      map_cell->occ_dist = max_occ_distance;
      if(static_cast<uint32_t>(occupancy_grid[j].get_occ_class()) == i) { 
        map_cell->occ_state = 1;
      } else {
        map_cell->occ_state = 0;
      }
    }
  }
  return true;
}

bool AmclMapGenerator::DoBuilding() {

  for(auto& map_layer : map_layers_) {
    if (map_layer == nullptr || map_layer->cells == nullptr) {
      return false;
    }
    map_update_cspace(map_layer, max_occ_distance);
    
  }
  return true;
}

const std::array<map_t*, MapLayerSize>& AmclMapGenerator::GetAmclMap() const {
  return map_layers_;
}

}