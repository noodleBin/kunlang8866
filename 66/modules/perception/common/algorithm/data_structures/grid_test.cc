#include "modules/perception/common/algorithm/data_structures/grid.h"

#include "gtest/gtest.h"
#include <string>

namespace century {
namespace perception {
namespace algorithm {

TEST(GridTest, GridTest) {
  double grid_size = 0.5;
  double points[3] = {10,10,10};
  double points2[3] = {20,10,10};
  GridId3d grid1(points, grid_size);
  GridId3d grid2(points2, grid_size);

  std::string type1 = "GridId3d";
  std::string type1_1 = "GridId3d1";
  std::string type2 = "GridId3d2";

  std::cout << grid1.hash() << std::endl;
  EXPECT_EQ(grid1.equals(grid2), false) << "grid1 and grid2 are equal";
  Container3d<std::string> container1(grid_size);
  container1.emplace(points, type1);
  container1.emplace(points2, type2);
  container1.emplace(points, type1_1);
  EXPECT_EQ(container1.size(), 2) << "size is not equal to 2 " << container1.size();
  EXPECT_EQ(container1[grid1][0], type1) << "type is not equal to type1";
  EXPECT_EQ(container1[grid1][1], type1_1) << "type is not equal to type1";
  EXPECT_EQ(container1[grid2][0], type2) << "type is not equal to type1";

}

} // namespace algorithm
} // namespace perception
}  // namespace century