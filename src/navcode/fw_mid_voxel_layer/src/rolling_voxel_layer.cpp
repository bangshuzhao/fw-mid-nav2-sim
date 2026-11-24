#include <mutex>

#include "nav2_costmap_2d/voxel_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace fw_mid
{
class RollingVoxelLayer : public nav2_costmap_2d::VoxelLayer
{
public:
  void updateBounds(
    double x, double y, double yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    if (!enabled_) {
      return;
    }

    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*getMutex());

    // 用观测缓存里的有效点云重新填充这一层。
    resetMaps();
    VoxelLayer::updateBounds(x, y, yaw, min_x, min_y, max_x, max_y);

    // 把已经消失的障碍也纳入主代价图的更新范围。
    touch(getOriginX(), getOriginY(), min_x, min_y, max_x, max_y);
    touch(getOriginX() + getSizeInMetersX(), getOriginY() + getSizeInMetersY(),
      min_x, min_y, max_x, max_y);
  }
};
}  // namespace fw_mid

PLUGINLIB_EXPORT_CLASS(fw_mid::RollingVoxelLayer, nav2_costmap_2d::Layer)