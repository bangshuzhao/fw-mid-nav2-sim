#include <algorithm>
#include <cmath>
#include <limits>

#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mppi::critics
{

class PathDirectionCritic : public CriticFunction
{
public:
  void initialize() override
  {
    auto get_param = parameters_handler_->getParamGetter(name_);
    get_param(weight_, "cost_weight", 16.0);
  }

  void score(CriticData & data) override
  {
    if (!enabled_) {
      return;
    }

    const auto & path = data.path;
    for (std::size_t i = 1; i < path.x.size(); ++i) {
      const float dx = path.x(i) - path.x(0);
      const float dy = path.y(i) - path.y(0);
      if (std::hypot(dx, dy) < 0.01f) {
        continue;
      }

      // 路径位移投影到起点的车头方向，正值表示前进，负值表示倒车。
      const float forward = dx * std::cos(path.yaws(0)) + dy * std::sin(path.yaws(0));
      const float direction = std::copysign(1.0f, forward);
      const auto & velocities = data.state.vx;

      for (std::size_t row = 0; row < velocities.shape(0); ++row) {
        float wrong_way = 0.0f;
        float pose_error = 0.0f;

        for (std::size_t step = 0; step < velocities.shape(1); ++step) {
          wrong_way += std::max(0.0f, -direction * velocities(row, step));
        }

        // 每隔四个预测步检查一次位置和航向，减少最近路径点的搜索次数。
        for (std::size_t step = 3; step < velocities.shape(1); step += 4) {
          const float x = data.trajectories.x(row, step);
          const float y = data.trajectories.y(row, step);
          float best = std::numeric_limits<float>::infinity();
          std::size_t nearest = 0;

          for (std::size_t pt = 0; pt < path.x.size(); ++pt) {
            const float px = x - path.x(pt);
            const float py = y - path.y(pt);
            const float gap = px * px + py * py;
            if (gap < best) {
              best = gap;
              nearest = pt;
            }
          }

          const float yaw_gap = std::remainder(
            data.trajectories.yaws(row, step) - path.yaws(nearest),
            2.0f * static_cast<float>(M_PI));
          pose_error += std::sqrt(best) + 0.45f * std::abs(yaw_gap);
        }

        data.costs(row) += weight_ * data.model_dt * (wrong_way + 4.0f * pose_error);
      }
      return;
    }
  }

private:
  float weight_{0.0f};
};

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::PathDirectionCritic, mppi::critics::CriticFunction)