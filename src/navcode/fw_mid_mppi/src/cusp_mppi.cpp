#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fw_mid_mppi
{

class CuspMPPI : public nav2_mppi_controller::MPPIController
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    MPPIController::configure(parent, name, tf, costmap_ros);
    auto node = parent.lock();
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".cusp_hold_time", rclcpp::ParameterValue(7.0));
    hold_time_ = node->get_parameter(name + ".cusp_hold_time").as_double();
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".cusp_departure_distance", rclcpp::ParameterValue(0.0));
    departure_distance_ = node->get_parameter(name + ".cusp_departure_distance").as_double();
    nav2_util::declare_parameter_if_not_declared(
      node, name + ".stall_assistance", rclcpp::ParameterValue(false));
    stall_assistance_ = node->get_parameter(name + ".stall_assistance").as_bool();
    if (departure_distance_ > 0.0 || stall_assistance_) {
      minimum_radius_ = node->get_parameter(name + ".AckermannConstraints.min_turning_r").as_double();
      forward_limit_ = node->get_parameter(name + ".vx_max").as_double();
      reverse_limit_ = std::abs(node->get_parameter(name + ".vx_min").as_double());
    }
  }

  void setPlan(const nav_msgs::msg::Path & plan) override
  {
    pending_ = plan;
  }

  void setSpeedLimit(const double & limit, const bool & percentage) override
  {
    MPPIController::setSpeedLimit(limit, percentage);
    speed_limit_.store(percentage ? -limit : limit, std::memory_order_relaxed);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_speed, nav2_core::GoalChecker * goal_checker) override
  {
    if (pending_) {
      const auto now = clock_->now();
      if (acceptPlan(robot_pose, now)) {
        MPPIController::setPlan(*pending_);
        departure_.reset();
        low_speed_since_.reset();
        assist_until_.reset();
        active_tail_ = pending_->poses.back();
        accepted_at_ = now;
      }
      pending_.reset();
    }
    if (!stall_assistance_) {
      return computeLegacyCommand(robot_pose, robot_speed, goal_checker);
    }
    const auto now = clock_->now();
    if ((now - last_time_called_).seconds() > reset_period_) {
      reset();
      low_speed_since_.reset();
      assist_until_.reset();
    }
    last_time_called_ = now;
    std::lock_guard<std::mutex> parameter_lock(*parameters_handler_->getLock());
    const auto poses_before = path_handler_.getPath().poses.size();
    auto reference = path_handler_.transformPath(robot_pose);
    geometry_msgs::msg::TwistStamped command;
    command.header = robot_pose.header;
    if (path_handler_.getPath().poses.size() < poses_before) {
      beginReversal(robot_pose);
      return command;
    }
    std::optional<geometry_msgs::msg::TwistStamped> departure_command;
    if (departure_) {
      departure_command = departReversal(robot_pose, robot_speed, reference);
      if (!departure_command) {
        departure_.reset();
      } else if (distance(*departure_, robot_pose.pose.position) < 0.5 * departure_distance_) {
        return *departure_command;
      }
    }
    auto & active = path_handler_.getPath();
    const auto inversion = mppi::utils::findFirstPathInversion(active);
    const auto & end = reference.poses.back().pose;
    bool approaching_cusp = false;
    double incoming_direction = 0.0;
    if (inversion > 0 && inversion < active.poses.size()) {
      auto cusp = active.poses[inversion - 1];
      cusp.header = active.header;
      cusp.header.stamp = robot_pose.header.stamp;
      if (tf_buffer_->canTransform(reference.header.frame_id, cusp.header.frame_id,
        cusp.header.stamp, rclcpp::Duration::from_seconds(0.0)))
      {
        const auto local = tf_buffer_->transform(cusp, reference.header.frame_id);
        approaching_cusp = distance(end.position, local.pose.position) < 0.035;
        if (approaching_cusp && inversion >= 2) {
          const auto & previous = active.poses[inversion - 2].pose;
          const double heading = yaw(previous.orientation);
          const double dx = cusp.pose.position.x - previous.position.x;
          const double dy = cusp.pose.position.y - previous.position.y;
          incoming_direction = std::copysign(1.0, dx * std::cos(heading) + dy * std::sin(heading));
        }
      }
    }
    const double remaining = distance(robot_pose.pose.position, end.position);
    if (approaching_cusp && !departure_command && remaining < 0.90) {
      // Use a consistent geometric reference on the final approach to a reversal.
      // A sample-based optimizer can alternate curvature signs as its horizon shrinks.
      const double speed = std::clamp(0.50 * remaining, 0.06, 0.17);
      if (auto approach = trackingArc(robot_pose, reference, speed, incoming_direction)) {
        if (visualize_) {
          visualize(std::move(reference));
        }
        return *approach;
      }
    }
    auto * map = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
    const auto optimized = optimizer_.evalFeasibleControl(
      robot_pose, robot_speed, reference, goal_checker);
    if (!optimized) {
      if (departure_command) {
        return *departure_command;
      }
      // Check a short arc when the full prediction horizon is blocked.
      if (auto slow = trackingArc(robot_pose, reference, 0.10)) {
        return *slow;
      }
      throw nav2_core::PlannerException("No collision-free control or short reference arc");
    }
    command = *optimized;
    if (departure_command) {
      // Keep the MPC warm while the checked departure command finishes the handoff.
      return *departure_command;
    }
    const double direction = referenceDirection(reference);
    if (direction * command.twist.linear.x < -0.015 && reference.poses.size() > 2) {
      // A cropped path contains a single driving direction. Reject an unplanned
      // reversal, which otherwise unloads and reloads the wheels at the bend.
      if (auto slow = trackingArc(robot_pose, reference, 0.10)) {
        return *slow;
      }
      command.twist = geometry_msgs::msg::Twist();
    }
    if (std::abs(command.twist.linear.x) < 0.040 && std::abs(robot_speed.linear.x) < 0.025) {
      if (!low_speed_since_) {
        low_speed_since_ = now;
      }
    } else {
      low_speed_since_.reset();
    }
    const bool need_assistance = low_speed_since_ && (now - *low_speed_since_).seconds() > 0.55;
    const bool assisting = assist_until_ && now < *assist_until_;
    if ((need_assistance || assisting) && reference.poses.size() > 2 &&
      (distance(robot_pose.pose.position, end.position) > 0.18 ||
      std::abs(std::remainder(yaw(robot_pose.pose.orientation) - yaw(end.orientation),
      2.0 * M_PI)) > 0.15))
    {
      if (auto slow = trackingArc(robot_pose, reference, 0.10)) {
        if (!assisting) {
          assist_until_ = now + rclcpp::Duration::from_seconds(0.85);
          RCLCPP_DEBUG(logger_, "Using a checked low-speed arc to progress along the reference");
        }
        command = *slow;
      }
    }
    if (approaching_cusp) {
      // Preserve the curvature while reducing speed on the remaining braking distance.
      const double limit = std::clamp(0.75 * remaining, 0.045, 0.19);
      if (std::abs(command.twist.linear.x) > limit) {
        const double scale = limit / std::abs(command.twist.linear.x);
        command.twist.linear.x *= scale;
        command.twist.angular.z *= scale;
      }
    }
    if (visualize_) {
      visualize(std::move(reference));
    }
    return command;
  }

private:
  geometry_msgs::msg::TwistStamped computeLegacyCommand(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * goal_checker)
  {
    const auto poses_before = path_handler_.getPath().poses.size();
    auto command = MPPIController::computeVelocityCommands(pose, speed, goal_checker);
    if (departure_distance_ > 0.0 && path_handler_.getPath().poses.size() < poses_before) {
      beginReversal(pose);
      command.twist = geometry_msgs::msg::Twist();
    }
    return command;
  }

  void beginReversal(const geometry_msgs::msg::PoseStamped & pose)
  {
    optimizer_.reset();
    departure_ = pose.pose.position;
    departure_started_ = clock_->now();
    waiting_to_reverse_ = true;
    low_speed_since_.reset();
    assist_until_.reset();
  }

  std::optional<geometry_msgs::msg::TwistStamped> departReversal(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    const nav_msgs::msg::Path & path)
  {
    if (distance(*departure_, pose.pose.position) > departure_distance_ ||
      (clock_->now() - departure_started_).seconds() > 6.0)
    {
      return std::nullopt;
    }
    if (waiting_to_reverse_ && std::abs(speed.linear.x) > 0.025) {
      geometry_msgs::msg::TwistStamped command;
      command.header = pose.header;
      return command;
    }
    waiting_to_reverse_ = false;
    return trackingArc(pose, path, 0.13);
  }

  std::optional<geometry_msgs::msg::TwistStamped> trackingArc(
    const geometry_msgs::msg::PoseStamped & pose, const nav_msgs::msg::Path & path,
    double nominal_speed, double terminal_direction = 0.0)
  {
    const auto & points = path.poses;
    if (points.empty() || (points.size() < 3 && terminal_direction == 0.0)) {
      return std::nullopt;
    }
    const double direction = terminal_direction != 0.0 ? terminal_direction : referenceDirection(path);
    if (direction == 0.0) {
      return std::nullopt;
    }
    double limit = direction > 0.0 ? forward_limit_ : reverse_limit_;
    const double requested_limit = speed_limit_.load(std::memory_order_relaxed);
    if (requested_limit > 0.0) {
      limit = std::min(limit, requested_limit);
    } else if (requested_limit < 0.0) {
      limit *= -requested_limit / 100.0;
    }
    double length = 0.0;
    std::size_t index = 0;
    while (index + 1 < points.size() && length < 0.22) {
      length += distance(points[index].pose.position, points[index + 1].pose.position);
      ++index;
    }
    if (length < 0.10 && terminal_direction == 0.0) {
      return std::nullopt;
    }
    const double angle = yaw(pose.pose.orientation);
    const auto & position = pose.pose.position;
    const double travel_yaw = angle + (direction < 0.0 ? M_PI : 0.0);
    auto target = points[index].pose.position;
    if (terminal_direction != 0.0) {
      // Intersect the upcoming polyline with the lookahead circle. Choosing the
      // next sampled pose jumps the steering target by a full path cell.
      constexpr double lookahead = 0.22;
      if (distance(position, points.front().pose.position) <= lookahead) {
        target = points.back().pose.position;
        for (std::size_t i = 1; i < points.size(); ++i) {
          const auto & next = points[i].pose.position;
          if (distance(position, next) <= lookahead) {
            continue;
          }
          const auto & prev = points[i - 1].pose.position;
          const double sx = next.x - prev.x, sy = next.y - prev.y;
          const double px = prev.x - position.x, py = prev.y - position.y;
          const double norm = sx * sx + sy * sy;
          const double dot = px * sx + py * sy;
          const double d = dot * dot + norm * (lookahead * lookahead - px * px - py * py);
          const double u = std::clamp((-dot + std::sqrt(std::max(0.0, d))) / norm, 0.0, 1.0);
          target.x = prev.x + u * sx;
          target.y = prev.y + u * sy;
          break;
        }
      }
    }
    const double dx = target.x - position.x, dy = target.y - position.y;
    const double forward = std::cos(travel_yaw) * dx + std::sin(travel_yaw) * dy;
    if (forward < (terminal_direction == 0.0 ? 0.06 : 0.022) || std::hypot(dx, dy) > 0.42) {
      return std::nullopt;
    }
    const double lateral = -std::sin(travel_yaw) * dx + std::cos(travel_yaw) * dy;
    double curvature = 2.0 * lateral / (dx * dx + dy * dy);
    const auto & end = points.back().pose;
    const double remaining = distance(position, end.position);
    if (terminal_direction != 0.0 && remaining < 0.32) {
      // Pure pursuit can reach the cusp position with the wheels pointing along
      // the wrong arc. Reserve enough remaining travel to reach its heading.
      const double heading_error = std::remainder(yaw(end.orientation) - angle, 2.0 * M_PI);
      constexpr double heading_reserve = 0.045;
      if (std::abs(heading_error) > heading_reserve) {
        const double required = (std::abs(heading_error) - heading_reserve) / std::max(remaining, 0.04);
        const double sign = std::copysign(1.0, heading_error);
        const double blend = std::clamp((0.32 - remaining) / 0.10, 0.0, 1.0);
        curvature += blend * sign * std::max(0.0, required - sign * curvature);
      }
    }
    curvature = std::clamp(curvature, -1.0 / minimum_radius_, 1.0 / minimum_radius_);
    auto * map = costmap_ros_->getCostmap();
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
    const auto footprint = costmap_ros_->getRobotFootprint();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    const auto arc_length = std::min(0.16, 0.8 * forward);
    const int samples = std::max(1, static_cast<int>(std::ceil(arc_length / 0.015)));
    for (int i = 0; i <= samples; ++i) {
      const double s = arc_length * i / samples;
      const double heading = angle + curvature * s;
      const double chord = std::abs(curvature) < 1e-6 ? s :
        2.0 * std::sin(curvature * s / 2.0) / curvature;
      const double middle = (angle + heading) / 2.0;
      const double x = position.x + direction * chord * std::cos(middle);
      const double y = position.y + direction * chord * std::sin(middle);
      unsigned int mx, my;
      if (!map->worldToMap(x, y, mx, my) ||
        checker.footprintCostAtPose(x, y, heading, footprint) >= nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        return std::nullopt;
      }
      std::vector<nav2_costmap_2d::MapLocation> outline;
      const double cos_heading = std::cos(heading), sin_heading = std::sin(heading);
      for (const auto & point : footprint) {
        if (!map->worldToMap(x + cos_heading * point.x - sin_heading * point.y,
          y + sin_heading * point.x + cos_heading * point.y, mx, my))
        {
          return std::nullopt;
        }
        outline.push_back({mx, my});
      }
      std::vector<nav2_costmap_2d::MapLocation> cells;
      map->convexFillCells(outline, cells);
      for (const auto & cell : cells) {
        if (map->getCost(cell.x, cell.y) >= nav2_costmap_2d::LETHAL_OBSTACLE) {
          return std::nullopt;
        }
      }
    }
    const double speed = std::min(nominal_speed, limit);
    geometry_msgs::msg::TwistStamped command;
    command.header = pose.header;
    command.twist.linear.x = direction * speed;
    command.twist.angular.z = speed * curvature;
    return command;
  }

  static double referenceDirection(const nav_msgs::msg::Path & path)
  {
    if (path.poses.empty()) {
      return 0.0;
    }
    const auto & start = path.poses.front().pose;
    const double heading = yaw(start.orientation);
    for (const auto & stamped : path.poses) {
      const double dx = stamped.pose.position.x - start.position.x;
      const double dy = stamped.pose.position.y - start.position.y;
      if (std::hypot(dx, dy) >= 0.012) {
        return std::copysign(1.0, dx * std::cos(heading) + dy * std::sin(heading));
      }
    }
    return 0.0;
  }

  static double yaw(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  bool acceptPlan(const geometry_msgs::msg::PoseStamped & pose, const rclcpp::Time & now)
  {
    if (pending_->poses.empty()) {
      throw std::runtime_error("FW-mid MPPI received an empty path");
    }
    if (!active_tail_ || (now - last_time_called_).seconds() > reset_period_ ||
      distance(pending_->poses.back().pose.position, active_tail_->pose.position) > 0.08 ||
      std::abs(std::remainder(yaw(pending_->poses.back().pose.orientation) -
      yaw(active_tail_->pose.orientation), 2.0 * M_PI)) > 0.12 ||
      (now - accepted_at_).seconds() >= hold_time_)
    {
      return true;
    }

    // PathHandler keeps this path and removes its completed reversal segments in place.
    auto & active = path_handler_.getPath();
    const auto & points = active.poses;
    const auto inversion = mppi::utils::findFirstPathInversion(active);
    if (inversion == 0 || inversion + 1 >= points.size()) {
      return true;
    }
    const auto timeout = rclcpp::Duration::from_seconds(0.05);
    if (!tf_buffer_->canTransform(pose.header.frame_id, active.header.frame_id,
      pose.header.stamp, timeout))
    {
      return true;
    }
    const auto transform = tf_buffer_->lookupTransform(
      pose.header.frame_id, active.header.frame_id, pose.header.stamp);
    std::vector<geometry_msgs::msg::PoseStamped> local_points(inversion + 1);
    for (std::size_t i = 0; i <= inversion; ++i) {
      tf2::doTransform(points[i], local_points[i], transform);
    }
    const auto & cusp = local_points[inversion - 1].pose.position;
    if (distance(cusp, pose.pose.position) > 0.75) {
      return true;
    }
    std::size_t closest = 0;
    double closest_distance = 1.0;
    for (std::size_t i = 0; i < inversion; ++i) {
      const auto gap = distance(local_points[i].pose.position, pose.pose.position);
      if (gap < closest_distance) {
        closest = i;
        closest_distance = gap;
      }
    }
    if (closest_distance > 0.22) {
      return true;
    }
    // Release the retained path as soon as a new obstacle blocks this manoeuvre.
    auto * costmap = costmap_ros_->getCostmap();
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap->getMutex());
    for (std::size_t i = closest; i <= inversion; ++i) {
      const auto & local = local_points[i];
      const auto & p = local.pose.position;
      if (checker.footprintCostAtPose(p.x, p.y, yaw(local.pose.orientation),
        costmap_ros_->getRobotFootprint()) >= nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        return true;
      }
    }
    RCLCPP_DEBUG(logger_, "Retaining a collision-free FW-mid reversal during replanning");
    return false;
  }

  std::optional<nav_msgs::msg::Path> pending_;
  std::optional<geometry_msgs::msg::PoseStamped> active_tail_;
  rclcpp::Time accepted_at_{0, 0, RCL_ROS_TIME};
  double hold_time_{7.0};
  double departure_distance_{0.0};
  double minimum_radius_{0.45};
  double forward_limit_{0.0}, reverse_limit_{0.0};
  // Negative values encode percentages in a single atomic snapshot.
  std::atomic<double> speed_limit_{0.0};
  std::optional<geometry_msgs::msg::Point> departure_;
  rclcpp::Time departure_started_{0, 0, RCL_ROS_TIME};
  bool waiting_to_reverse_{false};
  bool stall_assistance_{false};
  std::optional<rclcpp::Time> low_speed_since_, assist_until_;
};

}  // namespace fw_mid_mppi

PLUGINLIB_EXPORT_CLASS(fw_mid_mppi::CuspMPPI, nav2_core::Controller)
