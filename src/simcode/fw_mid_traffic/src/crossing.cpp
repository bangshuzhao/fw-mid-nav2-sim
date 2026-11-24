#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/World.hh>
#include <ignition/msgs/odometry.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/Element.hh>

namespace fw_mid
{
class Crossings : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate,
  public ignition::gazebo::ISystemPostUpdate
{
  using Entity = ignition::gazebo::Entity;
  using ECM = ignition::gazebo::EntityComponentManager;
  using UpdateInfo = ignition::gazebo::UpdateInfo;

  struct Walker
  {
    std::string name;
    Entity entity{ignition::gazebo::kNullEntity};
    double x, low, high, lane, speed, trigger;
    double y{0.0}, destination{0.0};
    bool armed{true};
    ignition::transport::Node::Publisher truth;
  };

public:
  void Configure(const Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    ECM &, ignition::gazebo::EventManager &) override
  {
    world = ignition::gazebo::World(entity);
    for (auto entry = sdf->FindElement("walker"); entry; entry = entry->GetNextElement("walker")) {
      Walker walker;
      walker.name = entry->Get<std::string>("model");
      walker.x = entry->Get<double>("x");
      walker.low = entry->Get<double>("low_y");
      walker.high = entry->Get<double>("high_y");
      walker.lane = entry->Get<double>("lane_y");
      walker.speed = entry->Get<double>("speed");
      walker.trigger = entry->Get<double>("trigger_distance");
      if (!(walker.low < walker.lane && walker.lane < walker.high &&
        walker.speed > 0.0 && walker.trigger > 0.0))
      {
        throw std::invalid_argument("Invalid pedestrian crossing: " + walker.name);
      }
      walker.y = walker.destination = walker.low;
      walker.truth = transport.Advertise<ignition::msgs::Odometry>(
        "/traffic/" + walker.name + "/truth");
      walkers.push_back(std::move(walker));
    }
  }

  void PreUpdate(const UpdateInfo & info, ECM & ecm) override
  {
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (info.paused || dt <= 0.0) {
      return;
    }
    if (robot == ignition::gazebo::kNullEntity) {
      robot = world.ModelByName(ecm, "fw_mid");
      if (robot == ignition::gazebo::kNullEntity) {
        return;
      }
      previous_x = ignition::gazebo::worldPose(robot, ecm).X();
    }
    const auto position = ignition::gazebo::worldPose(robot, ecm).Pos();
    const double vx = (position.X() - previous_x) / dt;
    previous_x = position.X();
    for (auto & walker : walkers) {
      if (walker.entity == ignition::gazebo::kNullEntity) {
        walker.entity = world.ModelByName(ecm, walker.name);
        if (walker.entity == ignition::gazebo::kNullEntity) {
          continue;
        }
      }
      const double dx = walker.x - position.X();
      const double dy = walker.lane - position.Y();
      // Ground truth only starts the simulated encounter. It is never sent to Nav2.
      if (walker.y == walker.destination && !walker.armed &&
        (std::abs(dx) > walker.trigger + 0.65 || std::abs(dy) > 1.1))
      {
        walker.armed = true;
      }
      if (walker.armed && std::abs(dy) < 0.46 && std::abs(dx) <= walker.trigger &&
        std::abs(dx) > 0.45 && (dx > 0.0 ? vx : -vx) > 0.12)
      {
        walker.destination = walker.y == walker.low ? walker.high : walker.low;
        walker.armed = false;
      }
      if (walker.y != walker.destination) {
        const double delta = walker.destination - walker.y;
        const double heading = delta > 0.0 ? IGN_PI / 2.0 : -IGN_PI / 2.0;
        walker.y += std::copysign(std::min(std::abs(delta), dt * walker.speed), delta);
        if (std::abs(walker.destination - walker.y) < 1e-9) {
          walker.y = walker.destination;
        }
        ignition::gazebo::Model(walker.entity).SetWorldPoseCmd(
          ecm, {walker.x, walker.y, 0.0, 0.0, 0.0, heading});
      }
    }
  }

  void PostUpdate(const UpdateInfo & info, const ECM & ecm) override
  {
    if (info.paused || info.simTime < next_truth) {
      return;
    }
    next_truth = info.simTime + std::chrono::milliseconds(20);
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    for (auto & walker : walkers) {
      if (walker.entity == ignition::gazebo::kNullEntity) {
        continue;
      }
      const auto pose = ignition::gazebo::worldPose(walker.entity, ecm);
      ignition::msgs::Odometry message;
      auto header = message.mutable_header();
      header->mutable_stamp()->set_sec(ns / 1000000000);
      header->mutable_stamp()->set_nsec(ns % 1000000000);
      auto frame = header->add_data();
      frame->set_key("frame_id");
      frame->add_value("world");
      message.mutable_pose()->mutable_position()->set_x(pose.X());
      message.mutable_pose()->mutable_position()->set_y(pose.Y());
      message.mutable_pose()->mutable_orientation()->set_z(pose.Rot().Z());
      message.mutable_pose()->mutable_orientation()->set_w(pose.Rot().W());
      walker.truth.Publish(message);
    }
  }

private:
  ignition::gazebo::World world{ignition::gazebo::kNullEntity};
  Entity robot{ignition::gazebo::kNullEntity};
  double previous_x{0.0};
  std::chrono::steady_clock::duration next_truth{};
  ignition::transport::Node transport;
  std::vector<Walker> walkers;
};
}

IGNITION_ADD_PLUGIN(fw_mid::Crossings, ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure, ignition::gazebo::ISystemPreUpdate,
  ignition::gazebo::ISystemPostUpdate)
