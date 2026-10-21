#ifndef RAYDAR_RADARSYSTEM_HH_
#define RAYDAR_RADARSYSTEM_HH_

#include <raydar/RadarRayCaster.hh>
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <rclcpp/rclcpp.hpp>
#include <raydar/msg/radar_target_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <mutex>
#include <vector>

namespace raydar
{
  class RadarSystem
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPostUpdate
  {
    public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

    public: void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override;

    public: ~RadarSystem() override;

    private: void PublishResults(
      const std::vector<RayHit> &_hits,
      const gz::math::Pose3d &_wamvPose);

    private: int64_t lastScanSecond_{-1};
    private: double maxRange_{100.0};
    private: double fovHalf_{M_PI / 3.0};
    private: int numRays_{360};
    private: bool verbose_{false};

    private: rclcpp::Node::SharedPtr node_;
    private: rclcpp::Publisher<raydar::msg::RadarTargetArray>::SharedPtr pub_;
    private: rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_;

    private: RadarRayCaster rayCaster_;
    private: std::mutex hitsMutex_;
    private: std::vector<RayHit> latestHits_;
  };
}

#endif  // RAYDAR_RADARSYSTEM_HH_
