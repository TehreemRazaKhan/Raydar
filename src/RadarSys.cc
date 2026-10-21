#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Helpers.hh>
#include <gz/common/Console.hh>

#include <rclcpp/rclcpp.hpp>
#include <raydar/msg/radar_target.hpp>
#include <raydar/msg/radar_target_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <memory>
#include <string>

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
      gz::sim::EventManager &_eventMgr) override
    {
      maxRange_ = 100.0;
      fovHalf_  = 60.0 * M_PI / 180.0;

      // Optional SDF-configurable verbosity
      if (_sdf && _sdf->HasElement("verbose"))
        verbose_ = _sdf->Get<bool>("verbose");

      if (!rclcpp::ok())
        rclcpp::init(0, nullptr);

      node_ = std::make_shared<rclcpp::Node>("radar_system_plugin");

      // Raw radar data publisher
      pub_ = node_->create_publisher<raydar::msg::RadarTargetArray>(
        "/radar/targets", 10);

      // Visualization publisher
      markerPub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/radar/markers", 10);

      if (verbose_)
      {
        gzlog << "RadarSystem loaded. Publishing on /radar/targets and /radar/markers"
              << std::endl;
      }
    }

    public: void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override
    {
      if (_info.paused)
        return;

      rclcpp::spin_some(node_);

      auto simSeconds = std::chrono::duration_cast<std::chrono::seconds>(
        _info.simTime).count();
      if (simSeconds == lastPrintedSecond_)
        return;
      lastPrintedSecond_ = simSeconds;

      // --- 1. Find WAM-V ---
      bool foundWamv = false;
      gz::math::Pose3d wamvPose;
      gz::sim::Entity wamvEntity = gz::sim::kNullEntity;

      _ecm.Each<gz::sim::components::Model,
                gz::sim::components::Name,
                gz::sim::components::Pose>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Model *,
            const gz::sim::components::Name *_name,
            const gz::sim::components::Pose *_pose) -> bool
        {
          if (_name->Data() == "wamv")
          {
            wamvPose   = _pose->Data();
            wamvEntity = _entity;
            foundWamv  = true;
            return false;
          }
          return true;
        });

      if (!foundWamv)
        return;

      gz::math::Vector3d wamvVel{0, 0, 0};
      auto wamvVelComp =
        _ecm.Component<gz::sim::components::LinearVelocity>(wamvEntity);
      if (wamvVelComp)
        wamvVel = wamvVelComp->Data();

      double radarX   = wamvPose.Pos().X();
      double radarY   = wamvPose.Pos().Y();
      double radarYaw = wamvPose.Rot().Yaw();

      // --- 2. Build both messages ---
      raydar::msg::RadarTargetArray msg;
      msg.header.stamp    = node_->now();
      msg.header.frame_id = "wamv/base_link";
      msg.range_max       = static_cast<float>(maxRange_);
      msg.fov_half_angle  = static_cast<float>(fovHalf_);

      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker clear;
      clear.header = msg.header;
      clear.action = visualization_msgs::msg::Marker::DELETEALL;
      markers.markers.push_back(clear);

      int32_t markerId = 0;

      // --- 3. Scan ---
      _ecm.Each<gz::sim::components::Model,
                gz::sim::components::Name,
                gz::sim::components::Pose>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Model *,
            const gz::sim::components::Name *_name,
            const gz::sim::components::Pose *_pose) -> bool
        {
          const std::string &n = _name->Data();
          if (n == "wamv" || n == "sydneyregatta" || n == "Coast Waves")
            return true;

          double tx = _pose->Data().Pos().X();
          double ty = _pose->Data().Pos().Y();
          double dx = tx - radarX;
          double dy = ty - radarY;
          double range = std::sqrt(dx * dx + dy * dy);

          if (range > maxRange_ || range < 1e-6)
            return true;

          double worldAngle = std::atan2(dy, dx);
          double azimuth = worldAngle - radarYaw;
          while (azimuth >  M_PI) azimuth -= 2.0 * M_PI;
          while (azimuth < -M_PI) azimuth += 2.0 * M_PI;

          if (std::abs(azimuth) > fovHalf_)
            return true;

          // Doppler
          gz::math::Vector3d los(dx, dy, 0.0);
          los.Normalize();

          gz::math::Vector3d targetVel{0, 0, 0};
          auto velComp =
            _ecm.Component<gz::sim::components::LinearVelocity>(_entity);
          if (velComp)
            targetVel = velComp->Data();

          double radialVelocity = (targetVel - wamvVel).Dot(los);

          // --- Raw target message ---
          raydar::msg::RadarTarget target;
          target.header          = msg.header;
          target.target_id       = n;
          target.range           = static_cast<float>(range);
          target.azimuth         = static_cast<float>(azimuth);
          target.elevation       = 0.0f;
          target.radial_velocity = static_cast<float>(radialVelocity);
          target.rcs             = 1.0f;
          msg.targets.push_back(target);

          // --- Marker: sphere ---
          // Radar is 2D (range + azimuth), so plot on the x-y plane of
          // the wamv/base_link frame. z = 0 keeps them at the waterline.
          double mx = range * std::cos(azimuth);
          double my = range * std::sin(azimuth);
          double mz = 0.0;

          visualization_msgs::msg::Marker sphere;
          sphere.header          = msg.header;
          sphere.ns              = "radar_targets";
          sphere.id              = markerId++;
          sphere.type            = visualization_msgs::msg::Marker::SPHERE;
          sphere.action          = visualization_msgs::msg::Marker::ADD;
          sphere.pose.position.x = mx;
          sphere.pose.position.y = my;
          sphere.pose.position.z = mz;
          sphere.pose.orientation.w = 1.0;
          sphere.scale.x = 1.0;
          sphere.scale.y = 1.0;
          sphere.scale.z = 1.0;

          float ratio = static_cast<float>(range / maxRange_);
          sphere.color.r = ratio;
          sphere.color.g = 1.0f - ratio;
          sphere.color.b = 0.0f;
          sphere.color.a = 0.8f;
          sphere.lifetime.sec = 2;
          markers.markers.push_back(sphere);

          // --- Marker: text label ---
          visualization_msgs::msg::Marker label;
          label.header          = msg.header;
          label.ns              = "radar_labels";
          label.id              = markerId++;
          label.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          label.action          = visualization_msgs::msg::Marker::ADD;
          label.pose.position.x = mx;
          label.pose.position.y = my;
          label.pose.position.z = mz + 2.0;
          label.pose.orientation.w = 1.0;
          label.scale.z         = 1.5;
          label.color.r = 1.0f;
          label.color.g = 1.0f;
          label.color.b = 1.0f;
          label.color.a = 1.0f;
          label.text = n + "\n" + std::to_string(range) + "m";
          label.lifetime.sec = 2;
          markers.markers.push_back(label);

          if (verbose_)
          {
            gzlog << "  " << n
                  << " range=" << range
                  << " azimuth=" << azimuth * 180.0 / M_PI
                  << " v_radial=" << radialVelocity << std::endl;
          }
          return true;
        });

      // --- 4. Publish both ---
      pub_->publish(msg);
      markerPub_->publish(markers);
    }

    private: int64_t lastPrintedSecond_{-1};
    private: double maxRange_{100.0};
    private: double fovHalf_{M_PI / 3.0};
    private: bool verbose_{false};

    private: rclcpp::Node::SharedPtr node_;
    private: rclcpp::Publisher<raydar::msg::RadarTargetArray>::SharedPtr pub_;
    private: rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPub_;
  };
}

GZ_ADD_PLUGIN(
  raydar::RadarSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(raydar::RadarSystem, "raydar::RadarSystem")
