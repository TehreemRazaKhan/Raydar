#ifndef RAYDAR_RADARRAYCASTER_HH_
#define RAYDAR_RADARRAYCASTER_HH_

#include <raydar/Types.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/Scene.hh>
#include <gz/math/Vector3.hh>
#include <gz/rendering/Camera.hh>
#include <vector>

namespace raydar
{
  /// Casts rays into a Gazebo rendering scene to find surface intersections.
  /// MUST be used on the rendering thread (or with appropriate locking).
  class RadarRayCaster
  {
    public: RadarRayCaster();
    public: ~RadarRayCaster();

    /// Attach to a rendering scene. Call after the scene is ready.
    public: bool Initialize(gz::rendering::ScenePtr _scene);

    /// Cast a fan of rays from the given world-frame origin, in the plane
    /// defined by _yaw.
    /// - _origin: radar position in world frame
    /// - _yaw: platform heading in world frame (radians)
    /// - _fovHalfAngle: half of the horizontal field of view (radians)
    /// - _maxRange: maximum detection distance (meters)
    /// - _numRays: number of rays to cast across the FOV
    public: std::vector<RayHit> Scan(
      const gz::math::Vector3d &_origin,
      double _yaw,
      double _fovHalfAngle,
      double _maxRange,
      int _numRays);

    public: bool IsReady() const { return rayQuery_ != nullptr; }

    private: gz::rendering::ScenePtr scene_;
    private: gz::rendering::RayQueryPtr rayQuery_;
  };
}

#endif  // RAYDAR_RADARRAYCASTER_HH_
