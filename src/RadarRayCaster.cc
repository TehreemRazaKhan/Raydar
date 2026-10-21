#include <raydar/RadarRayCaster.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/common/Console.hh>

#include <cmath>

namespace raydar
{
  RadarRayCaster::RadarRayCaster() = default;
  RadarRayCaster::~RadarRayCaster() = default;

  bool RadarRayCaster::Initialize(gz::rendering::ScenePtr _scene)
  {
    if (!_scene)
    {
      gzerr << "RadarRayCaster: null scene" << std::endl;
      return false;
    }
    scene_ = _scene;

    rayQuery_ = scene_->CreateRayQuery();
    if (!rayQuery_)
    {
      gzerr << "RadarRayCaster: failed to create RayQuery" << std::endl;
      return false;
    }

    gzmsg << "RadarRayCaster: initialized" << std::endl;
    return true;
  }

  std::vector<RayHit> RadarRayCaster::Scan(
    const gz::math::Vector3d &_origin,
    double _yaw,
    double _fovHalfAngle,
    double _maxRange,
    int _numRays)
  {
    std::vector<RayHit> hits;
    if (!rayQuery_) return hits;

    gz::math::Vector3d origin = _origin;
    origin.Z(origin.Z() + 1.0);

    double startAngle = _yaw - _fovHalfAngle;
    double step = (_numRays > 1)
      ? (2.0 * _fovHalfAngle) / static_cast<double>(_numRays - 1)
      : 0.0;

    for (int i = 0; i < _numRays; ++i)
    {
      double angle = startAngle + i * step;

      rayQuery_->SetOrigin(origin);
      rayQuery_->SetDirection(
        gz::math::Vector3d(std::cos(angle), std::sin(angle), 0.0));

      auto result = rayQuery_->ClosestPoint();
      if (!result) continue;
      if (result.distance <= 0.0 || result.distance > _maxRange) continue;

      RayHit hit;
      hit.azimuth  = angle - _yaw;
      hit.range    = result.distance;
      hit.objectId = result.objectId;
      hit.targetId = "";
      hits.push_back(hit);
    }

    return hits;
  }
}
