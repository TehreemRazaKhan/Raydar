#ifndef RAYDAR_TYPES_HH_
#define RAYDAR_TYPES_HH_

#include <cstdint>
#include <string>

namespace raydar
{
  /// One detection from a single radar ray.
  struct RayHit
  {
    double azimuth;        // radians, relative to platform heading
    double range;          // meters to the hit point
    std::string targetId;  // optional: resolved model name if known
    uint64_t objectId;     // rendering object id (for debugging)
  };
}

#endif  // RAYDAR_TYPES_HH_
