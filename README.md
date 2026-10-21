# Raydar

A custom radar sensor plugin for **Gazebo Garden** (gz-sim7) with **ROS 2 Humble** integration.

The plugin simulates a marine radar by detecting discrete target models (buoys, posts, boats, etc.) within a configurable range and field of view. It publishes target detections in polar coordinates (range, azimuth), along with a radial velocity (Doppler) estimate, and optionally renders them as RViz markers.

## Features

- Gazebo system plugin (`raydar::RadarSystem`)
- Publishes radar target detections in a custom ROS 2 message
- Publishes RViz markers for visualization
- Configurable range, field of view, and angular resolution via SDF
- Optional debug logging
- Self-contained ROS 2 package (messages, plugin, and hook file in one repo)

## Requirements

- Ubuntu 22.04 (tested)
- ROS 2 Humble
- Gazebo Garden (`gz-sim7`, `gz-rendering7`)
- A world containing a model named `wamv` (the radar origin)

## Build

```bash
cd ~/Documents/marine
colcon build --merge-install --packages-select raydar
source install/setup.bash
```

## SDF Configuration

Add the plugin to your world's `<world>` block or inside a specific `<model>` block:

```xml
<plugin filename="libRadarSystem.so" name="raydar::RadarSystem">
  <max_range>100.0</max_range>
  <fov_half_angle_deg>60.0</fov_half_angle_deg>
  <num_rays>360</num_rays>
  <verbose>false</verbose>
</plugin>
```

### Parameters

| Element | Type | Default | Description |
|---|---|---|---|
| `max_range` | double | `100.0` | Maximum detection range in meters |
| `fov_half_angle_deg` | double | `60.0` | Half of the horizontal field of view, in degrees (60 means ±60°, i.e. 120° total) |
| `verbose` | bool | `false` | If true, prints per-target detections to the terminal |

## Published Topics

| Topic | Type | Description |
|---|---|---|
| `/radar/targets` | `raydar/msg/RadarTargetArray` | Radar detections in polar coordinates |
| `/radar/markers` | `visualization_msgs/msg/MarkerArray` | RViz markers: colored spheres (green = close, red = far) with text labels showing target name and range |

The `header.frame_id` on all messages is `wamv/base_link`, so RViz's **Fixed Frame** should be set to the same value, as it is tested on vrx robotx package

| Field | Unit | Description |
|---|---|---|
| `target_id` | — | Name of the detected model (e.g. `mb_marker_buoy_red`) |
| `range` | meters | Distance from the radar origin to the target center |
| `azimuth` | radians | Angle relative to platform heading, positive counterclockwise, in `[-π, π]` |
| `elevation` | radians | Currently always 0 (2D radar) |
| `radial_velocity` | m/s | Positive = moving away from radar, negative = approaching |
| `rcs` | — | Placeholder for radar cross section (currently always 1.0) |

## Visualizing in RViz

2. In a new terminal:

   ```bash
   source ~/Documents/marine/install/setup.bash
   rviz2
   ```

3. Set **Fixed Frame** to `wamv/base_link`or custom frame.
4. Add a **MarkerArray** display and set its topic to `/radar/markers`.

You should see colored spheres at each target's position, with text labels above them.

To inspect the raw data:

```bash
ros2 topic echo /radar/targets
```

## Known Limitations

- **Point-target model**: each detected model is treated as a single point at its center. Extended objects (shorelines, long walls, large meshes) are not correctly perceived — they appear as a single return at the model's origin.
- **No occlusion**: the radar does not check whether the line of sight is blocked by another object.
- **No measurement noise**: targets are reported with exact ground-truth values. Real radars have range-, angle-, and Doppler-dependent noise.
- **Doppler is currently 0**: velocity lookup is a placeholder; real Doppler from link velocities is planned.
- **Static scene assumption**: targets are re-detected every scan by iterating over all models in the world. Performance degrades with very large worlds.

A ray-casting upgrade is planned to address the point-target limitation and enable detection of shorelines and extended structures.

## License

Apache-2.0
