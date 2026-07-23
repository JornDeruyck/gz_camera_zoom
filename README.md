# gz_camera_zoom

[![Build](https://github.com/JornDeruyck/gz_camera_zoom/actions/workflows/build.yml/badge.svg)](https://github.com/JornDeruyck/gz_camera_zoom/actions/workflows/build.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)

A Gazebo Sim (`gz-sim`) system plugin that adds continuous **optical** zoom
(horizontal field-of-view control) to a camera sensor, driven by a
gz-transport topic. This changes the camera's projection, the same thing a
real zoom lens does when its focal length changes — it is not a digital
crop/upscale of the rendered image.

Built and tested against the versions shipped in ROS 2 Jazzy / Gazebo
Harmonic: `gz-sim8` 8.11.0, `gz-rendering8` 8.2.3, `gz-sensors8` 8.2.2,
`gz-transport13` 13.5.0, `gz-msgs10` 10.3.2.

## Why this design

Gazebo's `Sensors` system renders all cameras on a dedicated, internal render
thread, synchronized against the simulation thread with its own mutexes and
condition variables (see `gz-sim/src/systems/sensors/Sensors.cc`).  

This plugin follows the same pattern gz-sim's own `LensFlare` and
`CameraVideoRecorder` systems use: transport callbacks only ever write a
plain command value behind a mutex; all `gz::rendering::Camera` access
happens inside a callback connected to `gz::sim::events::PreRender`, which
Gazebo emits from inside the render thread itself. No new locks against
Gazebo's internals, no touching the camera from anywhere else.

## Attaching the plugin

Nest it inside the `<sensor type="camera">` element, same as
`CameraVideoRecorder`:

```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image><width>640</width><height>480</height></image>
  </camera>
  <topic>camera/image</topic>

  <plugin filename="CameraZoom" name="gz_camera_zoom::CameraZoom">
    <topic>camera/zoom/cmd</topic>
    <camera_info_topic>camera/zoom/camera_info</camera_info_topic>
    <level_topic>camera/zoom/level</level_topic>
    <min_hfov>0.05</min_hfov>
    <max_hfov>1.047</max_hfov>
    <zoom_rate>0.4</zoom_rate>
    <cmd_timeout>0.5</cmd_timeout>
  </plugin>
</sensor>
```

All `<plugin>` parameters are optional:

| Param                | Default                          | Meaning                                                        |
|-----------------------|----------------------------------|-----------------------------------------------------------------|
| `topic`               | `<sensor topic>/zoom/cmd`         | Command topic (`gz.msgs.Double`)                                 |
| `camera_info_topic`   | `<sensor topic>/zoom/camera_info` | Live, zoom-corrected `gz.msgs.CameraInfo`                        |
| `level_topic`         | `<sensor topic>/zoom/level`       | Current zoom factor as `gz.msgs.Double` (`max_hfov / current_hfov`) |
| `min_hfov`            | `0.05` rad                        | Narrowest FOV = maximum zoom-in                                  |
| `max_hfov`            | sensor's configured `horizontal_fov` | Widest FOV = fully zoomed out                                 |
| `zoom_rate`           | `0.4` rad/s                        | HFOV change rate at command magnitude 1.0                       |
| `cmd_timeout`         | `2.0` s                           | Safety watchdog: treat as "stop" if no command arrives in time  |

## Commanding zoom

Publish a `gz.msgs.Double` on the command topic:

- `+1.0` → start zooming in (narrow the FOV) at the full configured rate
- `-1.0` → start zooming out (widen the FOV)
- any value in between scales the rate; `0.0` stops
- **keep publishing** to hold a direction, the same way a real PTZ zoom
  motor is driven — a single one-shot message will auto-stop after
  `cmd_timeout` seconds (the watchdog), so "start zoom in" / "stop zoom in"
  map to publishing a nonzero value and then publishing `0.0` (or just
  letting the watchdog catch it if the commanding node goes away)

`cmd_timeout` is a dead-publisher safety net, not a control-loop rate
requirement — it defaults to a generous 2s specifically so that a modest,
possibly irregular publish rate (e.g. a plain `ros2 topic pub` with no `-r`,
which defaults to 1 Hz) doesn't cause visible stutter. If `cmd_timeout` is
shorter than your actual publish period, the commanded rate will flicker
between the value and zero once per message, and the zoom will visibly
step rather than glide smoothly — the per-tick FOV integration itself is
already smooth every render frame, so stutter here is always a rate/timeout
mismatch, not a rendering issue.

```bash
gz topic -t /camera/zoom/cmd -m gz.msgs.Double -p 'data: 1.0'   # one-shot: zooms in for cmd_timeout seconds, then stops
gz topic -t /camera/zoom/cmd -m gz.msgs.Double -p 'data: 0.0'   # stop immediately
gz topic -t /camera/zoom/cmd -m gz.msgs.Double -p 'data: -1.0'  # one-shot: zoom out for cmd_timeout seconds
gz topic -e -t /camera/zoom/level                               # watch current zoom factor

# to hold a direction indefinitely, keep publishing faster than cmd_timeout, e.g.:
watch -n 0.2 "gz topic -t /camera/zoom/cmd -m gz.msgs.Double -p 'data: 1.0'"
```

From ROS 2, bridge the command topic with `ros_gz_bridge` as a
`std_msgs/Float64`. `ros2 topic pub /topic std_msgs/msg/Float64 "{data: 1.0}"`
(no `-r`) publishes continuously at 1 Hz, comfortably within the default 2s
timeout; `--once` sends a single message and will auto-stop after
`cmd_timeout` seconds.

## On `/camera_info`

The stock `gz-sensors` `CameraSensor` builds its `camera_info` message once
at load time and only republishes it with a new timestamp afterwards — it
does not recompute intrinsics if the FOV changes later (there's no public
API to do so; this is `gz-sensors` internal state). So zooming will
correctly change the rendered image, but the sensor's own `.../image`-topic
sibling `camera_info` will silently go stale.

This plugin publishes its own, always-current `CameraInfo` on a separate
topic (`.../zoom/camera_info` by default) instead, recomputed every frame
from the camera's live projection matrix via the same
`gz::rendering::projectionToCameraIntrinsic()` utility `gz-sensors` itself
uses.

## Building

This is a standalone `ament_cmake` package so it builds with the rest of the
ROS 2 workspace:

```bash
colcon build --packages-select gz_camera_zoom
source install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/install/gz_camera_zoom/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH"
gz sim -r install/gz_camera_zoom/share/gz_camera_zoom/worlds/camera_zoom_demo.sdf
```

The demo world (`worlds/camera_zoom_demo.sdf`) has a static camera looking
down a row of colored spheres, with the plugin attached and the topic names
above. Its header comment has the exact commands to try.

## A note on rendering activity

`gz-sim`'s `Sensors` system normally skips a camera's own image
capture/publish when nothing is subscribed to its image topic, as a
performance optimization. This plugin's own connection to
`gz::sim::events::PreRender` makes the render thread tick every simulation
step regardless (verified: zoom control and the `.../zoom/camera_info` and
`.../zoom/level` topics all update correctly with zero image subscribers),
since it operates directly on the `gz::rendering::Camera` object rather than
through the sensor's own active/inactive gating. The image topic itself
still won't carry frames until something subscribes to it, which is normal
and expected — zoom control doesn't require an image consumer to work.

## Contributing

Issues and pull requests are welcome — see [CONTRIBUTING.md](CONTRIBUTING.md).

## License

Licensed under the [Apache License, Version 2.0](LICENSE).
