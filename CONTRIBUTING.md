# Contributing

Thanks for considering a contribution to `gz_camera_zoom`.

## Reporting issues

Please open a GitHub issue with:

- Gazebo/ROS distro and versions (`gz sim --versions`, `echo $ROS_DISTRO`), since
  this plugin is sensitive to `gz-sim`/`gz-rendering`/`gz-sensors` internals
  that change between releases.
- The SDF snippet where the plugin is attached.
- Steps to reproduce, and any `gzerr`/`gzmsg` console output.

## Development

```bash
colcon build --packages-select gz_camera_zoom --cmake-args -DCMAKE_BUILD_TYPE=Debug
colcon test --packages-select gz_camera_zoom
```

`colcon test` runs `ament_copyright` via `ament_lint_auto` to check license
headers. This project intentionally follows Gazebo's own upstream C++ style
(as used by `gz-sim`'s `LensFlare`/`CameraVideoRecorder` systems — blank
line after each `public:`, PascalCase methods, Doxygen `\brief` comments)
rather than ROS's Google-derived `ament_cpplint`/`ament_uncrustify` style;
please match the existing style in `CameraZoom.cc`/`.hh` rather than the
ROS default.

## Pull requests

- Keep PRs focused; unrelated cleanups make review harder.
- If you're touching the render-thread/`PreRender` handling, please read the
  "Why this design" section of the [README](README.md) first — the
  threading model here is load-bearing, not incidental.
- New source files should carry the Apache-2.0 header (see any existing
  `.cc`/`.hh` file for the exact boilerplate).

## License

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~
