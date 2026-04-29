# AGENTS.md — gridmapper

## Build

```shell
# From workspace root (/home/jazzy/nav_t_ws):
colcon build --packages-select gridmapper --cmake-args -Wno-dev -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --symlink-install
```

The custom ROS package `control_input_msgs` must be built first:
```shell
colcon build --packages-select control_input_msgs --cmake-args -Wno-dev --symlink-install
```

No lint, typecheck, or test commands exist for this package. There are no unit tests — only ament lint stubs in `CMakeLists.txt`.

## Architecture

- Single ROS 2 `ament_cmake` package producing one binary: `gridmapper_node`.
- Entry point: `src/main.cpp` → instantiates `GridMapperNode<pcl::PointXYZI>` (a template class).
- Two operational modes controlled by `mapper_mode` parameter:
  - `"global"`: persistent Kalman-fused global map with save/load support.
  - `"local"`: robot-centric sliding window for real-time obstacle avoidance.
- Launch files: `launch/global.launch.py` and `launch/local.launch.py`, each loading the corresponding `config/gridmapper_*.yaml`.

## GPU / OpenCL gotchas

- OpenCL kernel files (`.cl` in `src/kernels/`) are **runtime-compiled** — loaded as text, compiled by the OpenCL driver, not by CMake.
- At runtime, kernel files are resolved via `ament_index_cpp` (install path) or fallback to `ROOT_DIR` (source tree define: `-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"`).
- After editing a `.cl` file, a rebuild is **not** needed, but the node must be restarted.
- OpenCL is enabled per-launch via the `use_gpu` launch argument (defaults to `true` in both launch files), overriding the YAML default of `false`.

## Parameter quirks

- `mapper_mode` accepts strings `"local"` or `"global"` (not a bool).
- `fuse_external_map_en` is LOCAL-mode only — fuses `/map` occupancy grid into `/terrain_map` for blind-spot awareness. The global config disables it.
- The global launch file sets `sigterm_timeout=60.0` to allow orderly shutdown with map saving; the local launch does not.
- `READMElocal.md` references a deprecated `localization_mode_en` parameter — ignore it; use `READMElocal.md` only for the GPU algorithm descriptions.

## Data flow

- Input topics (time-synchronized via `message_filters::ApproximateTime`):
  - `/cloud_registered_body_horizon` (PointCloud2)
  - `/odometry_horizon` (Odometry)
  - `/odometry_imu_horizon` (Odometry, optional high-rate)
- Key output: `/terrain_map` (PointCloud2 — colored traversability view), `/global_grid_map` (GridMap), `/map` (OccupancyGrid).
- Services:
  - `/gridmap_save` (GLOBAL only) — saves map, resets cache, records topology.
  - `/clear_local` (LOCAL only) — clears local map buffer.
