# AGENTS.md — local_planner

## Build & Run

```bash
# Build (from workspace root, e.g. nav_t_ws)
colcon build --packages-select local_planner --cmake-args -Wno-dev -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --symlink-install

# Launch
ros2 launch local_planner local_planner.launch.py use_sim_time:=true start_rviz:=true debug_info:=true
```

## Architecture

- Two nodes in one package: `localPlanner` (terrain-aware path scoring) and `pathFollower` (path tracking → cmd_vel)
- Communication: `/odometry_horizon` + `/terrain_map` + `/plan` → localPlanner → `/local_path` + `/free_paths` → pathFollower → `/cmd_vel`
- `control_input_msgs` is a custom external package dependency, not standard ROS 2

## Key Conventions

- C++17, ament_cmake, OpenMP used in localPlanner for parallel path evaluation
- Node parameters are namespaced per node in YAML (`localPlanner: ros__parameters:`, `pathFollower: ros__parameters:`)
- Three robot config profiles: `_x30`, `_dog`, `_dog_rcnav` — launch file defaults to `x30`; change `params_file` line in `local_planner.launch.py` to switch
- `.ply` and `.txt` files in `paths/` are pre-generated path libraries read at runtime; the `.m` files are MATLAB scripts for offline path generation only

## Testing

- Only ament_lint_auto (no unit/integration tests)

## Lint

```bash
colcon test --packages-select local_planner
```