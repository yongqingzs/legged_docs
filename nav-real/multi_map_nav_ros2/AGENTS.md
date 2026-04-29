# AGENTS.md

## Build

```sh
colcon build --packages-select multi_map_nav --cmake-args -Wno-dev -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --symlink-install
```

- Must run from the **workspace root** (`~/nav_t_ws`), not from inside this package.
- `--symlink-install` is required so launch/params changes take effect without rebuilding.

## Architecture

- **Single executable**: `multi_map_nav_node` (C++, `src/multi_map_nav.cpp`)
- **Nav2 plugin**: `DummyController` (`src/dummy_controller.cpp`) — a zero-velocity controller that lets external planners drive the robot while Nav2's goal_checker handles arrival detection.
- **Launch file**: `launch/multi_map_nav.launch.py` starts the node plus all Nav2 servers (map_server, planner, controller, behaviors, bt_navigator, lifecycle_manager) and a static `map→odom` TF.
- **Data dirs**: `maps/` (gitignored — contains `.yaml` + image pairs and `map_connections.txt`), `params/` (Nav2 YAML configs), `rviz/`.

## Key Concepts

- **Coordinate transform**: `World = Local + YAML_origin`. Each map's YAML `origin` field is parsed at startup; all navigation targets are projected into a unified world frame so Nav2 can plan across cropped sub-maps.
- **Map connections**: CSV format `FROM_MAP,TO_MAP,FROM_X,FROM_Y,TO_X,TO_Y,Z`. `bidirectional_connections` (default `true`) auto-generates reverse edges.
- **Goal interfaces**:
  - Topic: `/multi_map_goal_pose` (`PoseStamped` — `header.frame_id` must be the target map name)
  - Action: `/multi_map_navigate_to_pose` (`nav2_msgs/action/NavigateToPose`)
  - Action: `/multi_map_navigate_through_poses` (`nav2_msgs/action/NavigateThroughPoses`)
- **`use_fake_cmdvel`** (launch arg): remaps `cmd_vel` → `cmd_vel_fake` for dry-run testing.

## Launch Arguments

| Arg | Default | Notes |
|-----|---------|-------|
| `initial_map` | `park` | Must match a map name in connections file |
| `map_connections_file` | `map_connections` | Filename only (no `.txt`), resolved inside `maps/` |
| `params_file` | `normal` | Filename only (no `.yaml`), resolved inside `params/` |
| `use_sim_time` | `true` | |
| `use_fake_cmdvel` | `false` | |
| `bidirectional_connections` | `true` | |
| `waypoint_dwell_time` | `2.0` | Seconds to wait at each waypoint |
| `patrol_loops` | `3` | Times to loop through pose list |

## Dependencies

- External package `inspection_task_hub` (custom srv) must be available in the workspace.
- OpenCV is linked — ensure `libopencv-dev` is installed.

## Tests

- Only `ament_lint_auto` (no unit tests). Run with: `colcon test --packages-select multi_map_nav`

## File Conventions

- `maps/` directory is **gitignored** — it holds runtime map data not versioned here.
- Map YAML files must contain an `origin` field; the node parses it for coordinate conversion.
- The `params/` directory contains Nav2 parameter presets (`normal.yaml`, `new_local.yaml`).