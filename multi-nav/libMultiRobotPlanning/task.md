# 该项目的哪个算法实现了和 @/home/jazzy/py/GCS_Multi_Inspection_MIBNB 中算法类似的功能？
• 最接近的是 CBS-TA（Conflict-Based Search with Optimal Task Assignment），对应实现为：
- include/libMultiRobotPlanning/cbs_ta.hpp:15
- 示例：example/cbs_ta.cpp:1

两者都包含“为多个机器人分配任务并规划路径”的高层目标。

不过它们并不等价：

| GCS_Multi_Inspection_MIBNB | libMultiRobotPlanning 对应/差异 |
|---|---|
| 分配多个 checkpoint 给每个 depot，目标是最小化最大负载 + 总距离 | CBS-TA 基于一对一任务分配，优化总路径代价 |
| 每个 depot 对已分配 checkpoint 解 TSP 访问顺序 | 本库没有多任务巡回/TSP 求解器 |
| 在凸集图上用 MIP Branch-and-Bound 求连续无碰路径 | 本库的 CBS-TA 默认以离散图/状态空间 A* 作为低层规划 |
| 各 depot 轨迹独立生成，代码没有跨机器人冲突消解 | CBS-TA 的核心正是检测并消解机器人之间的时空冲突 |

因此，若问“功能最类似的单一算法”，答案是 CBS-TA；若只看检查点分配，则较接近 include/libMultiRobotPlanning/assignment.hpp:14，但它只能做最小总成本的一对一匹配，不能替代该项目的负载均衡多点分配。Python 项目的实际流程可见 /home/jazzy/py/GCS_Multi_Inspection_MIBNB/spp/high_level_inspection_MIBNB.py:12：分配后再解每个 depot 的 TSP。

问题:
1. GCS_Multi_Inspection_MIBNB "最小化最大负载 + 总距离"如何理解？CBS-TA "基于一对一任务分配，优化总路径代价"不是多机分配算法吗？
2. "检测并消解机器人之间的时空冲突" 如何理解？
3. 能用具体的例子直观说明这两个算法之间的异同吗？

• 1. 两种“分配”的目标不同

GCS_Multi_Inspection_MIBNB 的 checkpoint 分配是多对多的：一个 depot/机器人可以拿到多个 checkpoint，且每个 checkpoint 必须恰好分给一台机器人。它优化的是：
```
[
\min\left(\max_i L_i+\lambda\sum_i L_i\right)
]
```
其中 (L_i) 是第 (i) 台机器人被分配任务的代价之和，第一项让最忙机器人的负载尽量小，第二项避免为了均衡而产生过多绕路。实现见 /home/jazzy/py/GCS_Multi_Inspection_MIBNB/spp/high_level_inspection_MIBNB.py:39。

例如有两台机器人、三个检查点：
```
分配               R1 负载    R2 负载    最大负载
━━━━━━━━━━━━━━━━━  ━━━━━━━━━  ━━━━━━━━━  ━━━━━━━━━━
R1: A、B；R2: C          3          1           3
─────────────────  ─────────  ─────────  ──────────
R1: A；R2: B、C          1         10          10
```
即使第二种总距离未必大很多，第一种也更好，因为任务完成时间通常由最慢机器人决定。之后，该项目再对每台机器人获得的多个 checkpoint 单独解 TSP，决定其访问顺序，见 /home/jazzy/py/GCS_Multi_Inspection_MIBNB/Inspection_wrapper.py:581。

CBS-TA 当然也是多机器人分配算法，但它的标准问题是“一台机器人对应一个任务目标”。每个高层搜索根节点是一组一对一匹配，再为每台机器人规划到其目标的路径，优化所有机器人路径代价之和。它并不原生处理“R1 访问 A、B、C，且需要确定 A-B-C 顺序”的多任务巡检/TSP 问题。其任务映射确实是 agent -> task，见 include/libMultiRobotPlanning/cbs_ta.hpp:219。

2. “检测并消解时空冲突”的含义

两台机器人不能在同一时刻占用同一位置，也通常不能在同一时间段沿同一条边反向穿越。

例如，一条单格宽走廊：
```
A1 起点  --  X  --  A2 起点
A1 目标  <-- X -->  A2 目标
```
若两台机器人同时出发、都经过 X，会在 t=1 同时到达 X，这是顶点冲突。若两者在一条边的两端互换位置，则是边冲突。

CBS-TA 的做法是：

1. 先按当前任务分配，为每台机器人独立找最短路径。
2. 检查是否有顶点或边冲突。
3. 若冲突，将问题分支：
    - 分支一：禁止 A1 在 t=1 进入 X；
    - 分支二：禁止 A2 在 t=1 进入 X。
4. 只对受约束的机器人重新规划，直到找到无冲突方案。
5. 同时继续枚举更优或次优的任务匹配，必要时用另一组分配避开冲突。

这正是 include/libMultiRobotPlanning/cbs_ta.hpp:135 中 getFirstConflict、createConstraintsFromConflict 和重新低层搜索的流程。

相比之下，GCS 项目是“分配后，每台机器人分别规划自己的连续轨迹”。从其高层流程看，轨迹按 depot 并行独立生成 /home/jazzy/py/GCS_Multi_Inspection_MIBNB/Inspection_wrapper.py:643，没有将机器人间碰撞作为约束反馈回任务分配、TSP 或连续轨迹求解。

3. 一个完整的直观对比

假设有两个无人机 R1、R2，三个巡检点 A、B、C，区域中有窄走廊。

GCS_Multi_Inspection_MIBNB：

分配：R1 -> A, B；R2 -> C
排序：R1 解 TSP，得到 Depot1 -> A -> B
路径：每个相邻检查点之间，用 GCS/MIP 求连续可行轨迹

它擅长的是“巡检覆盖”：保证所有检查点被分配，兼顾负载均衡，给每架无人机规划多站访问路线。问题是若 R1、R2 同时经过窄走廊，当前流程不保证避免相互碰撞。

CBS-TA：

候选分配 1：R1 -> A；R2 -> C
各自路径：两机都经过窄走廊，发生冲突
冲突分支：让 R1 等待，或让 R2 改道/等待
候选分配 2：R1 -> C；R2 -> A
比较所有无冲突方案的总代价，选最小者

它擅长的是“多机协同通行”：任务分配与避碰路径规划联动，确保时间维度上无碰撞。问题是它不直接解决“一台机器人巡检多个 checkpoint 的访问顺序”。

所以实际工程组合常是：
```
GCS 的多点任务分配 + 每机 TSP
        ↓
将每条巡检路线转换为带时间的多机路径问题
        ↓
CBS / CBS-TA / SIPP 等协调器消解跨机器人冲突
```
若要让本库更接近该 Python 项目，CBS-TA 是合适的协调框架，但需要扩展任务定义和低层环境，使一个“任务”能够表示一串 checkpoint，或在任务分配/TSP 后额外接入 CBS 冲突协调层。

## task1
### @/home/jazzy/cpp/libMultiRobotPlanning 这个项目包含仿真的内容吗？请复现，如果需要 python 环境，使用 uv 在项目下建立

• 包含，但它是离散 MAPF 的路径动画，不是带动力学、传感器或物理引擎的机器人仿真。

我已复现 CBS 四机器人交换场景：
- 输入：test/mapf_swap4.yaml
- CBS 调度输出：build/simulation_cbs_swap4.yaml
- 动画视频：build/simulation_cbs_swap4.mp4
- 第 4 秒截图：build/simulation_cbs_swap4_t4.png

结果：总代价 28、makespan 8、CBS 高层展开 5 个节点。四台机器人从上下左右穿过中央障碍进行目标交换，生成的调度无顶点冲突和反向边冲突。

可视化实现位于 example/visualize.py:13，它按时间插值移动圆形机器人，并在检测到机器人距离小于 0.7 时标红报告碰撞。

本项目下已通过 uv 建立 .venv 并安装 matplotlib、pyyaml；CMake 构建产物位于 build/。复现命令如下：
```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 4

cd build
./cbs -i ../test/mapf_swap4.yaml -o simulation_cbs_swap4.yaml
../.venv/bin/python ../example/visualize.py \
../test/mapf_swap4.yaml simulation_cbs_swap4.yaml \
--video simulation_cbs_swap4.mp4
```

问题:
1. 该项目包含哪些算法，各算法针对哪些场景？请举形象的例子说明。并且复现各算法的仿真。

| 算法 | 适用场景 | 形象例子 | 复现结果 |
|---|---|---|---|
| **A\*** | 单机器人、静态地图、要求最短路 | 叉车绕开货架去指定工位 | `build/reproductions/a_star.yaml`，代价 3 |
| **A\* epsilon** | 单机器人，需要更快的近似解 | 配送车允许多走一点以更快出路线 | `build/reproductions/a_star_epsilon_w1_5.yaml`，`w=1.5`，代价 3 |
| **SIPP** | 单机器人、存在动态占用 | 机器人在路口等待另一台车先通过 | `build/reproductions/sipp_dynamic_obstacle.yaml`，在 `t=3..8` 等待 |
| **CBS** | 多机器人，要求最优且无碰撞 | 四台车从十字通道四侧交换位置 | `build/reproductions/cbs_swap4.mp4`，总代价 28、makespan 8 |
| **ECBS** | 多机器人，可接受有界次优来加速 | 仓库高峰期优先快速给出可行无碰撞方案 | `build/reproductions/ecbs_swap4_w1_3.mp4`，`w=1.3`，总代价 28 |
| **CBS-TA** | 多机器人，一对一目标分配与避碰联合求解 | 两台车要去两个工位，自动决定谁去哪个 | `build/reproductions/cbs_ta_assignment.mp4`，总代价 6 |
| **ECBS-TA** | CBS-TA 的有界次优加速版 | 大规模工位派发，需要快速协同方案 | `build/reproductions/ecbs_ta_assignment_w1_3.mp4`，总代价 6 |
| **Prioritized Planning + SIPP** | 多机器人，按优先级快速规划 | 先给主通道 AGV 规划，后续车辆绕开其时间窗口 | `build/reproductions/prioritized_sipp_swap4.mp4`，总代价 28 |
| **CBS on Roadmap** | 多机器人在任意图、运动原语或道路网络中规划 | 机器人在预定义航线图中避让 | `build/reproductions/cbs_roadmap_annotated.mp4`，总代价 8 |
| **Assignment** | 最小总成本一对一分配 | 四名员工分配给四个订单 | `build/reproductions/assignment_4x4.yaml`，总代价 275 |
| **Next-Best Assignment** | 按成本依次枚举可行分配 | 最优人选不可用时，立即选择下一优方案 | `build/reproductions/next_best_assignment_4x4.yaml`，24 种方案，最优 275 |

| 算法 | 具体机器人例子 |
|---|---|
| **A\*** | 只有机器人 `a`。它从仓库入口 `(0,0)` 去充电桩 `(2,1)`，中间 `(1,1)` 是货架。`a` 选择 `(0,0) → (1,0) → (2,0) → (2,1)`，总代价 `3`。它不关心其他机器人，也不处理动态障碍。 |
| **A\* epsilon** | 仍只有 `a`，但调度系统只给 `20 ms` 出路线。最短路线可能代价 `10`；设置 `w=1.5` 后，算法允许代价最多约 `15` 的路线，以更少搜索节点换取速度。就像“允许 `a` 多绕半圈，但必须立刻发车”。 |
| **SIPP** | `a` 从 `(0,1)` 去 `(2,3)`；但机器人 `b` 的既有调度表规定：它在 `(2,3)` 停留至 `t=8`。`a` 走到 `(1,3)` 后不能进入目标，因此执行：`t=3` 到达 `(1,3)`，等待到 `t=8`，`t=9` 再进入 `(2,3)`。SIPP 的关键是把 `(2,3)` 的可进入时间建模为**安全时间区间**。 |
| **CBS** | 四台机器人要通过被中央障碍物分开的区域：`a: (0,2) → (4,2)`，`b: (4,2) → (0,2)`，`c: (2,4) → (2,0)`，`d: (2,0) → (2,4)`。独立最短路可能让 `a` 与 `c` 同时占 `(1,2)`。CBS 发现冲突后分支：一种方案禁止 `a` 在 `t=3` 占该格，另一种禁止 `c` 占该格；重规划直至四者在不同时间经由上下绕行通道通过。 |
| **ECBS** | 场景同 CBS。CBS 必须证明“总路程绝对最小”；ECBS 设置 `w=1.3` 后，只要求解的总代价不超过当前理论下界的 `1.3` 倍。比如 `a` 多等待一拍或 `d` 走稍长绕路也可接受，以明显减少高层冲突树搜索。当前复现实例恰好仍找到总代价 `28` 的解。 |
| **CBS-TA** | `a` 位于 `(0,0)`，`b` 位于 `(1,0)`；待执行目标为 `P=(4,0)`、`Q=(3,0)`。候选分配包括：`a→P, b→Q` 与 `a→Q, b→P`。CBS-TA 不只比较距离，还会检查两种分配下的路径冲突。若某种分配使 `a` 与 `b` 在窄走廊相遇，它会像 CBS 一样加入时空约束，或转向另一组任务分配。 |
| **ECBS-TA** | 场景同 CBS-TA，但不必穷尽证明每一种“分配 + 避碰路径”组合都严格最优。比如 `a→P,b→Q` 的总代价为 `6`，另一条无冲突方案总代价 `7`；当 `w=1.3` 时，总代价 `7` 也可能被接受，从而更快结束搜索。 |
| **优先级规划 + SIPP** | 设优先级 `a > b > c > d`。先为 `a` 规划并固定其完整时空轨迹；然后 `b` 将 `a` 的轨迹视为动态障碍，用 SIPP 绕开或等待；再依次规划 `c、d`。例如 `a` 在 `t=3` 经过 `(3,2)`，`b` 就不能同一时刻进入该格。它快，但优先级不佳时，`a` 的路线可能堵死 `d`。 |
| **CBS on Roadmap** | 不是方格，而是路网节点 `A-B-C-D` 和一条支路。机器人 `a` 从节点 `A` 去 `C`，`b` 从 `D` 去 `B`。若两者会在同一时段反向通过边 `B-C`，CBS 会禁止其中一台在该时段使用该边，令它在 `B` 等待或走支路。适合室内航线、AGV 道路网络、运动原语图。 |
| **Assignment** | 有四台机器人 `a,b,c,d` 和四个一次性任务 `T0..T3`。成本矩阵表示“机器人去执行任务的距离/耗时”。求解得到：`a→T3, b→T2, c→T1, d→T0`，总代价 `275`。它只负责配对，不生成路线，也不检查机器人会不会相撞。 |
| **Next-Best Assignment** | 场景同 Assignment。第一方案是 `a→T3, b→T2, c→T1, d→T0`。若 `d` 电量不足而不能接 `T0`，系统不必重新从头求解，可以给出下一成本最低的完整分配，再给第三方案，依此枚举。复现的 `4×4` 例子共有 `24` 种一对一匹配。 |
