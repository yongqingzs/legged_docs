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
