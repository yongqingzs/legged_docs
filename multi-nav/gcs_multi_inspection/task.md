该项目实现的是一个“多无人机电厂巡检路径规划”流程，核心不是图像识别，而是基于地理坐标和区域几何信息，计算多架无人机访问所有巡检点的最优路线。

## 1. 项目实现的功能

主要流程位于 Inspection_wrapper.py：

1. 读取巡检场景数据
    - 读取起点、边界、区域、无人机基地和巡检点 CSV。
    - 使用 pymap3d 将经纬高坐标转换为以起点为原点的 ENU 坐标：
        - East：东向
        - North：北向
        - Up：高度方向

2. 建立凸集图模型
    - 每个 Regions.csv 中的多边形区域被建模为一个凸多面体。
    - 对严格相交的区域建立图边。
    - 每条边的代价使用欧氏距离，即 TwoNorm。
    - Depot 和 Checkpoint 被作为单点凸集加入图中。

3. 将巡检点分配给多个 Depot
    - 对每个 Depot 到每个 Checkpoint 求解带速度和时间约束的最短路径。
    - 使用 MILP 将所有 Checkpoint 分配给某一个 Depot。
    - 目标同时考虑：
        - 各无人机最大负载；
        - 总路径距离。
    - alpha 控制负载均衡和总距离之间的权衡。

4. 为每个无人机规划访问顺序
    - 对每个 Depot 分配到的 Checkpoint，计算两两最短距离。
    - 使用带 MTZ 子环消除约束的 TSP MILP，得到访问顺序。
    - 路线通常是：
    Depot -> Checkpoint1 -> Checkpoint2 -> ... -> Depot

5. 生成连续轨迹
    - 对 TSP 路径中的每一段分别求解图上的最短路径。
    - 提取每个凸区域的进入点、代表点和离开点。
    - 通过 LinearTrajectory 和 Hermite 插值生成随时间变化的位置轨迹。
    - 轨迹包含时间、位置、所属区域等信息。

6. 输出飞行航点
    - 生成高密度航点，默认时间间隔为 0.01 秒。
    - 同时输出：
        - ENU 局部坐标航点；
        - 经纬高 LLH 航点。

7. 可视化
    - 绘制环境、区域、Depot、Checkpoint 和规划轨迹。
    - 可生成静态 PDF。
    - 可生成或播放多无人机运动动画。

底层算法主要在 spp/high_level_inspection.py、spp/low_level_inspection.py 和 spp/shortest_path_3point_time.py 中实现，使用 Drake 的数学规划接口和 Mosek 求解器。

## 2. 输入和输出

### 输入文件

程序默认从当前目录读取以下文件：

文件               内容            具体格式
━━━━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Origin.csv         ENU 坐标原点    类型、名称、经度、纬度、高度
─────────────────  ──────────────  ─────────────────────────────────────────────
Boundary.csv       场景外边界      NW/NE/SE/SW 四个边界点
─────────────────  ──────────────  ─────────────────────────────────────────────
Regions.csv        可通行区域      每行一个多边形名称，以及多个 经度,纬度 顶点
─────────────────  ──────────────  ─────────────────────────────────────────────
Depots.csv         无人机基地      Depot 名称、经度、纬度、高度
─────────────────  ──────────────  ─────────────────────────────────────────────
Checkpoints.csv    巡检点          Checkpoint 名称、经度、纬度、高度

当前示例数据包含：

- 1 个原点；
- 4 个边界点；
- 28 个区域多边形；
- 3 个 Depot；
- 10 个 Checkpoint。

程序只读取第一列标记为 g 的记录。坐标字段按照经度、纬度、高度读取；内部转换时使用 [纬度, 经度, 高度] 顺序传给 pymap3d。区域多边形的高度被固定为 0，实际规划主要使用二维 East-North 坐标。

Corners.csv 当前没有被 Inspection_wrapper.py 的主流程读取，属于辅助数据。

### 中间数据和函数输出

主要函数的输出如下：

#### input_csv_reader

返回：

origin_name
origin_llh
boundaries
regions
depots
depots_nh
checkpoints
checkpoints_nh

其中：

- origin_llh：原点的 (纬度, 经度, 高度)
- boundaries：边界点的 ENU 坐标字典
- regions：多边形顶点的 ENU 二维坐标
- depots / checkpoints：三维 ENU 坐标
- depots_nh / checkpoints_nh：只保留 East-North 的二维坐标

#### graph_construction

返回：

- 凸集对象列表；
- 区域名称列表；
- 区域连接关系；
- 欧氏距离代价函数；
- 每个 Depot 所在的区域；
- 每个 Checkpoint 所在的区域；
- 多无人机凸集图对象。

#### assign_checkpoints_to_depots

返回：

cost_d2c
cluster_matrix
cluster_dict

具体含义：

- cost_d2c[depot][checkpoint]：某 Depot 到某 Checkpoint 的最短路径代价；
- cluster_matrix：Depot-Checkpoint 的 0/1 分配矩阵；
- cluster_dict：例如：

{
    "Depot0": ["Checkpoint1", "Checkpoint4"],
    "Depot1": ["Checkpoint0", "Checkpoint5"],
    "Depot2": ["Checkpoint2", "Checkpoint3", "Checkpoint6"]
}

#### high_level_tsp

返回：

- 每个 Depot 对应子图的距离矩阵；
- 每个 Depot 的 TSP 结果。

TSP 结果通常是：

(path, total_cost)

其中 path 是访问顺序，total_cost 是总路径距离。

#### low_level_traj_plan

返回：

{
    "Depot0": LinearTrajectory(...),
    "Depot1": LinearTrajectory(...),
    "Depot2": LinearTrajectory(...)
}

每个 LinearTrajectory 包含：

- 时间范围；
- 一系列 (区域名, 时间, 坐标) 航点；
- value(t)：给定时间返回无人机位置；
- end_time：该无人机任务结束时间。

### 最终文件输出

generate_intensive_traj_waypoints() 为每个 Depot 生成两个 CSV。

#### 1. 局部 ENU 航点

目录：

intensive_waypoints/local_enu/

文件示例：

Depot0_intensive_waypoints_enu.csv

字段：

Depot, Vertex, Time, East, North, Up

示例：

Depot0,Polygon17,12.34,7.52,28.41,0

#### 2. 经纬高 LLH 航点

目录：

intensive_waypoints/llh/

文件示例：

Depot0_intensive_waypoints_llh.csv

字段：

Depot, Vertex, Time, Latitude, Longitude, Altitude

这些文件可以作为后续无人机控制、航迹跟踪或任务下发的输入。

### 可选图形输出

如果用户在主程序中选择绘图：

- Power_Plant_Env.pdf：区域凸集和连接图；
- Power_Plant_Env_with_points.pdf：加入 Depot 和 Checkpoint 的场景图；
- Power_Plant_Inspection_Solution_Static.pdf：静态巡检轨迹图。

动画函数返回 Matplotlib FuncAnimation 对象。主程序当前设置 save=None，因此默认不会保存动画；项目中已有的 MP4 文件属于已有示例结
果或 Notebook 生成结果。


## task1
使用 uv 创建该项目的环境，并复现当前输入结果(原先结果不要覆盖)。

配置 Mosek 许可证后，可执行：
```
cd /home/jazzy/py/GCS_Multi_Inspecton/reproduced_2026-08-03
MPLBACKEND=Agg PYTHONPATH=.. ../.venv/bin/python ../Inspection_wrapper.py
```
输入 n、n 即可生成新的航迹 CSV，仍只会写入该复现目录。


问题:
1. 我添加了 Mosek 许可证，现在 Inspection_wrapper.py 已经可以运行，我已经按照你说的跑了一轮复现。
请分析复现结果是否和原结果一致。
### 2. depot 是否可以理解为机器人单位，比如当前是 3 个机器人，是否可以增加到 5 个或者 10 个
- 在哪里查看分配给某个 depot 哪些检查点？检查点的顺序如何？

• 可以。这里的 depot 基本可以理解为一个机器人单位的起始位置：
- Depots.csv 中每一行对应一个机器人/depot。
- 程序通过 len(depots_nh.keys()) 自动得到机器人数量，见 Inspection_wrapper.py:129。
- 因此可以把 Depot3、Depot4 等继续加入 Depots.csv，理论上支持 5 个或 10 个机器人。
- 新 depot 必须位于某个有效区域内，否则无法连接到图。
- 检查点会被约束为恰好分配给一个 depot，见 spp/high_level_inspection.py:30。

需要注意：动画入口目前传入了固定的 8 个颜色，增加到 10 个机器人时需要改成 colors=None，或提供至少 10 个颜色，否则会触发颜色数量
断言，见 Inspection_wrapper.py:594。

查看分配结果：
```
cost_d2c, cluster_matrix, cluster_dict = assign_checkpoints_to_depots(
    ...
    verbose=True,
)
```
该结果中的 cluster_dict 就是分配关系：
```
{
    "Depot0": ["Checkpoint7", "Checkpoint3", ...],
    "Depot1": ["Checkpoint5", "Checkpoint0", ...],
    "Depot2": ["Checkpoint1", "Checkpoint2", ...],
}
```
分配逻辑位于 spp/high_level_inspection.py:11。这里列表的顺序只是检查点分配结果的存储顺序，不一定是实际巡检顺序。

查看实际巡检顺序：
```
dist_matrix_dicts, depot_tsp_sols = high_level_tsp(
    ...
    verbose=True,
)
```
实际顺序在：
```
depot_tsp_sols["Depot0"][0]
```
它返回类似：
```
["Depot0", "Checkpoint7", "Checkpoint3",
"Checkpoint8", "Checkpoint6", "Depot0"]
```
TSP 路径生成位于 spp/high_level_inspection.py:123，入口调用位于 Inspection_wrapper.py:542。

生成的航迹 CSV 也可以查看 Vertex 列，但同一个检查点会因为时间采样重复多行；实际顺序应取 Vertex 列中检查点名称的首次出现顺序。


3. 我在 @/home/jazzy/py/GCS_Multi_Inspecton/reproduced_2026-08-03/intensive_waypoints/llh 查找检查点顺序，在哪里有？

### 4. 我看目前输出的航迹点很密集，这个密集程度是在哪里/哪个参数决定的？密集程度会不会影响求解效率，还是只是简单的切分？

• 目前的密集程度由入口中的 round_digits=2 决定：
```
low_level_traj_plan(..., round_digits=2)
generate_intensive_traj_waypoints(..., round_digits=2)
```
见 Inspection_wrapper.py:566 和 Inspection_wrapper.py:573。

在 generate_intensive_traj_waypoints() 中：
```
time_step = 10 ** (-round_digits)
```
因此当前：
```
round_digits = 2
time_step = 0.01 秒
```
也就是每 0.01 s 采样一个点。航迹速度约为 1 m/s，所以大约每厘米一个点，CSV 会非常大。

具体采样逻辑在 spp/linear_trajectory.py:306，它只是调用已经生成好的轨迹，在时间轴上插值采样，并保留原始关键航迹点。

可以调整为：
```
round_digits=1  # 0.1 秒一个点
round_digits=0  # 1 秒一个点
round_digits=3  # 0.001 秒一个点
```
密集采样本身不会影响 Mosek、TSP 或路径规划求解效率，因为它发生在所有求解完成之后。它主要影响：

- CSV 文件大小
- 写文件时间
- 内存占用
- 后续读取和传输
- 若用这些点制作动画，数据准备时间

有一个细节需要注意：当前同一个 round_digits 还传给了 spp/linear_trajectory.py:42，用于对求解出的时间和坐标进行四舍五入。因此修
改它不仅会改变输出采样间隔，也会改变低层轨迹数据的数值精度。不过这不会增加求解规模，只可能造成很小的数值差异。


## task2
### @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB 是 @/home/jazzy/py/GCS_Multi_Inspecton 替换了求解器的版本:
```
Drake提供了一个MixedIntegerBranchAndBound类可以把混合整数规划放松到连续，然后底层调用连续求解器求解，我就把原来用Mosek求解的几个地方改成用Clarabel和CLP求解了
```
请查看其实现相较原本版本更改了什么，是否正确合适？

• 不建议将该版本视为“无需 Mosek 的等价替换”后直接采用。MixedIntegerBranchAndBound 的使用方向是对的，但实现仍保留了实际的 Mosek 依赖，并且存在可复现性与功能回归问题。

1. 高优先级：实际上仍依赖 Mosek，README 的说明不成立。
    /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/spp/convex_sets.py:7 仍导入 MosekSolver，并在多处实际调用。尤其 /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/spp/convex_sets.py:310 的相交性测试由 graph_construction() 调用，见 /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/Inspection_wrapper.py:125。没有许可证时，程序会在进入 Branch-and-Bound 之前失败。

2. 高优先级：依赖文件无法创建正确环境。
    /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/requirements.txt 仍声明 Mosek==11.2.2，且 pydrake==0.2.1 不是 MIT Drake，缺少 pydrake.solvers.MixedIntegerBranchAndBound。因此新的干净环境无法按该文件运行；应改为实际 Drake 包及经过验证的版本约束。

3. 中优先级：此版本不再是原多智能体求解器的直接替代。
    原 MultiAgentGraphOfConvexSets3PointsInspection 被替换成单智能体 GraphOfConvexSets3Points，例如 /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/Inspection_wrapper.py:128。当前工作流仍能通过“分配后逐个 depot、逐段路径”完成多机器人任务，但不再支持一个联合优化问题内的多智能体变量、不同机器人 v_max 列表或潜在协同约束。当前传入标量 v_max=1 时可工作。

4. 中优先级：没有失败保护与求解预算。
    低层 MISOCP 用 Clarabel 做连续松弛是合理的，见 /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/spp/
    shortest_path_3point_time_MIBNB.py:463；高层两个纯 MILP 用 CLP 是合理的，见 /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/spp/high_level_inspection_MIBNB.py:51。但 Branch-and-Bound 没有时间限制、节点限制、gap 或日志配置。规模上升时，求解时间会指数增长，不能指望它替代商用 MIP/MISOCP 求解器的性能。

5. 低优先级：新图类中有未覆盖的运行时错误。
    /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/spp/graph_of_convex_sets_3points.py:107 遍历的是检查点字典的名称，却把名称字符串当作二维坐标访问；启用对应绘图会出错。/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/spp/graph_of_convex_sets_3points.py:51 写入了不存在的 inspection_sets。/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/spp/graph_of_convex_sets_3points.py:228 使用不存在的 self.n_agents。

结论：求解器替换的核心思路可行，适合小规模、无许可证的探索；但当前版本尚不适合声称“去除 Mosek”或作为原实现的可靠替代。首先应彻底移除 convex_sets.py 中的 Mosek 调用、修正依赖声明，并为典型输入加入路线可行性、检查点覆盖、目标值和耗时的回归测试。

问题:
1. 使用 uv 在 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB 下创建虚拟环境，并复现当前输入结果(原先结果不要覆盖)
```
cd /home/jazzy/py/GCS_Multi_Inspecton/reproduced_2026-08-03
MPLBACKEND=Agg PYTHONPATH=.. ../.venv/bin/python ../Inspection_wrapper.py
```

目前还是无法脱离 MosekSolver 运行
```
File "/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/spp/convex_sets.py", line 311, in strict_intersects
result = solver.Solve(prog)
             ^^^^^^^^^^^^^^^^^^
ValueError: MosekSolver cannot Solve because MosekSolver::enabled() is false, i.e., MosekSolver has not been properly configuredfor use. Typically this means that an environment variable has not been set. Refer to the MosekSolver class overview documentation for how to enable it.
```

### 2. 请在 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB 中彻底去除对 MosekSolver 的依赖，并在虚拟环境中去除该依赖项，另外:
- 说明你是如何修改的
- 实际测试通过

已在 /home/jazzy/py/GCS_Multi_Inspecton_MIBNB 完成去 Mosek 化，并在 .venv 中移除了 mosek 包。

修改内容：
- /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/spp/convex_sets.py
将所有 MosekSolver().Solve(...) 替换为 ClarabelSolver().Solve(...)，用于 polyhedron center、intersection、feasibility 等连续凸/线性检查。

- /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/spp/shortest_path.py
移除 MosekSolver，改为：
    - relaxation 模式：ClarabelSolver
    - binary/MIP 模式：MixedIntegerBranchAndBound(..., ClarabelSolver.id())

- /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/spp/shortest_path_3point_time_MIBNB.py
清理了旧 Mosek/Gurobi 相关注释和未使用 import，保留 MixedIntegerBranchAndBound + Clarabel。

- /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/requirements.txt 和 /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/requirements_long.txt 删除显式 Mosek==...，并把错误的 pydrake==0.2.1 改为实际可用的 drake==1.55.0。

- /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/README.md 更新为 Drake Branch-and-Bound + Clarabel/CLP 的说明，不再描述 Mosek。


3. 你修改后的 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB 结果是否和原先 @/home/jazzy/py/GCS_Multi_Inspecton 匹配:
```
Depot0 (Assigned 4 checkpoints) with total load: 305.03 -> Checkpoint assigned to it is: ['Checkpoint3', 'Checkpoint6', 'Checkpoint7', 'Checkpoint8']
Depot1 (Assigned 3 checkpoints) with total load: 225.38 -> Checkpoint assigned to it is: ['Checkpoint0', 'Checkpoint5', 'Checkpoint9']
Depot2 (Assigned 3 checkpoints) with total load: 270.63 -> Checkpoint assigned to it is: ['Checkpoint1', 'Checkpoint2', 'Checkpoint4']
```


### 4. 我运行 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB，发现运行速度较慢，且 cpu 调度较低，请查找运行慢的原因(或者实现本身就慢)，分析提高算法运行速度的方法。先不要修改源码。

CPU 调度低也合理：当前代码是串行循环，每次只启动一个 Python 进程内的一个 Branch-and-Bound 求解。你看到的“低 CPU”大概率是按整机多核百分比显示的；单进程实际常见是占一个核约 100%。Drake 的 MixedIntegerBranchAndBound 本身是通过 branch-and-bound 反复解连续松弛，文档也说明它会在树节点中固定二元变量并解剩余问题；当前代码没有把这些 65 个独立子问题并行化。Drake 另有 SolveInParallel 可并行解多个独立 MathematicalProgram，但这里没有使用。(drake.mit.edu (https://drake.mit.edu/doxygen_cxx/classdrake_1_1solvers_1_1_mixed_integer_branch_and_bound.html)) (drake.mit.edu
(https://drake.mit.edu/doxygen_cxx/namespacedrake_1_1solvers.html))

主要慢点：
1. Branch-and-Bound 替代商用 MISOCP 求解器后天然更慢。
    原 Mosek 是成熟的集成式 MIP/MISOCP 求解器；现在是 Drake 的通用 B&B 外壳，每个节点调用 Clarabel 解连续二阶锥松弛。对这种 GCS MISOCP，开源组合通常会慢不少。

2. 重复求解次数太多。
    depot-checkpoint 距离、TSP 距离矩阵、低层轨迹段都重新构建并重新求解相似图问题。很多 depot-checkpoint 距离在分配阶段算过，后面距离矩阵里又算一遍。

3. 每个 GCS 子问题规模不小。
    日志显示每个低层问题大约 834 个变量、1134 个约束；跑 65 次就是主要成本。

4. Big-M 时间约束可能较松。
    /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/spp/shortest_path_3point_time_MIBNB.py:250 用全图最大边长和节点数估计 M_time，通常偏保守。Big-M 越松，连续松弛越弱，B&B 越容易多分支。

5. 图构建也有额外连续求解，但不是主耗时。
    graph_construction() 对 28 个区域做有向两两相交检查，最多 28*27=756 次 strict_intersects，见 /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/Inspection_wrapper.py:124。这些现在也走 Clarabel，可产生额外开销，但现有日志里主耗时仍是 65 次 B&B。

提高速度的方向，按收益排序：

1. 并行化 65 个独立子问题。
    depot-checkpoint 成本、TSP 距离矩阵中的 pairwise distance、低层 segment 都天然可并行。最直接是用 multiprocessing 或 joblib 按进程并行；Drake 也提供 SolveInParallel 用于并行解多个 MathematicalProgram，文档说明它可按 parallelism 使用多核，并支持动态调度。(drake.mit.edu (https://drake.mit.edu/doxygen_cxx/namespacedrake_1_1solvers.html))

2. 缓存距离结果。
    把 (start, end, objective params) 对应的 shortest-path cost/solution 缓存起来。当前 Depot -> Checkpoint 在分配和后续距离矩阵中存在重复计算，checkpoint-checkpoint 距离也可以在同一输入下复用。

3. 先只算 cost，必要时才算完整 trajectory。
    分配和 TSP 只需要距离成本，不需要 x_enter/x_leave 的完整可视化轨迹。可以做轻量 cost-only 问题，最后仅对实际采用的 13 段求完整轨迹。

4. 收紧 M_time。
    当前 M_time 是全图保守上界。可以按 source-target、候选子图或已知几何距离给更紧的上界，通常能显著减少 B&B 搜索树。

5. 减少二元变量规模。
    当前每个子问题把全图 60 条边都纳入。可先用图搜索/几何邻接裁剪，只保留 source-target 可能路径附近的区域和边，降低每次 MISOCP 的变量与约束数。

6. 分层近似。
    对分配/TSP 阶段，先用区域图最短路或欧氏距离近似代替完整 GCS 求解；只在最终低层轨迹阶段用 GCS 精解。这会牺牲一点严格性，但速度提升通常最大。

7. 可选求解预算。
    MixedIntegerBranchAndBound::Options 支持
    max_explored_nodes，可以限制搜索节点数，换取更快但可能非全局最优/失败的结果。(drake.mit.edu
    (https://drake.mit.edu/doxygen_cxx/structdrake_1_1solvers_1_1_mixed_integer_branch_and_bound_1_1_options.html))

我的判断：当前慢是预期内的。Mosek 版换成 Drake B&B + Clarabel 后，如果不并行、不缓存、不裁剪，运行时间增加很正常。最值得先做的是“并行 + 缓存”，因为不改变数学模型；其次是收紧 M_time 和裁剪图。


问题:
### 5. 按照你的思路对 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB 进行“并行 + 缓存”的优化，并验证结果

已完成“并行 + 缓存”优化，并做了完整验证。

改动集中在两个文件：
- /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/Inspection_wrapper.py
    - 增加 INSPECTION_WORKERS worker 配置，默认最多 4 个进程。
    - depot-to-checkpoint 成本计算改为 ProcessPoolExecutor 并行。
    - high-level TSP 调用时传入 cost_d2c，复用前面已经算过的 depot-checkpoint 距离。
    - low-level 每个 depot 的完整轨迹规划改为进程并行。
    - 保留 draw_graphs=True 时的串行路径，避免 matplotlib 绘图并行风险。

- /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/spp/high_level_inspection_MIBNB.py
    - build_subgraph_dist_matrix() 增加 distance_cache。
    - 已缓存的 depot-checkpoint pair 不再重复求解。
    - 未缓存的 checkpoint-checkpoint pair 使用进程池并行求解。

验证结果：
- compileall 通过。
- 无 Mosek 环境下运行通过。
- 完整复现目录：/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/reproduced_process_pool_2026-08-04_run1
- exit_status=0
- 生成 6 个航迹 CSV。
- run.log 中所有 GCS 求解均为 Solution feasible: True。
- 墙钟时间：45s

对比之前：
- 原无缓存版本：65 次 GCS B&B 求解，优化耗时合计约 95.2s。
- 缓存后：55 次 GCS B&B 求解，减少 10 次重复 depot-checkpoint 距离求解。
- 线程池版本/串行缓存版本约 101-103s，没有明显收益。
- 进程池版本降到 45s，这是有效提速点。

使用方式：
```
cd /home/jazzy/py/GCS_Multi_Inspecton_MIBNB
INSPECTION_WORKERS=4 MPLBACKEND=Agg PYTHONPATH=. .venv/bin/python Inspection_wrapper.py
```
如果机器核心数较少，可设 INSPECTION_WORKERS=2；如果想完全串行复现，可设 INSPECTION_WORKERS=1。


### 6: 请审核你刚才的更改"并行+缓存"，修改是否符合理论和具备正确性？并且分析为什么这种规划方法运行速度慢，这样能满足实时性要求吗？你有什么建议？

根据你的思路完成以下任务，并验证通过：
1. 将 v_max 传入所有高层点对计算。
2. 缓存键加入 v_max、alpha、beta、cyclic 和地图/检查点指纹。
3. 明确记录对称性前提，或者改为有向缓存。
4. 加入 1/2/4 worker、缓存开关和不同 v_max 的回归测试。
5. 复用长生命周期进程池，并通过 worker initializer 保存只读图数据，减少序列化。


主要修改：
- 所有高层点对求解现在显式接收 v_max，不再写死为 1。
- 缓存键包含：source、target、v_max、alpha、beta、cyclic、输入 SHA-256 指纹。
- 缓存改为有向键；当前欧氏距离模型通过 symmetric_costs=True 显式声明对称。非对称模型可关闭该选项。
- 地图指纹覆盖凸集、边、长度函数、depot、checkpoint 及其所属区域。
- 新增共享 worker context，完整图数据只在进程初始化时传入一次，任务仅传点名和参数。
- 同一个进程池贯穿检查点分配、TSP 距离计算和低层航迹规划。
- 支持：
    - INSPECTION_WORKERS=1/2/4
    - INSPECTION_CACHE=0/1


7. 现在 inspection_wrapper.py 的输出不太规范，直接输出在项目目录下。能否改为输出在 /output 下的带日期、时间戳的文件夹下(相关文件夹自动生成)？并且在该文件夹下同时输出"输入参数列表"、"规划结果"等信息。

8. @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/intensive_waypoints 下是之前算法生成的结果(这个我只想存档); @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/Boundary.csv 等文件都是 input 需要的，但这些的太零乱了。你觉得如何修改合适？先不要修改源码。

9. 像当前 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/output/20260804_055407_963096/planning_results.json 的输出结果太冗余了，只要输出关键信息就可以了，并且需要记录总体规划时间

10. @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/configs/power_plant.json 我看这里 ""workers": null"，但似乎也是多进程运行。是我理解有误还是实现有问题？

11. 我想将输出的经纬度坐标在真实的 GIS 地图上显示出来，你有什么好的方案？先讨论，不要修改源码。
先明确希望使用的底图服务和坐标系，然后以 GeoPackage + WGS84 GeoJSON 作为标准输出，保留现有 GCJ-02 CSV 作原始记录。这样既能在 QGIS 中严谨核对，也能直接服务网页地图。

## task2
### 1. 如果我使用谷歌地图和相应坐标系作为底图方案，适配方案如何设计比较好？geojson 和 gpkg 分别是什么格式？先讨论，不要修改源码。

可以，建议把“规划坐标”和“地图显示坐标”分成两层处理，不要直接修改原始输出。

当前输出坐标被说明为 GCJ-02，而 Google 地图通常使用 WGS84 地理坐标，并在网页显示时投影到 Web Mercator（EPSG:3857）。因此推荐流程是：
```
规划输出 LLH（保留 GCJ-02）
        |
        | 受控的 GCJ-02 -> WGS84 转换
        v
WGS84 GeoJSON / GeoPackage
        |
        v
Google Maps 底图显示
```
需要注意：

- 不要只把 GCJ-02 的数值标记成 EPSG:4326，这只是改标签，不能消除偏移。
- 应使用经过验证的 GCJ-02 到 WGS84 转换方法；在中国区域还应确认数据使用和转换是否符合相关规定。
- Google 地图网页通常只需要传入 longitude, latitude 的 WGS84 坐标，地图内部会处理到 EPSG:3857。
- 原始 GCJ-02 CSV 应继续保留，便于复现和审计。
- 可以同时输出 trajectory_gcj02 和 trajectory_wgs84 两套数据，避免后续无法判断坐标来源。

针对 Google Maps，推荐的显示结构是：

- 每个 depot 一条按时间排序的 LineString
- 检查点为 Point
- depot 起点/终点为单独的 Point
- 属性中保留 depot_id、route_order、timestamp、altitude、checkpoint_id
- 需要动画时，再保留每个轨迹点的时间字段

#### GeoJSON 是什么

GeoJSON 是基于 JSON 的轻量空间数据格式，适合 Web 地图和接口传输。例如：

{
"type": "Feature",
"geometry": {
    "type": "LineString",
    "coordinates": [
    [116.123, 39.456],
    [116.124, 39.457]
    ]
},
"properties": {
    "depot_id": 1
}
}

优点：

- Google Maps、Leaflet、MapLibre 等前端容易加载
- 文本格式，便于检查和接口传输
- 适合轨迹、点、区域等简单图层

限制：

- RFC 7946 规定其坐标语义为 WGS84 经纬度
- 不适合存储复杂工程数据、多个图层、栅格数据
- 文件较大时性能和属性管理不如数据库格式
- CRS 通常不能依靠文件内部声明解决，因此必须在文件名或元数据中明确说明

#### GPKG 是什么

GeoPackage（.gpkg）是基于 SQLite 的 OGC 标准空间数据库文件。一个文件中可以保存多个图层及其坐标系、属性、空间索引和元数据。

可以在一个 planning.gpkg 中保存：

- depots
- checkpoints
- trajectories
- trajectory_points
- regions

优点：

- 支持明确的 CRS 定义
- 支持多个图层
- 支持属性字段、空间索引和较大数据量
- QGIS、ArcGIS、GDAL 等工具支持良好
- 适合作为正式交付和归档格式

限制：

- Google Maps 前端不能像 GeoJSON 一样直接加载，通常需要后端或预处理转换
- 不如 GeoJSON 方便手工查看和网络传输

#### 推荐方案

如果目标是快速在 Google 地图网页中查看：

原始 GCJ-02 CSV
        -> 转换为 WGS84
        -> trajectory_wgs84.geojson
        -> Google Maps JavaScript API Data Layer 加载

如果还需要 GIS 分析和长期归档：

原始 CSV
        -> planning.gpkg
            ├── depots
            ├── checkpoints
            ├── trajectories
            └── trajectory_points

实际项目中可以同时提供：

- trajectory_wgs84.geojson：Google Maps 网页显示
- planning.gpkg：QGIS/ArcGIS 分析和归档
- 原始 GCJ-02 CSV：规划复现和审计

最终需要先确认一件事：你使用的是 Google Maps 网页/API、Google Earth，还是 QGIS 加载 Google 底图。三者对数据导入方式不同，但都应优先使用经过验证的 WGS84 数据，而不是直接将当前 GCJ-02 数值当作 WGS84。


问题:
### 2. 我想的是先使用 Google Maps 网页输出坐标集合，但我有几个问题:
- 输出的坐标如何到 Google Maps 网页上，有明确的方案吗，谷歌是否提供接口？需要付费吗？
- 当前输入坐标使用 GCJ-02，但以后可能有 WGS84 坐标集合，如何设计架构？


#### GCJ-02 和 WGS84 的架构设计
不要让算法代码直接假设所有输入都是某一种坐标系。建议增加明确的坐标参考信息：
```
{
"coordinates": {
    "crs": "GCJ-02",
    "axis_order": "lon_lat",
    "vertical_datum": "unknown"
}
}
```
每个输入场景都显式声明：
- GCJ-02
- WGS84
- 其他坐标系
- 经纬度顺序
- 高程单位和高程基准

推荐采用分层架构：
```
原始输入
|
| 坐标系识别、校验、转换
v
统一内部坐标
|
| 规划算法使用 ENU / 局部米制坐标
v
规划结果
|
| 按目标平台转换
v
Google Maps: WGS84
QGIS/国内底图: 根据需要输出 WGS84 或 GCJ-02
```
其中有两个实际选择。

#### 选择 1：统一内部使用 WGS84
输入 GCJ-02 时，在进入规划前转换为 WGS84，然后再建立 ENU 坐标。

优点：
- 国际 GIS、Google Maps、QGIS 更一致
- 后续接入 OSM、Cesium、Mapbox 更简单
- 混合输入更容易管理

缺点：
- 会改变当前基于 GCJ-02 数值运行的历史结果
- GCJ-02 到 WGS84 不是普通 EPSG 投影转换，而是区域偏移转换
- 必须验证转换精度和使用合规性


#### 说明
实现:
1. 不要让算法代码直接假设所有输入都是某一种坐标系。建议增加明确的坐标参考信息(WGS84、GCJ-02等)
2. 统一内部使用 WGS84: 输入 GCJ-02 时，在进入规划前转换为 WGS84，然后再建立 ENU 坐标
- 当前算法输出结果不重要
3. 输出时可以选定 WGS84 或者 GCJ-02，默认为 WGS84
4. 这些坐标转换相关抽离出来，不要和算法耦合，放入独立模块中
5. 测试验证通过

问题:
我看你新增实现中有大量公式，pymap3d(该依赖项已包含)无法完成吗，它的计算结果是否会更可靠？

3. 请使用谷歌地图作为底图，能将实现类似方案 A：Google Maps JavaScript API  
生成一个网页，使用 Google Maps JavaScript API 创建地图，再加载规划生成的 GeoJSON 或坐标数组：
```
const map = new google.maps.Map(document.getElementById("map"), {
center: { lat: 39.456, lng: 116.123 },
zoom: 16
});

map.data.loadGeoJson("trajectory_wgs84.geojson");
```
适合：
- 按 depot 显示不同颜色
- 点击轨迹点查看属性
- 播放轨迹
- 后续增加筛选、时间轴、动画

问题:
- 为什么一定需要 GOOGLE_MAPS_API_KEY 吗? 我该去哪里获取,如果没法获取,有别的替代方案吗?
- /tmp/gcs_mibnb_google_maps_validation/20260804_092232_015898 这个产物是什么？为什么要生成在 /tmp 下


问题
1. 我将 GOOGLE_MAPS_API_KEY 回退了，因为这不是我期望的
2. 我发现之前的那次更改"feat: add GCJ-02 <-> WGS84."，导致"'/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/output/20260804_102657_594962/figures/Power_Plant_Inspection_Solution_Static.pdf"上的轨迹是错误的，请分析排查，我是基于当前的 config 运行
3. 将"round_digits"拆成两个独立配置：一个仅控制低层轨迹数值保留精度，另一个仅控制导出采样间隔
4. 我看输出的 csv:
```
Depot,Vertex,Time,East,North,Up
Depot0,Depot0,0.0,7.09,28.23,0
Depot0,Depot0,0.0,7.09,28.23,0
Depot0,Depot0,0.0,7.09,28.23,0
Depot0,Polygon17,0.0,7.09,28.23,0
Depot0,Polygon17,2.07,9.12,28.64,0
```
Vertex 这一列代表什么，Polygon17 又是什么？能否举例说明

5. 待解决项:
- @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/data/scenarios/power_plant 下的 csv 首行均有乱码，可以改为英文
- output 里目前列顺序为"Latitude, Longitude"，希望以后输出改为"Longitude, Latitude"，并运行通过

6. 我找到了一个坐标显示软件"https://www.gpsov.com/"，里面可以在 GIS 地图上快速导入坐标点，我需要你增加在 output/时间戳 下的输出，说明:
- 格式使用 txt 即可
- 将 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/data/scenarios/power_plant/input/Regions.csv 这种转换输出，格式类似:
```
g119.97940555,30.32500590
g119.97941226,30.32497580
g119.97944444,30.32498043
g119.97944310,30.32501169
&
g119.97943908,30.32501285
g119.97976631,30.32506147
g119.97977167,30.32502905
g119.97944444,30.32498275
```
说明:
```
1. g 代表 gcj-02 坐标系，没有 g 则默认 WGS84 坐标系
2. & 代表两块不同区域的表示
3. 119.97940555,30.32500590 代表 Longitude,Latitude
```
- @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/data/scenarios/power_plant/input/Checkpoints.csv 需要转换输出
- @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/data/scenarios/power_plant/input/Depots.csv 需要转换输出
- 规划输出的经纬度坐标需要类似上述形式转换输出，不同 depots 按照 & 隔开即可

7. 配置里这么填写:
```
"coordinate_reference": {
"input_crs": "GCJ-02",
"output_crs": "WGS84",
"record_markers": ["g"]
},
```
那么实际输出的轨迹(gpsov、intensive_waypoints)前缀是带 g 的吗？

8. intensive_waypoints/llh 这种输出文件夹下最好改为明确说明是哪个坐标系，你觉得呢

### 9. 当前测试场景比较单一，我想增加测试，说明:
- 在 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/data/scenarios/power_plant/input 增加不同 input 文件
- Boundary、Regions 先不变
- 不要在原有 config 文件里修改，但可以在 config 文件夹下增加配置项
针对以下目标增加不同场景:
- Depots，增减个数或改变点位
- checkpoints，增减个数或改变点位
请增加情形，并测试通过


新增输入文件：
- Depots_2.csv：2 个 depot
- Depots_5.csv：5 个 depot
- Depots_shifted.csv：3 个位置调整后的 depot
- Checkpoints_5.csv：5 个检查点
- Checkpoints_15.csv：15 个检查点
- Checkpoints_shifted.csv：10 个位置调整后的检查点

新增配置：
- /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/configs/power_plant_two_depots.json
- /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/configs/power_plant_five_depots.json
- /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/configs/power_plant_five_checkpoints.json
- /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/configs/power_plant_fifteen_checkpoints.json
- /home/jazzy/py/GCS_Multi_Inspecton_MIBNB/configs/power_plant_shifted.json

示例运行：
```
INSPECTION_CONFIG=configs/power_plant_five_depots.json \
.venv/bin/python Inspection_wrapper.py
```
测试覆盖了配置加载、坐标读取、depot/checkpoint 数量检查和图构建。虚拟环境中共 20 项测试全部通过。

问题:
1. 运行:
```
INSPECTION_CONFIG=configs/power_plant_five_depots.json \
.venv/bin/python Inspection_wrapper.py
```
出现错误:
```
File "/home/jazzy/.local/share/uv/python/cpython-3.12-linux-x86_64-gnu/lib/python3.12/concurrent/futures/_base.py", line 401, in __get_result
raise self._exception
RuntimeError: No feasible path from Depot3 to Checkpoint0
```
是算法的问题还是点位的问题，请分析排查，先不要修改源码

按照你的思路完成:
1. 将 Depots_5.csv 中的 Depot3、Depot4 调整到现有 Regions.csv 的多边形内部。
2. 增加输入校验：任何 depot 或 checkpoint 若不属于 

问题:
1. @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/configs/power_plant_fifteen_checkpoints.json 我运行这个配置发现输出图片中 15 个 checkpoints 有些几乎重叠，是选点的问题，还是输出的问题？

2. 它和 input 里的 "Coordinate Marker: g" 相关, "g"代表 GCJ-02 坐标系,"w"代表 GCS-84 坐标系,能否直接将 record_markers 删除,因为第一行(说明行)是否直接过滤比较好，请分析。

3. 我觉得以"Coordinate Marker"为中心去转换坐标系过于繁琐(需要逐一识别坐标系)，我觉得应该以 input_crs 为中心，而不管 "Coordinate Marker"，你觉得呢？因为人为应该不能出现坐标系混合的情况。

4. @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/data/scenarios/power_plant 在这个同级目录下生成一个文件夹，里面坐标转换成 WSG-84 坐标系

5. 我查看 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/output/20260805_025931_331702/intensive_waypoints/llh_gcj02/Depot1_intensive_waypoints_llh.csv，发现有很多重复点位:
```
Depot1,Polygon8,318.21000000000004,119.97941229626889,30.32497578025462,0.0049444251070217115
Depot1,Polygon1,318.21000000000004,119.97941229626889,30.32497578025462,0.0049444251070217115
Depot1,Polygon1,318.21000000000004,119.97941229626889,30.32497578025462,0.0049444251070217115
Depot1,Polygon1,318.21000000000004,119.97941229626889,30.32497578025462,0.0049444251070217115
Depot1,Polygon0,318.21000000000004,119.97941229626889,30.32497578025462,0.0049444251070217115
```
这是什么原因，你觉得合理吗？

6. 当前每次启动 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/Inspection_wrapper.py，算法完成后都要输入两次 y/n，现在改为 picture 默认为 y，animate 默认为 n，减少人机交互

### 7. 当前规划轨迹 depot1 从当前位置开始巡检，最后会规划返回原点。我觉得不要返回了，比如 depot1 分配到 3、6、9 三个点，它走完这几个点就可以了，不用返回。你觉得呢？

从业务描述看，如果任务目标只是“从 depot 出发完成分配的检查点巡检”，那么不返回 depot 更合理，可以减少航程、时间和能耗。

当前实现是闭环路径：
```
Depot1 -> Checkpoint0 -> Checkpoint5 -> Checkpoint9 -> Depot1
```
其中返回 depot 主要由高层 TSP 的闭环设定和低层 cyclic=True 决定。它不是绘图或导出问题，而是规划模型的目标定义。

是否取消返回取决于任务约束：
- 若巡检完成后机器人可以在最后一个检查点结束任务、降落或由其他系统接管：建议取消返回。
- 若 depot 是充电点、回收点、通信恢复点，或者必须回到固定位置：应保留返回。
- 若还要比较不同 depot 的长期循环任务：闭环更适合。
- 若是一次性任务或实时派遣：开放路径通常更合适。

建议增加一个配置项，例如：
```
"return_to_depot": false
```
语义为：
```
true  -> Depot -> checkpoints -> Depot
false -> Depot -> checkpoints
```
需要同步修改：
1. 高层 TSP 从闭环 TSP 改为开放路径 TSP。
2. 低层最后一段不再规划“最后检查点 -> Depot”。
3. 路径结果和 planning_results.json 明确记录是否返回。
4. 增加闭环/开放路径结果一致性测试。

需要注意：这会改变优化目标和规划结果，不能只删除输出文件中的最后几行，否则只是隐藏了返程，实际规划成本仍包含返程。

#### 实现过程

已完成开放路径规划，当前默认不返回 depot。

配置项：
```
"return_to_depot": false
```
已写入所有现有场景配置。设置为 true 时可恢复闭环：
```
Depot -> checkpoints -> Depot
```
当前 false 时为：
```
Depot -> checkpoints
```
实现使用高层 TSP 的虚拟终点，因此返程不参与优化成本，低层轨迹也不会生成最后检查点回 depot 的一段。

验证通过：
- 24 项测试全部通过。
- 默认配置完整运行成功：
```
/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/output/20260805_072442_125413
```
- planning_results.json 记录：
```
"return_to_depot": false
```
- Depot1 实际路径：
```
Depot1 -> Checkpoint0 -> Checkpoint9 -> Checkpoint5
```
- Depot1 航迹最后一个顶点为 Checkpoint5，时间 236.43 s，不再返回 Depot1。


测试:
1. 变电站 gcj-02  OK
2. 变电站 wgs84  OK

## task3
问题:
1. @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB 当前会依据 Regions.csv 进行规划，但这种区域是直接根据卫星地图扣图得到的。所以很多拐点其实和建造物边缘重合。是否加入一定膨胀体积比较好？请分析是否合理和可行性，不要修改源码，但需要注意:
- 膨胀体积方向问题，如果方向错了，会导致更差的结果
- 膨胀体积设置问题，多少膨胀体积合适

结论：加入安全裕量是合理且可行的，但方向必须由 Region 的语义决定。对于当前数据，建议先确认 Region 是“可飞区域”还是“建筑物区域”，再决定采用向内收缩、障碍物向外膨胀，还是建立建筑外侧巡检走廊。

### 2. 当前的 region 是可飞/可行区域,实施方案应该如何?

推荐方案如下。

首先不要逐个独立收缩每个 Region。当前 Regions.csv 很可能是由多个相邻多边形拼成的可飞区域分解。如果每个多边形都向内收缩，原本相邻的 Region 之间会产生缝隙，图上的连通边可能全部消失，导致路径不可行。

更稳妥的流程是：
```
Regions.csv
    ↓
构造可飞区域整体 Union
    ↓
按安全距离向内收缩
    ↓
重新进行凸分解
    ↓
重新建立 GCS 图和路径规划
```
也就是说，应该先把所有可飞 Region 合并为整体可行空间，再整体收缩，而不是逐块收缩。
安全距离应在内部 ENU 平面中以米计算：
```
安全距离 =
无人机水平尺寸半径
+ 定位误差
+ 地图误差
+ 控制跟踪误差
+ 风扰和额外安全余量
```
例如可以先测试：
```
0 m、0.5 m、1 m、2 m、3 m
```
然后比较：
- 是否仍然存在可行解；
- Depot 和 Checkpoint 是否仍在收缩后的可飞区域内；
- 最小边界距离；
- 总航程和规划时间；
- 是否出现 Region 断裂或路径明显绕行。

建议配置使用明确语义，例如：
```
"geometry": {
"free_space_margin_m": 2.0,
"join_style": "round"
}
```
不要使用含义模糊的 inflate_distance，因为对当前可飞区域来说实际是“向内收缩”。

还需要特别处理 Depot 和 Checkpoint：
- 如果点位在收缩后区域外，应立即报错；
- 或者根据业务规则将其投影到最近的可飞区域内部；
- 不能继续让求解器使用无效点位。

最后，安全收缩解决的是边界安全距离问题；如果拐点主要来自 Region 凸分解本身，还应在低层增加轨迹平滑或圆角约束。单纯收缩区域不会自动消除所有几何拐点。

问题:
1. 运行显示，但我的膨胀距离设置得很低了(1m):
```
  File "/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/Inspection_wrapper.py", line 925, in <module>
    backgroundG, sets, vertices, edges, l, depots_in, checkpoints_in = graph_construction(
                                                                       ^^^^^^^^^^^^^^^^^^^
  File "/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/Inspection_wrapper.py", line 391, in graph_construction
    raise ValueError(
ValueError: Every depot and checkpoint must belong to at least one Region; unassigned objects: depot Depot2 at ENU (71.578931, 267.307920); checkpoint Checkpoint3 at ENU (102.792024, 60.699687); checkpoint Checkpoint4 at ENU (68.225532, 244.593745); checkpoint Checkpoint5 at ENU (-11.865489, 168.880411); checkpoint Checkpoint6 at ENU (-2.579798, 116.522897)
```

2. 改成""free_space_margin_m": 0.5"后，出现错误:
```
INFO:drake:Clarabel returns InsufficientProgress
INFO:drake:Clarabel returns NumericalError
```

### 3. 我将膨胀体积这块回退了，因为我还要再考虑一下。现在我要添加一个重要功能。这个 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB 需要打包为 docker 镜像，以供"算法中心"(一个平台，上面会存储各种算法镜像和容器)调用，注意:
- 该容器需要对外暴露服务，从而被调用
- 可参考 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/tmp/算法接口文档.md，这是以前视觉算法和算法中心对接的接口
- 需要思考被调用的方式，输入输出(config、data、output)是 zip 打包的形式发送，还是以其他形式发送？
- 你建立 dockerfile，但不能实际构建，因为你现在已经位于一个容器中，我需要外部验证
请先分析方案，不要修改源码

建议将当前项目封装成“HTTP 算法服务 + 任务结果包”的形式，而不是让算法中心直接执行容器内的脚本。

#### 1. 推荐总体架构
```
算法中心
    │
    │ HTTP POST
    ▼
规划算法容器
    ├─ 接收任务
    ├─ 校验输入
    ├─ 执行规划
    ├─ 生成 timestamp 输出目录
    └─ 返回结果或异步回调
```
容器内部仍然使用现有规划代码，但需要增加一层服务适配器：
```
HTTP API
↓
请求解析与文件解包
↓
规划任务函数
↓
生成 output/<request_id>/
↓
结果压缩或结果下载
```
当前 Inspection_wrapper.py 是命令行入口，后续需要将其主要逻辑抽成可调用函数，例如：
```
run_planning(config_path, output_root, request_id)
```
HTTP 服务只负责调用这个函数，不应通过子进程反复启动脚本。

#### 2. 输入传输方式

推荐使用“JSON 请求 + ZIP 文件地址”，而不是把 ZIP 直接 Base64 放入 JSON。

1. 推荐方式
```
{
"algorithmCode": "gcs_multi_inspection",
"callType": "async",
"requestId": "req-20260805-001",
"callbackUrl": "http://algorithm-center/callback",
"requestParams": {
    "inputPackageUrl": "http://storage/input/req-20260805-001.zip",
    "config": {
    "planner": {
        "v_max": 1.0,
        "workers": 4
    },
    "coordinate_reference": {
        "input_crs": "GCJ-02",
        "output_crs": "WGS84"
    }
    }
}
}
```
ZIP 内可以包含：
```
input/
Origin.csv
Boundary.csv
Regions.csv
Depots.csv
Checkpoints.csv
config.json
manifest.json
```
其中：
- config.json：规划参数；
- manifest.json：文件清单、版本、校验值；
- input/*.csv：本次任务数据。

这样做的优点是：
- JSON 请求体较小；
- 大文件不需要 Base64，传输效率更高；
- 输入数据可以通过对象存储复用；
- 结果可以异步生成；
- 算法中心可以保存输入包和结果包，方便追溯。

如果算法中心暂时不支持对象存储，也可以提供备用接口：
```
POST /v1/plan
Content-Type: multipart/form-data

package: input.zip
request: request.json
```
不建议使用 Base64 ZIP，体积会增加约 33%，并且占用更多内存。

#### 3. 同步与异步接口

当前规划算法使用 Drake/Clarabel，计算时间可能较长，因此应以异步为主。

1. 提交任务
```
POST /v1/plan
```
返回：
```
{
"code": 202,
"msg": "任务已提交",
"requestId": "req-20260805-001",
"jobId": "job-abc123",
"status": "queued"
}
```
2. 查询状态
```
GET /v1/plan/job-abc123
```
返回：
```
{
"code": 200,
"jobId": "job-abc123",
"status": "running",
"progress": null
}
```
完成后：
```
{
"code": 200,
"jobId": "job-abc123",
"status": "succeeded",
"resultPackageUrl": "http://storage/output/job-abc123.zip",
"planningTimeSeconds": 12.48
}
```
如果提供 callbackUrl，任务完成后向回调地址发送相同的结果信息。

3. 同步模式

保留：
```
"callType": "sync"
```
同步请求直接返回结果摘要或结果 ZIP，但必须设置 HTTP 超时和最大执行时间。同步模式适合小规模测试，不建议作为生产默认模式。

#### 输出结果包

建议将当前输出目录整体压缩为：
```
output.zip
├─ input_parameters.json
├─ planning_results.json
├─ run_metadata.json
├─ intensive_waypoints/
│  ├─ local_enu/
│  └─ llh_wgs84/ 或 llh_gcj02/
├─ gpsov/
└─ figures/
```
其中：
- planning_results.json：分配结果、TSP 顺序、轨迹摘要、规划时间；
- run_metadata.json：请求 ID、开始/结束时间、总耗时、算法版本；
- intensive_waypoints：轨迹 CSV；
- gpsov：GPSOV 导入文件；
- figures：可选图像，生产环境可以通过参数关闭。

HTTP 返回内容不建议直接嵌入全部轨迹数据，而应返回：
- 结果摘要；
- 结果包下载地址；
- 规划耗时；
- 输入指纹；
- 错误信息。

#### 5. 建议的服务接口

容器至少暴露：
```
GET  /healthz
GET  /readyz
POST /v1/plan
GET  /v1/plan/{job_id}
GET  /v1/plan/{job_id}/result
POST /v1/plan/{job_id}/cancel
```
/healthz 只表示进程存活，/readyz 检查依赖和服务是否可以接受任务。

容器端口可以统一使用：
```
8080
```
Docker 层使用：
```
EXPOSE 8080
```
实际端口由算法中心通过容器编排映射。

#### 6. Docker 镜像建议

基础镜像建议优先考虑：
```
ubuntu:24.04
```
原因是 Drake、Clarabel、CLP、Matplotlib 等依赖通常比 python:slim 需要更多系统库。镜像构建时：
1. 安装 Python 3.12；
2. 安装 uv；
3. 使用 requirements.txt 安装依赖；
4. 复制源码；
5. 创建非 root 用户；
6. 使用 ASGI 服务启动；
7. 不复制本地 .venv、output 和历史归档结果。

启动形式建议类似：
```
uvicorn service:app --host 0.0.0.0 --port 8080
```
当前环境中不能实际构建镜像，因此后续只能提供 Dockerfile，并由外部 Docker 环境执行：
```
docker build -t gcs-multi-inspection:latest .
docker run --rm -p 8080:8080 gcs-multi-inspection:latest
```
#### 7. 必须考虑的安全和稳定性问题

输入 ZIP 不能直接信任，需要：
- 防止 ZIP 路径穿越；
- 限制压缩包大小；
- 限制解压后总大小；
- 只允许规定的 CSV 文件；
- 禁止配置文件引用容器外任意路径；
- 校验 CSV 首行和字段；
- 校验坐标系；
- 校验 depot/checkpoint 是否位于 Region；
- 每个任务使用独立临时目录；
- 使用 requestId/jobId 防止输出目录冲突；
- 设置任务超时和并发数；
- 失败时保留错误报告，不暴露完整内部 traceback。

由于当前算法是 CPU 密集型，建议生产环境限制每个容器同时运行的任务数，避免多个任务争抢 Clarabel/CLP 线程。

#### 8. 版本与可复现性
每个结果中应记录：
```
{
"algorithm": "gcs_multi_inspection",
"algorithm_version": "git-commit-or-release",
"request_id": "req-20260805-001",
"input_fingerprint": "...",
"config_fingerprint": "...",
"planning_time_seconds": 12.48
}
```
这样算法中心可以区分：

- 输入不同；
- 配置不同；
- 算法代码版本不同；
- 求解器版本不同。

总体建议是：采用旧接口文档中的 algorithmCode/requestParams/requestId/callbackUrl 外层协议，但把规划输入和输出设计为 ZIP/对象存储包，并以异步任务接口作为生产模式。

#### 具体实现
输入 ZIP 需要包含：
```
config.json
input/Origin.csv
input/Boundary.csv
input/Regions.csv
input/Depots.csv
input/Checkpoints.csv
```
服务启动：
```
uvicorn service:app --host 0.0.0.0 --port 8080
```
接口冒烟测试已通过：
```
GET /healthz -> 200
```
完整规划单元测试此前已通过。按照你的要求，没有在当前容器中执行 Docker 构建。外部环境构建：
```
docker build -t gcs-multi-inspection:latest .
docker run --rm -p 8080:8080 gcs-multi-inspection:latest
```

问题:
1. 我在宿主机(juice@10.0.40.55 密码: 123456)上已经同步了你的 docker 服务相关提交 @/home/juice/py/GCS_Multi_Inspecton_MIBNB，你现在可以真实构建镜像、容器，并测试服务了:
- 请在宿主机上构建容器
- 完成相关服务真实测试
- 需要注意 dockerfile 中 pip3 install 的下载路径建议改为 https://mirrors.aliyun.com/pypi/simple/


已在宿主机完成真实构建和服务测试。

构建命令：
```
docker build -t gcs-multi-inspection:test .
```
结果：
```
镜像构建成功
镜像大小：1.27GB
镜像 ID：2db24edd25fe
```
Dockerfile 已使用阿里云 PyPI 源：
```
pip install -i https://mirrors.aliyun.com/pypi/simple/ -r requirements.txt
```
容器已启动：
```
容器名：gcs-multi-inspection-test
端口：18080 -> 8080
状态：Up
```
健康检查已通过：
```
GET http://10.0.40.55:18080/healthz
{"status":"ok"}

GET http://10.0.40.55:18080/readyz
{"status":"ready"}
```
真实规划任务测试也已通过：
```
requestId: docker-real-001
状态：succeeded
总运行时间：38.64 秒
规划时间：37.99 秒
```
结果 ZIP 下载成功，包含：
```
input_parameters.json
planning_results.json
run_metadata.json
intensive_waypoints/
gpsov/
figures/
```
结果文件共 19 个，轨迹 CSV、GPSOV 文件和静态 PDF 均正常生成。

当前服务地址：
```
http://10.0.40.55:18080
```
提交任务示例：
```
curl -X POST http://10.0.40.55:18080/v1/plan \
-F package=@input.zip \
-F 'request={"algorithmCode":"gcs_multi_inspection","callType":"async","requestId":"test-001"}'
```
补充说明：虽然项目代码未调用 MosekSolver，但 drake==1.55.0 的 Python 依赖会间接安装 Mosek Python 包。当前镜像中存在该传递依赖，但规划实际使用的仍是 Clarabel/CLP。

提交测试：
```
curl -X POST http://10.0.40.55:18080/v1/plan \
-F package=@input.zip \
-F 'request={"algorithmCode":"gcs_multi_inspection","callType":"async","requestId":"manual-test-001"}'
```
查询状态：
```
curl http://10.0.40.55:18080/v1/plan/manual-test-001
```
下载结果：
```
curl -o result.zip \
http://10.0.40.55:18080/v1/plan/manual-test-001/result
```

问题:
1. 请参考 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/tmp/算法接口文档.md，给出 GCS_Multi_Inspecton_MIBNB 的接口说明文档，输出到 docs 下

2. 在宿主机上 (juice@10.0.40.55 密码: 123456) 导出 "gcs-multi-inspection" 这个镜像到宿主机 "/data/images_store" 下，使用 tar 格式

3. 我在一个新场景上打了点 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/tmp/二期打点1.ovjsn，需要生成配置，说明:
- 这些点位均为 GCJ-02
- 点位说明
```
2-1 ~ 4: 为一区域
2-5 ~ 8: 为一区域
2-9 ~ 12: 为一区域
2-13 ~ 16: 为一区域

2-3、4、5、6: 为一区域
2-6、7、9、10: 为一区域
2-9、12、13、14: 为一区域
2-2、3、13、15: 为一区域
```
请按照我的点位说明，生成一个类似 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/output/20260805_072651_012390/gpsov/regions.txt 的文件，以供我在 奥维地图软件 查看打点正确性

4. 奥维显示错误，其实它用的 CGCS2000 坐标系，region 不用带 g，另外我发现以下区域内部产生交叉:
```
2-13 ~ 16
2-6、7、9、10
2-9、12、13、14
```
请重新生成

修改后:
```
2-1 ~ 4
2-5 ~ 8
2-9 ~ 12
2-13, 2-14, 2-16, 2-15

2-3、4、5、6
2-6, 2-7, 2-10, 2-9
2-9, 2-12, 2-14, 2-13
2-2、3、13、15
```

5. 修改得很对，需要继续生成相关文件(参考 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/data/scenarios/power_plant_wgs84)，生成在 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/data/scenarios/zju2_1_wgs84，说明:
- 需要 5 个文件
```
Boundary.csv
Checkpoints.csv
Depots.csv
Origin.csv
Regions.csv
```
- Boundary.csv 就是 2-1、8、11、16
- Origin.csv 就是 Boundary 的中心点: "119.98037785、30.28864447"
- Regions.csv 参考 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/tmp/二期打点1_regions.txt 你生成的区域
- Depots.csv  2 个机器人即可，符合要求即可
- checkpoints.csv: 10 个点，在区域里即可

```
INSPECTION_CONFIG=configs/zju2_1.json \
.venv/bin/python Inspection_wrapper.py
```

6. 运行
```
INSPECTION_CONFIG=configs/zju2_1.json \
.venv/bin/python Inspection_wrapper.py
```
出现错误:
```
File "/home/jazzy/.local/share/uv/python/cpython-3.12-linux-x86_64-gnu/lib/python3.12/concurrent/futures/_base.py", line 401, in __get_result
    raise self._exception
RuntimeError: Unknown result. The problem is not optimal, infeasible, nor unbounded.
```

说明:
根因是高层点对求解里有多处把 alpha 硬编码成 1，导致 configs/zju2_1.json 里的：
```
"assignment_alpha": 0.1
```
没有真正传到所有高层点对计算中。新场景尺度较小，alpha=1 时 Clarabel/MIBNB 更容易返回：
```
RuntimeError: Unknown result. The problem is not optimal, infeasible, nor unbounded.
```
我已修复：
- assign_checkpoints_to_depots 的绘图分支、串行分支、worker task 都改为使用传入的 alpha
- high_level_tsp 新增 alpha/beta 参数
- TSP 距离矩阵和 cache key 都使用配置传入的 assignment_alpha
- 主流程调用 high_level_tsp 时传入 assignment_alpha
- 增加回归测试，防止以后又硬编码为 1

### assignment_alpha 是配置文件里的参数名；alpha 是算法内部求解最短路径时使用的目标函数权重。现在它们对应的是同一个东西。

可以简单理解为：
```
assignment_alpha = 高层点对距离计算里的路径长度权重
```
当前高层点对成本大致是：
```
cost = alpha * 路径长度 + beta * 时间项
```
目前代码里高层点对计算基本是：
```
beta = 0
```
所以实际就是：
```
cost = alpha * 路径长度
```
#### 直观例子

假设某个 depot 到某个 checkpoint 的最短路径长度是：
```
100 m
```
如果：
```
"assignment_alpha": 1.0
```
那么这个点对成本是：
```
cost = 1.0 * 100 = 100
```
如果：
```
"assignment_alpha": 0.1
```
那么点对成本是：
```
cost = 0.1 * 100 = 10
```
如果：
```
"assignment_alpha": 0.01
```
那么点对成本是：
```
cost = 0.01 * 100 = 1
```
注意：在 beta = 0 且所有点对都用同一个 alpha 的情况下，alpha 只是把所有距离成本整体缩放了一下。

也就是说：
```
100 m vs 200 m
```
在 alpha = 1 时：
```
100 vs 200
```
在 alpha = 0.1 时：
```
10 vs 20
```
大小关系没有变，所以通常不会改变“哪个路径更短”。

#### 它影响什么

主要影响三件事。

第一，影响数值尺度。

你的新场景范围很小，点位之间距离也比较短。如果 alpha = 1，某些优化问题的目标函数尺度、约束尺度和求解器内部容差之间可能不太匹配，Clarabel/MIBNB 就可能返回：
```
Unknown result
```
把 assignment_alpha 设为 0.1 后，本质上是把目标函数数值缩小 10 倍，求解器更稳定。

第二，影响 depot 分配时的成本矩阵。

比如两个机器人：
```
Depot0 -> CheckpointA: 20 m
Depot1 -> CheckpointA: 80 m

alpha = 1：

Depot0 cost = 20
Depot1 cost = 80

alpha = 0.1：

Depot0 cost = 2
Depot1 cost = 8
```
分配结果仍然倾向 Depot0，因为相对大小没变。

第三，未来如果加入 beta，它会影响“距离 vs 时间”的取舍。

例如：
```
cost = alpha * 路径长度 + beta * 到达时间
```
假设有两条路径：
```
路径 A：距离 100 m，时间 140 s
路径 B：距离 120 m，时间 100 s
```
如果只看距离：
```
A 更好
```
如果时间权重很大：
```
B 可能更好
```
这时 alpha 就不再只是缩放，它会和 beta 一起决定路径偏好。

#### 当前项目里怎么用

当前 assignment_alpha 参与两个高层步骤：

1. depot -> checkpoint 成本计算
2. TSP 中 depot/checkpoint 之间的距离矩阵计算

这些成本会影响：
```
哪个 checkpoint 分给哪个 depot
每个 depot 内部的巡检顺序
```
低层最终轨迹规划仍会根据实际路径求解生成航迹点。

#### 对你当前场景的建议

对现在这个 zju2_1 小尺度场景，建议保持：
```
"assignment_alpha": 0.1
```
它不会本质改变“谁离谁更近”的相对关系，但能改善求解器数值稳定性。

如果以后场景变大，比如厂区尺度从几十米变成几百米甚至公里级，可以再测试：
```
"assignment_alpha": 1.0
```
但目前不建议随意改大，因为它已经实测会让这个新场景触发 Unknown result。
