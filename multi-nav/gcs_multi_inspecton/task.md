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

@/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/intensive_waypoints 下是之前算法生成的结果(这个我只想存档); @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/Boundary.csv 等文件都是 input 需要的，但这些的太零乱了。你觉得如何修改合适？先不要修改源码。


像当前 @/home/jazzy/py/GCS_Multi_Inspecton_MIBNB/output/20260804_055407_963096/planning_results.json 的输出结果太冗余了，只要输出关键信息就可以了，并且需要记录总体规划时间