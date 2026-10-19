## task1
当前 @/home/jazzy/cpp/capability_mission_planner 完成规划时"如果两机冲突，会使用 cbs 解决冲突问题"；这是 @/home/jazzy/cpp/capability_mission_scenarios 测试场景；我想要你实现:
- @/home/jazzy/cpp/capability_mission_scenarios/output/bdz1/03_homogeneous_shared_charger_benchmark/routes_bdz1.png 是输出的结果图片，我希望能够额外生成总体 gif，从而直观看到 cbs 的效果
- 你的建议如何，实现方案如何？

问题:
1. capability_mission_planner 算法、gif 生成脚本是否适用新增场景 @/home/jazzy/cpp/capability_mission_scenarios/maps/sxbdz1，请验证(点位数量、任务类型不要设置得太少、太简单)

2. "最后一台机器人等待较长，调度效率仍有优化空间"该如何优化，请分析，方案需要具备可行性

3. 这是什么意思，你的方案是如何实现改进的，用直白的例子描述一下
"- 按配置中的机器人顺序逐台调度，前面的时间表固定后，后面的机器人只能避让"


问题:
1. 完成 "补齐安全检查后，先建立预约区间和阻塞记录，再接入安全时间区间搜索，最后增加一次有边界的局部修复"，每一步修改需要在多个场景验证行之有效且没有大的负面代价(否则回退)，只保留真正有效的修改

2. 你刚才的修改针对 @/home/jazzy/cpp/capability_mission_scenarios/maps/sxbdz1 下不同场景(最好比 01_six_robots_80_sites_100_tasks.yaml 更复杂，尽量覆盖全图)、/home/jazzy/cpp/capability_mission_scenarios/configs/bdz1/05_five_robot_types_60_points.yaml(04_five_robot_types_50_points.yaml、 06_five_robot_types_80_points.yaml )，是否有效？请实验。

3. 当前 capability_mission_planner 下 include src 目录源文件都放在一起，你有什么方案，调整/新增 include src 下的目录结构，使其更加清晰、功能分明(不要动原有功能)
- 你的方案不错，但我发现一些单独文件也放在单一目录下，可以目录拆解，但尽量不要过度拆解，增加过多子目录，适中且清晰就好


## 窄路会车问题
capability_mission_planner 从初始多机规划层面进行各机器人的冲突协调，但在实际场景中由于机器人避障、时延、动力学差异等问题在窄路还是会出现冲突。
