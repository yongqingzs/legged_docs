## task1
当前问题:
1. rai_inspection_agent 中每次调用图像分析时会调用 rag 查询工具
2. 调用 rag 查询工具，获取视觉检测要求-这没有问题
3. 但每次对话调用一次就够了，反复调用浪费资源
你觉得呢，客观分析(不要主观偏向我)。如果要修改，提供方案，先不要修改源码

更合理的做法是：确定性缓存，而不是依赖 LLM 自己少调用。
推荐方案 A：视觉分析工具内部缓存检测要求
- AnalyzeArtifactImageTool 增加缓存字段：
    - inspection_requirements_cache
    - inspection_requirements_cache_query
    - inspection_requirements_loaded_at
    - 可选 inspection_requirements_ttl_sec
- 第一次调用图像分析时查询 RAG。
- 后续同一个 tool 实例、同一个 query 直接复用缓存。
- 如果 RAG 查询失败，不缓存错误，避免一直复用失败结果。
- 可选提供 TTL，例如 10 分钟或 1 小时。

优点：
- 改动最小；
- 不影响 agent 正常问 query_robot_docs；
- 不依赖 LLM 行为；
- 对当前“视觉检测要求”这种固定查询非常合适。

缺点：
- 缓存粒度是 tool 实例级，不严格等于“每轮用户对话一次”。
- 如果 Streamlit/TUI 长时间运行，文档更新后可能需要 TTL 或手动重启才能刷新。


问题:
1. 方案 A 中 TTL 是什么意思？你担心文档更新这点不符合项目构建情况，因为现在 rag 库是开始时使用 faiss 构建，那么中途更新文档是否对 agent 无意义？如果无意义，第一次调用图像分析时进行 rag 检索比较合理(但注意是每次启动后第一次而不是每轮保存对话第一次)？你觉得该如何设计？


问题:
你刚才的修改没问题
1. 当前让 agent 调用云台然后分析场景，我发现返回了两次文本结果，这两次文本结果不一样但内容接近:
- 第一次: "基于提供的机器人视觉检测要求文档，对该巡检图像的详细分析报告如下：..."
- 第二次: 
"...结论
隧道入口区域状态正常
本次巡检显示环境整洁、光照良好：
•  ✅ 未检测到人员违规
•  ✅ 未发现泄漏或烟雾异常
•  ✅ 无设备故障指示灯报警
•  ✅ 所有检测项目均处于正常状态" 
这是为什么？这前后两段分别是什么？

2. 我下载了 langchain 官方提供的 deep-agents-code 实现，我想了解其对我们 rai_inspection_agent 的实现是否有借鉴价值，dcode 路径:
- /home/jazzy/.local/bin/dcode
- 实际的虚拟环境路径: ~/.local/share/uv/tools/deepagents-code/lib/python3.13/site-packages/deepagents_code/
我想了解，其 tui 模块是如何实现的，相比我们当前的实现有无优势，客观分析

不建议直接照搬的部分
deepagents-code 是面向 code agent 的，很多东西对 rai_inspection_agent 不合适：
- 文件 diff、代码修改审批；
- MCP 管理界面；
- provider/model selector；
- skills、cwd、git 状态；
- code sandbox、安全审批流；
- 大量 code-agent 专用 tool renderer。

这些会增加代码量和运行复杂度，对板卡上的机器人巡检 agent 未必划算。
客观结论
有借鉴价值，但不是“替换为 deepagents-code TUI”。
更合理的路线是：保留 RAI 当前轻量 TUI 的方向，吸收它的几个成熟设计。
按照建议优先级完成你的改进：
1. 抽出 tui_adapter.py
    让 agent 事件流和 UI 渲染解耦。
2. 引入专用消息 widget
    至少拆出 UserMessage、AssistantMessage、ToolCallMessage、ToolResultMessage。
3. 改进工具调用实时显示
    工具开始时立即显示，结果返回时更新同一个 widget，而不是任务结束后统一输出。
4. 工具结果做领域化展示
    例如视觉分析工具结果不应该和最终 assistant 回答平铺显示两遍，可以显示为“视觉分析结果摘要/可展开详情”。
5. 借鉴 clipboard fallback
    这个可以比较直接地吸收。
6. 输入框后续再增强
    历史、补全、多行粘贴值得做，但优先级低于实时工具状态。


## task2
从美观、用户体验的角度看 dcode，有哪些我们可以借鉴到 rai 中的地方(因为它也是基于 textual 构建)。先分析，不要修改源码，dcode 路径:
- /home/jazzy/.local/bin/dcode
- 实际的虚拟环境路径: ~/.local/share/uv/tools/deepagents-code/lib/python3.13/site-packages/deepagents_code/

按照 RAI 借鉴的优先级完成对 rai 的改进，并在 rai_inspection_agent 上测试通过:
1. 先改视觉风格
    - 减少完整卡片感；
    - 改为左边框 + 轻背景；
    - 统一主题变量；
    - 背景用深灰蓝，不用纯黑。
2. 做工具结果折叠
    - 工具结果默认摘要；
    - 长结果可展开；
    - 视觉分析、RAG、导航、ROS 工具分别有领域化摘要。
3. 增强输入框
    - / 命令补全；
    - 命令参数 hint；
    - 大段粘贴折叠；
    - 输入框内 prompt。
4. 优化状态栏
    - 用更短、更自然的表达；
    - 运行时显示当前任务状态；
    - idle 时显示 session/user 简要信息。
5. 加时间戳开关
    - 默认隐藏；
    - /timestamps 显示；
    - 对巡检回放和排障有价值。
6. 懒渲染长消息
    - 对板卡性能友好；
    - 对长任务体验也更好。

主要改动：
- 深灰蓝统一主题，消息改为左边框和轻背景，移除完整卡片样式。
- 工具结果默认显示摘要，可按 Enter 或点击展开全文。
- 支持视觉分析、RAG、导航、ROS 四类领域摘要和独立颜色。
- 长工具结果和长对话采用懒渲染，展开时才加载完整内容。
- 输入框增加 › prompt、斜杠命令补全、参数提示。
- 超过 800 字符或 3 行的大段粘贴显示为占位符，提交时自动还原全文。
- 状态栏运行时显示当前状态，空闲时简要显示用户和 session。
- 新增 /timestamps，默认隐藏时间戳，可随时切换。


## task3
我想直接看到 rai 压缩上下文是否有效，于是借助 langsmith 查看 agent 的上下文:
```
[tracing.langsmith]
use_langsmith = true
host = "https://api.smith.langchain.com"
```
但我发现 langsmith 中只能看到系统消息、每次对话的输入输出(但只是用户每次的输入、agent 的回复)，看不到 agent 实际的上下文是什么？
是 langsmith 的问题还是我没有设置对？
先分析，不要修改源码。

RAI 的实际压缩流程是：
1. 对当前 checkpoint 中的消息估算 token。
2. 同时满足以下条件才压缩：
    - 估算 token 大于 8192
    - 消息数量大于 12
3. 保留最近 12 条消息。
4. 更早的消息由简单模型生成摘要，保存到独立的 summary 状态字段。
5. 调用主模型前，把摘要追加到系统消息：
```
The following is an internal summary ...
<压缩后的摘要>
```
对应实现位于：
- src/rai_core/rai/agents/langchain/core/react_agent.py:246
- src/rai_core/rai/memory/graph.py:76
- src/rai_core/rai/memory/graph.py:138

因此，LangSmith 中不会单独显示一项叫“压缩上下文”。真正的模型上下文就是：
```
SystemMessage
基础系统提示
长期记忆
Short-Term Memory Summary
最近 12 条 Human/AI/Tool 消息
```

@/home/jazzy/rai_inspection_agent/tmp/2 这是我导出的一个对话，但我看到 summary 为空，所以我不清楚压缩上下文这个策略是否真实有效。我的建议:
1. 你在 tests/ 下创建一个脚本，实际测试压缩上下文策略是否有效，并且能看到真实的上下文;
2. 实际测试过。
这只是我的建议，你如何看待，有无其他或更好的方案。请客观分析，不要有主观偏向


增加你建议的两层测试，并且实际测试通过。完成后告诉我该如何使用测试文件，以看到上下文结果。
第一层是确定性回归测试，继续使用 Recording Fake LLM，覆盖默认阈值边界：
- token 超阈值，但消息数不超过 12：不压缩；
- 消息数超过 12，但 token 不超阈值：不压缩；
- 两项同时超过：必须压缩；
- 旧消息从 checkpoint 删除；
- 最近 12 条消息完整保留；
- 第二次压缩会合并旧摘要；
- session 恢复后摘要仍然存在；
- 主模型最终上下文中包含摘要，且不包含已删除的旧原文。

第二层是实际模型诊断测试，标记为：
```
@pytest.mark.billable
@pytest.mark.manual
```
该测试应：
1. 使用当前配置的真实 simple_model 和 complex_model。
2. 使用无 ROS 工具的 memory graph，避免误执行机器人动作。
3. 通过较低测试阈值快速触发压缩。
4. 在消息中加入唯一标记，例如：
```
OLD_CONTEXT_SENTINEL
RECENT_CONTEXT_SENTINEL
```
5. 输出并断言：
```
压缩前消息数和估算 token
生成的 summary
压缩后 checkpoint 消息
主模型实际接收到的完整消息列表
SystemMessage 是否包含 summary
旧标记是否已从原始消息中删除
最近标记是否仍然保留
```
这种方式比单纯查看 LangSmith更可靠，因为测试直接截获传给模型的 messages，不会受到 LangSmith页面层级和展示方式影响。


## task4
当前图像分析工具 @/home/jazzy/rai_inspection_agent/rai_inspection_agent/tools/artifact_analysis.py 生成的图像分析结果太长了，不需要像这种长篇报告的形式，我觉得只要:
1. 根据 rag 的要求项，每一项说明是否有问题，然后有个简短结论即可
我该如何修改？先讨论方案，并评估合理性


问题:
1. 我在当前容器 vscode 终端启动 rai.tui 样式是正常的，但在 terminator 远程 cat@10.0.40.230(密码: cat) 上启动:
```
cat@lubancat:~/rai_inspection_agent$ uv run python -m rai_inspection_agent.tui
```
发现 tui 的样式完全不一样(纯黑背景，深蓝输入框);
而在 terminator 启动容器中的 rai.tui，样式和 vscode 终端启动 tui 也不一样(黑灰背景，灰色输入框);
请你进行分析原因，如何解决比较好？先分析，不要修改源码。


分别在 Terminator 容器终端和远程板卡执行：
```
unset NO_COLOR
export TERM=xterm-256color
export COLORTERM=truecolor

uv run python -c \
'from rich.console import Console; print(Console().color_system)'
```
需要输出：
```
truecolor
```
然后启动：
```
uv run python -m rai_inspection_agent.tui
```
如果此时样式与 VSCode 基本一致，就能完全确认原因。

现代 Terminator 支持 truecolor，因此在这几个环境中手动设置 COLORTERM=truecolor 是合理的。不要改成 TERM=xterm-direct，远程板卡可能缺少对应 terminfo。

持久化方式

远程板卡可在 ~/.bashrc 中设置：
```
export TERM=xterm-256color
export COLORTERM=truecolor
unset NO_COLOR
```
容器通过 docker exec 进入时传递：
```
docker exec -it \
-e TERM=xterm-256color \
-e COLORTERM=truecolor \
<container> bash
```
也可以给启动命令加环境变量，影响范围更小：
```
TERM=xterm-256color COLORTERM=truecolor \
uv run python -m rai_inspection_agent.tui
```


问题：
1. 请放宽对 rai_inspection_agent 视觉分析工具调用次数相关的限制
2. 让 rai_inspection_agent 执行任务"前往 point1-8，并进行视觉分析"，中途显示"已达到本轮工具调用上限。Point7 和 Point8 的视觉分析将在下一轮继续执行"。我感觉和 20 个工具上限有关，这个上限是针对轮次设计的吗？我感觉有些苛刻了？分析原因，先不要修改源码。


## task5
1. 当前上下文压缩流程:
```
同时满足以下条件才压缩：
- 估算 token 大于 8192
- 消息数量大于 12
```
会不会有一种情况:
```
在开始对话里
- 估算 token 已经远大于 8192
- 消息数量小于 12
```
另外:
```
第一次已经满足以下条件压缩：
- 估算 token 大于 8192
- 消息数量大于 12
压缩成 summary，然后还加载最近 12 条消息，那么实际的上下文是不是 12 条消息 + summary；如果有一次对话很长，比如: "导航 point1~8，分别视觉分析"，那么实际上下文会不会比设想的多很多？
```
你怎么看待当前的上下文压缩策略？如果让你设计，你会如何设计或改进？给出方案。


建议落地顺序
1. 引入完整 context budget，修复“少量超长消息不压缩”。
2. 将固定 12 条改成按 token 动态保留，并保护工具调用组。
3. 把 summary、系统提示词、长期记忆、工具 schema 纳入预算。
4. 在内部每次 LLM 调用前增加预算检查。
5. 为长工具任务增加结构化执行进度，压缩已完成步骤。
6. 对视觉、RAG 等大结果使用摘要加 artifact 引用。
7. 加入监控数据：压缩前后 token、保留消息数、summary token、压缩原因。

必须补充的测试
至少应覆盖：
- 2 条消息但超过预算，能够压缩。
- 12 条超长消息压缩后确实低于目标预算。
- summary 加最近消息后的总上下文不超限。
- 不拆开 tool call 和 tool result。
- point1~8 长任务中，每次模型调用都低于预算。
- 单个超长视觉/RAG 结果能够转换为摘要或引用。
- summary 多次更新后仍受长度限制。


好的，我查看了你的方案。这是 langchain 官方 agent 实现 dcode 路径:
- /home/jazzy/.local/bin/dcode
- 实际的虚拟环境路径: ~/.local/share/uv/tools/deepagents-code/lib/python3.13/site-packages/deepagents_code/
dcode 上下文压缩策略是什么？
对我们当前方案有无借鉴价值或者能减少开发负担。


建议采用方式
### 通用能力放入 RAI
基于 LangChain 官方 middleware 建立 RAI 自己的上下文管理层：
```
ContextBudgetMiddleware
├── 官方 token 计数
├── 模型窗口配置
├── 每次模型调用前检查
├── 官方安全消息切分
├── summary 生成
├── ContextOverflowError 重试
└── 压缩指标和 LangSmith 事件
```
推荐初始参数：
```
trigger：模型最大输入的 70%～75%
keep：模型最大输入的 15%～20%
summary：最多 1000～1500 token
output reserve：按模型显式预留
message guard：作为独立辅助条件
```
RAI 的本地模型必须明确配置：
```
max_input_tokens
max_output_tokens
```
不要依赖模型服务自动上报。

### 工具结果压缩提供扩展接口
RAI 通用层定义类似：
```
ToolResultCompactor
ArtifactStore
```
rai_inspection_agent 提供领域实现：
```
视觉分析：检查项状态 + 简短结论 + 完整 artifact
RAG：命中文档标题 + 关键要求 + 完整检索结果引用
导航：目标点 + 状态 + 耗时 + 失败原因
ROS：关键字段 + 完整消息引用
```
这样比直接复制 dcode 的“全部写文件，然后 read_file”更符合机器人场景。

### 保留原始会话与有效上下文分离
建议像 dcode 一样区分：
```
原始 transcript：用于 TUI、导出、回放、排障
effective context：summary + 当前任务相关消息
```
不要让压缩直接删除用户可见的历史记录。

按照你的方案，基于 LangChain 官方 middleware 建立 RAI 自己的上下文管理层，并且注意上下文总体大小限制不能完全依赖模型本身窗口大小(本地模型甚至是端侧 jetson agx 上的模型，上下文窗口表面很大，但后面会速度下降)，实际窗口大小至少需要能从 config.toml 配置。完成并测试通过。


### 实现内容
- 每次 LLM 调用前检查上下文，包括同一用户任务内部的多次工具循环。
- 预算包含：
    - System Prompt
    - summary
    - 对话消息
    - 工具 schema
    - 图片估算成本
- 历史消息按 token 动态保留，并保护 AI tool call 与 ToolMessage 的对应关系。
- 当前用户轮次保留工具调用骨架，避免压缩后 ToolPolicy 计数失效。
- 大型工具结果会确定性裁剪并添加明确标记。
- 无法压缩的超大用户输入在调用模型前报错。
- summary 模型懒加载，普通短请求不会增加启动延迟。
- 超长历史采用有预算的分批滚动总结。  

核心实现：src/rai_core/rai/context/manager.py、src/rai_core/rai/agents/langchain/core/react_agent.py:191、src/rai_core/rai/
memory/graph.py:78。

