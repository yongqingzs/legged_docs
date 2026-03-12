# InternVLA-N1 评估流程详解

> 本文档面向 InternNav 的开发者，详细说明 `python scripts/eval/eval.py --config scripts/eval/configs/h1_internvla_n1_async_cfg.py` 的完整评估链路、VLN 语义指令的来源，以及如何修改语义指令。

---

## 目录

1. [整体架构概览](#1-整体架构概览)
2. [评估入口与配置加载](#2-评估入口与配置加载)
3. [完整评估流程](#3-完整评估流程)
4. [VLN 语义指令的来源](#4-vln-语义指令的来源)
5. [如何修改语义指令](#5-如何修改语义指令)
6. [关键文件索引](#6-关键文件索引)

---

## 1. 整体架构概览

整套评估系统由三个并行运作的层次组成：

```
┌─────────────────────────────────────────────────────┐
│                    eval.py (主进程)                  │
│  VLNDistributedEvaluator                            │
│    ├── 环境层: InternUtopia (Isaac Sim)             │
│    │     └── VLNEvalTask (观测生成 + 指令注入)       │
│    └── 智能体层: AgentServer (独立进程, GPU1)        │
│          └── InternVLAN1Agent                       │
│                ├── System 2 (VLM 推理线程)          │
│                │     └── InternVLAN1Net (LLM)       │
│                └── System 1 (DP 运动控制)            │
└─────────────────────────────────────────────────────┘
```

**双系统（Dual-System）设计**：
- **System 2（慢系统）**：基于 VLM（InternVLA-N1）理解自然语言导航指令，预测下一步的 pixel goal（图像中的像素目标点）；
- **System 1（快系统）**：基于 Diffusion Policy 将 pixel goal 转换为实际的速度控制信号，驱动 H1 机器人行走。

---

## 2. 评估入口与配置加载

### 2.1 入口文件

```
scripts/eval/eval.py
```

`main()` 执行以下步骤：

1. **加载配置**：动态导入 `h1_internvla_n1_async_cfg.py` 中的 `eval_cfg` 对象；
2. **补充默认配置**：调用 `internnav/configs/evaluator/vln_default_config.py` 中的 `get_config()`，将用户配置与默认配置做深度合并（机器人控制器、传感器、指标等）；
3. **初始化 Evaluator**：`Evaluator.init(evaluator_cfg)` 根据 `eval_type='vln_distributed'` 注册机制，实例化 `VLNDistributedEvaluator`；
4. **开始评估**：调用 `evaluator.eval()`。

### 2.2 关键配置项（`h1_internvla_n1_async_cfg.py`）

| 配置区块 | 重要参数 | 说明 |
|---------|---------|------|
| `agent` | `server_port=8023`, `model_name='internvla_n1'` | Agent Server 端口与模型标识 |
| `agent.model_settings` | `infer_mode='partial_async'` | 推理模式：partial_async 为 S2 推理一次、S1 执行多步 |
| `agent.model_settings` | `device='cuda:1'` | 模型使用 GPU1，Isaac Sim 使用 GPU0 |
| `env` | `use_fabric=False` | 关闭 Fabric 以避免渲染延迟 |
| `task` | `robot_flash=True`, `max_step=1000` | Flash 模式直接设置机器人位姿，步数上限 1000 |
| `dataset` | `base_data_dir='data/vln_pe/raw_data/r2r'` | R2R 数据集路径 |
| `dataset` | `split_data_types=['val_unseen']` | 使用 val_unseen 集合 |
| `eval_settings` | `use_agent_server=True` | 模型服务与仿真分离（不用分布式 PyTorch） |

---

## 3. 完整评估流程

### 3.1 初始化阶段

```
VLNDistributedEvaluator.__init__()
  ├── 读取数据集 → 确定 total_path_num
  ├── Env.init() → 启动 Isaac Sim / InternUtopia 环境
  │     └── 加载数据集 JSON.gz，构建 episodes 列表
  └── AgentClient 连接 AgentServer（8023 端口）
        └── AgentServer 加载 InternVLAN1Agent + InternVLAN1Net (VLM)
```

### 3.2 评估主循环

```
eval()
  ├── env.reset() → 加载第一个 episode（设置机器人初始位姿、场景）
  ├── warm_up()   → 机器人执行 stand_still 直到 finish_action=True（物理稳定）
  └── while env.is_running():
        ├── get_action(obs)
        │     ├── agent.step(obs) → 发送观测到 AgentServer，获取动作
        │     └── _transform_action_batch() → 转换为环境能理解的格式
        ├── env_step(action) → 执行动作，等待 finish_action
        └── terminate_ops()
              ├── 记录指标（success / fail_reason）
              ├── 保存可视化（video/）
              └── env.reset(reset_env_ids) → 切换下一个 episode
```

### 3.3 Agent 内部推理流程（InternVLAN1Agent.step）

每一步调用 `agent.step(obs)` 时：

```
step(obs)
  ├── 提取: rgb, depth, instruction（来自 obs 字典）
  ├── [System 2 推理线程（如满足触发条件）]
  │     └── policy.s2_step(rgb, depth, pose, instruction, intrinsic)
  │           ├── 构建 VLM Prompt（嵌入 instruction 文本）
  │           ├── 输入历史帧图像 + 当前帧图像
  │           ├── InternVLAN1ForCausalLM.generate() → 输出文本
  │           └── 解析输出：pixel goal (x,y) 或离散动作 (STOP/↑/←/→/↓)
  └── [System 1 推理（利用 S2 的 pixel goal）]
        └── policy.s1_step_latent() → Diffusion Policy → 速度控制序列
```

**partial_async 模式**：S2 推理一次，S1 执行最多 `sys2_max_forward_step`（默认 8）次后，再触发一次 S2 推理。

---

## 4. VLN 语义指令的来源

### 4.1 数据集层面

语义指令（自然语言导航指令）存储在 R2R 数据集文件中：

```
data/vln_pe/raw_data/r2r/val_unseen/val_unseen.json.gz
```

每个 episode 的 JSON 结构如下：

```json
{
  "episode_id": "...",
  "trajectory_id": "...",
  "scene_id": "mp3d/zsNo4HB9uLZ/zsNo4HB9uLZ.glb",
  "start_position": [...],
  "start_rotation": [...],
  "goals": [...],
  "reference_path": [[x, y, z], ...],
  "instruction": {
    "instruction_text": "Exit the bedroom and turn left. Walk straight passing the gray couch and stop near the rug.",
    "instruction_tokens": [...]
  }
}
```

`instruction_text` 就是 VLN 任务的语义指令，描述了机器人从起点走到终点所需遵循的自然语言路径描述。

### 4.2 数据加载层面

```
internnav/env/utils/episode_loader/dataset_utils.py → load_data()
```

`load_data()` 读取 `.json.gz` 文件，将每个 episode 的所有字段（包括 `instruction`）原样保留在 `new_item` 字典中，按 scan（场景）组织成 `dict[scan → list[episode]]`。

### 4.3 任务层面（观测注入）

```
internnav/env/utils/internutopia_extension/tasks/vln_eval_task.py
```

在 `VLNEvalTask` 中，`self.data` 就是单个 episode 的字典（来自数据集），每一步执行后，任务将 instruction 注入到观测字典中：

```python
obs['instruction'] = self.data['instruction']['instruction_text']
obs['instruction_tokens'] = self.data['instruction']['instruction_tokens']
```

这意味着**每一步**环境都会将该 episode 的导航指令随观测一起返回给 Agent。

### 4.4 Agent 层面（VLM Prompt）

```
internnav/model/basemodel/internvla_n1/internvla_n1_policy.py → InternVLAN1Net
```

VLM 的 Prompt 模板为（初始化于 `init_prompts()`）：

```
You are an autonomous navigation assistant. Your task is to <instruction>.
Where should you go next to stay on track?
Please output the next waypoint's coordinates in the image.
Please output STOP when you have successfully completed the task.
```

在每次 S2 推理时，`<instruction>.` 被替换为实际的 `instruction_text`：

```python
sources[0]["value"] = sources[0]["value"].replace('<instruction>.', instruction)
```

然后历史图像帧和当前帧作为视觉输入，一并送入 InternVLAN1ForCausalLM 进行推理。

### 4.5 指令流向完整链路

```
val_unseen.json.gz
    │ instruction.instruction_text
    ▼
load_data() (dataset_utils.py)
    │ episode dict（含 instruction 字段）
    ▼
VLNEvalTask.data (vln_eval_task.py)
    │ obs['instruction'] = self.data['instruction']['instruction_text']
    ▼
obs 字典（每步传给 Agent）
    │ instruction = obs['instruction']
    ▼
InternVLAN1Agent.step() (internvla_n1_agent.py)
    │ s2_input.instruction = instruction
    ▼
InternVLAN1Net.s2_step() (internvla_n1_policy.py)
    │ prompt.replace('<instruction>.', instruction)
    ▼
InternVLAN1ForCausalLM.generate() → 输出 pixel goal 或离散动作
```

---

## 5. 如何修改语义指令

### 5.1 方案一：修改数据集文件（推荐，改整个评估集）

直接修改 `val_unseen.json.gz` 中每个 episode 的 `instruction_text` 字段：

```python
import gzip, json, copy

input_path = 'data/vln_pe/raw_data/r2r/val_unseen/val_unseen.json.gz'
output_path = 'data/vln_pe/raw_data/r2r/val_unseen/val_unseen_custom.json.gz'

with gzip.open(input_path, 'rt', encoding='utf-8') as f:
    data = json.load(f)

for ep in data['episodes']:
    original = ep['instruction']['instruction_text']
    # 示例：在指令前面加上简化提示
    ep['instruction']['instruction_text'] = f"Navigate step by step: {original}"

with gzip.open(output_path, 'wt', encoding='utf-8') as f:
    json.dump(data, f)
```

然后在配置文件中指向新数据集：

```python
# h1_internvla_n1_async_cfg.py
dataset=EvalDatasetCfg(
    dataset_settings={
        'base_data_dir': 'data/vln_pe/raw_data/r2r',
        'split_data_types': ['val_unseen_custom'],  # 修改为新的 split 名
        ...
    },
)
```

### 5.2 方案二：修改 VLM Prompt 模板（改系统提示词）

修改 `internnav/model/basemodel/internvla_n1/internvla_n1_policy.py` 中的 `init_prompts()` 方法：

```python
def init_prompts(self):
    # 原始 prompt
    prompt = (
        "You are an autonomous navigation assistant. "
        "Your task is to <instruction>. "
        "Where should you go next to stay on track? "
        "Please output the next waypoint's coordinates in the image. "
        "Please output STOP when you have successfully completed the task."
    )
    # 修改示例：增加更多上下文约束
    prompt = (
        "You are a navigation robot in an indoor environment. "
        "Follow this instruction carefully: <instruction>. "
        "Identify the next most important waypoint in the image. "
        "Output coordinates as (row, col). "
        "Output STOP when the task is complete."
    )
    answer = ""
    self.conversation = [{"from": "human", "value": prompt}, {"from": "gpt", "value": answer}]
```

**注意**：`<instruction>.` 是占位符，必须保留（包括末尾的句号），否则替换逻辑会失效。

### 5.3 方案三：在 VLNEvalTask 中动态改写指令（运行时注入）

在观测注入处对指令做动态修改，适合需要按场景或步骤自适应调整指令的场景：

```python
# internnav/env/utils/internutopia_extension/tasks/vln_eval_task.py
# 原始代码（约第 212 行）
obs['instruction'] = self.data['instruction']['instruction_text']

# 修改示例：根据步数添加额外提示
step_hint = f" (Step {self.step_count})" if self.step_count > 0 else ""
obs['instruction'] = self.data['instruction']['instruction_text'] + step_hint
```

### 5.4 方案四：构造全新的自定义数据集

如果要完全替换为自定义导航指令，参考 R2R JSON 格式构造新 `.json.gz`：

```python
import gzip, json

custom_episodes = [
    {
        "episode_id": "001",
        "trajectory_id": "traj_001",
        "scene_id": "mp3d/your_scene/your_scene.glb",
        "start_position": [x, y, z],          # 世界坐标（经过 mp3d 坐标变换后）
        "start_rotation": [qx, qy, qz, qw],
        "goals": [{"position": [gx, gy, gz]}],
        "reference_path": [[x1,y1,z1], [x2,y2,z2], ...],
        "instruction": {
            "instruction_text": "Go to the kitchen and find the blue chair.",
            "instruction_tokens": []  # 可以为空列表
        }
    },
    # ...
]

data = {"episodes": custom_episodes}
with gzip.open('data/vln_pe/raw_data/r2r/custom/custom.json.gz', 'wt') as f:
    json.dump(data, f)
```

**坐标系注意**：MP3D 数据集中 `start_position` 存储格式为 `[x, z, y]`（Habitat 惯例），代码中会转换为 `[x, -y, z]`（Isaac Sim 惯例）：

```python
# dataset_utils.py
x, z, y = item['start_position']
new_item['start_position'] = [x, -y, z]
```

因此在自定义数据集中，`start_position` 应使用 Habitat 坐标系的 `[x, z, y]` 格式。

---

## 6. 关键文件索引

| 文件 | 作用 |
|------|------|
| `scripts/eval/eval.py` | 评估入口，加载配置、初始化 Evaluator |
| `scripts/eval/configs/h1_internvla_n1_async_cfg.py` | 用户配置（数据集、模型、环境等） |
| `internnav/configs/evaluator/vln_default_config.py` | 默认配置（机器人、控制器、传感器等），并提供 `get_config()` 合并逻辑 |
| `internnav/evaluator/vln_distributed_evaluator.py` | 主评估循环（`eval()`、`get_action()`、`env_step()`、`terminate_ops()`） |
| `internnav/evaluator/distributed_base.py` | 分布式基类（分布式评估时的 all_gather 逻辑） |
| `internnav/evaluator/base.py` | Evaluator 基类（注册机制、环境 + Agent 初始化） |
| `internnav/env/utils/episode_loader/dataset_utils.py` | 从 `.json.gz` 加载 episodes，做坐标变换和过滤 |
| `internnav/env/utils/internutopia_extension/tasks/vln_eval_task.py` | Isaac Sim 任务实现，负责观测生成（rgb/depth）和 **instruction 注入** |
| `internnav/agent/internvla_n1_agent.py` | InternVLA-N1 Agent，双线程管理 S1/S2 推理 |
| `internnav/model/basemodel/internvla_n1/internvla_n1_policy.py` | VLM 推理核心，**Prompt 模板定义**在 `init_prompts()` |
| `internnav/utils/comm_utils/server.py` | AgentServer（FastAPI），提供 `/agent/init`、`/agent/{name}/step` 等接口 |
| `data/vln_pe/raw_data/r2r/val_unseen/val_unseen.json.gz` | R2R val_unseen 数据集，**语义指令的原始来源** |
