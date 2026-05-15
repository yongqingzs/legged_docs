# rai_perception 模块详解

## 目录

- [1. 概述](#1-概述)
- [2. 模块架构](#2-模块架构)
- [3. 核心流水线](#3-核心流水线)
  - [3.1 物体检测](#31-物体检测-groundingdino)
  - [3.2 图像分割](#32-图像分割-grounded-sam)
  - [3.3 点云提取](#33-点云提取)
  - [3.4 点云过滤](#34-点云过滤)
  - [3.5 抓取点估计](#35-抓取点估计)
- [4. 工具系统](#4-工具系统)
- [5. 服务系统](#5-服务系统)
- [6. 模型注册表](#6-模型注册表)
- [7. 预设配置](#7-预设配置)
- [8. 关键 API](#8-关键-api)
- [9. 部署与启动](#9-部署与启动)

---

## 1. 概述

`rai_perception` 是 RAI 框架的**视觉感知扩展模块**，实现了基于**开放集检测模型**的机器人感知能力。

核心功能：

- **物体检测**：使用 GroundingDINO 模型，通过自然语言描述检测任意类别的物体
- **图像分割**：使用 Grounded-SAM（Segment Anything Model 2）生成像素级物体掩码
- **3D 点云重建**：将 2D 掩码 + 深度图反投影为 3D 点云
- **抓取点估计**：从点云中提取物体的抓取位置（支持多种策略）

```
┌───────────────────────────────────────────────────────────────────┐
│                     rai_perception 模块架构                         │
├───────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │   Tools     │  │  Services   │  │  Models     │              │
│  │  Layer      │  │  Layer      │  │  Registry   │              │
│  │             │  │             │  │             │              │
│  │ GetObject   │  │ Detection   │  │ get_model() │              │
│  │   Gripping  │  │  Service    │  │  → GDBoxer  │              │
│  │   Points    │  │             │  │  → GDSegm.  │              │
│  │  Tool       │  │ Segment.    │  └─────────────┘              │
│  │             │  │  Service    │                               │
│  │ GetDetect.  │  │             │  ┌─────────────┐              │
│  │  Tool       │  │ BaseVision  │  │ Algorithms  │              │
│  │             │  │  Service    │  │             │              │
│  │ GetDistanc. │  └─────────────┘  │ GDBoxer     │              │
│  │  Tool       │                   │ GDSegmenter │              │
│  └─────────────┘                   │ depth_to_   │              │
│                                    │  point_cloud│              │
│  ┌─────────────┐                   └─────────────┘              │
│  │ Components  │                                               │
│  │             │  ┌─────────────┐                               │
│  │ PointCloud  │  │ Presets     │                               │
│  │  FromSeg.   │  │             │                               │
│  │  Filter     │  │ default_    │                               │
│  │  Estimator  │  │  grasp      │  ┌─────────────┐              │
│  └─────────────┘  │ precise_    │  │ Utils       │              │
│                    │  grasp      │  │ service_    │              │
│                    │ top_grasp   │  │  utils      │              │
│                    └─────────────┘  │ perception_ │              │
│                                     │  presets    │              │
│                                     └─────────────┘              │
└───────────────────────────────────────────────────────────────────┘
```

---

## 2. 模块架构

```
rai_perception/
│
├── agents/              # 已废弃 → 使用 services/
│   ├── base_vision_agent.py      # 已废弃 → BaseVisionService
│   ├── grounded_sam.py           # 已废弃 → SegmentationService
│   └── grounding_dino.py         # 已废弃 → DetectionService
│
├── algorithms/          # 底层视觉算法（模型无关）
│   ├── boxer.py              # GDBoxer - GroundingDINO 检测算法
│   ├── segmenter.py          # GDSegmenter - Grounded-SAM 分割算法
│   └── point_cloud.py        # depth_to_point_cloud - 深度图转点云
│
├── components/          # 核心组件
│   ├── gripping_points.py       # 点云提取、过滤、抓取点估计
│   ├── perception_presets.py    # 预设配置 (default/precise/top_grasp)
│   ├── perception_utils.py      # 感知工具函数
│   ├── service_utils.py         # 服务实用工具
│   ├── topic_utils.py           # 话题发现工具
│   ├── math_utils.py            # 数学工具（四元数转旋转矩阵等）
│   └── visualization_utils.py   # 可视化工具
│
├── configs/             # 模型配置
│   └── gdino_config.py   # GroundingDINO 配置文件
│
├── models/              # 模型注册表
│   ├── detection.py          # 检测模型注册表
│   └── segmentation.py       # 分割模型注册表
│
├── scripts/             # 启动脚本
│   ├── run_perception_services.py   # 启动感知服务
│   └── download_weights.py          # 下载模型权重
│
├── services/            # ROS 2 服务节点
│   ├── base_vision_service.py   # 服务基类（模型加载、权重管理）
│   ├── detection_service.py     # 检测服务
│   ├── segmentation_service.py  # 分割服务
│   └── weights.py               # 权重下载与加载
│
└── tools/               # LangChain 工具
    ├── gripping_points_tools.py   # 抓取点工具
    ├── gdino_tools.py             # 检测工具、距离工具
    └── segmentation_tools.py      # 分割工具
```

---

## 3. 核心流水线

### 3.1 物体检测 (GroundingDINO)

```
输入: RGB 图像 + 物体类别名称列表
  ↓
GroundingDINO 模型推理
  ↓
输出: 检测框 [x, y, w, h] + 类别 + 置信度
```

**模型**：GroundingDINO (IDEA-Research)

**特点**：开放集检测，支持任意自然语言描述作为类别。不需要预训练类别限制。

**权重路径**：`~/.cache/rai/vision/weights/groundingdino_swint_ogc.pth`

**算法类**：`GDBoxer`

```
GDBoxer
  |-- get_boxes(image, classes, box_threshold, text_threshold)
  |     |
  |     +--→ Box(center, size_x, size_y, phrase, confidence)
  |
  |-- to_detection_msg(class_dict, timestamp)
        |
        +--→ ROS 2 Detection2D 消息
```

**关键参数**：

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `box_threshold` | 0.35 | 检测框置信度阈值 |
| `text_threshold` | 0.45 | 文本匹配阈值 |
| `use_cuda` | True | 是否使用 GPU 加速 |

### 3.2 图像分割 (Grounded-SAM)

```
输入: RGB 图像 + 检测框
  ↓
SAM2 模型 (Segment Anything Model 2)
  ↓
输出: 像素级分割掩码 (二值图像)
```

**模型**：SAM2 Hiera Large (Meta)

**特点**：基于检测框生成精确的像素级物体掩码，支持多实例。

**权重路径**：`~/.cache/rai/vision/weights/sam2_hiera_large.pt`

**算法类**：`GDSegmenter`

```
GDSegmenter
  |-- __init__(weight_path, config_path, use_cuda)
  |     |
  |     +-- 加载 SAM2 模型
  |     +-- 初始化 Hydra 配置系统
  |
  |-- segment(image, boxes)
        |
        +--→ 像素级掩码列表
```

### 3.3 点云提取

```
输入: 分割掩码 + 深度图 + 相机内参
  ↓
逐像素反投影
  ↓
输出: Nx3 数组 [X, Y, Z]
```

**核心函数**：`depth_to_point_cloud()`

```python
def depth_to_point_cloud(
    depth_image: NDArray[np.float32],  # (H, W)
    fx: float, fy: float,              # 焦距
    cx: float, cy: float,              # 光心
) -> NDArray[np.float32]:              # (N, 3)
```

**算法**：标准相机反投影

```
X = (u - cx) * depth / fx
Y = (v - cy) * depth / fy
Z = depth
```

**流程**（`PointCloudFromSegmentation.run()`）：

```
1. 获取 RGB 图像 (摄像头 topic)
2. 获取深度图 (深度摄像头 topic)
3. 获取相机内参 (CameraInfo topic)
4. 调用 Detection Service (GroundingDINO)
5. 调用 Segmentation Service (Grounded-SAM)
6. 用掩码提取深度图中的像素
7. depth_to_point_cloud() 反投影为 3D 点
8. 坐标变换: 相机坐标系 → 目标坐标系
```

### 3.4 点云过滤

```
输入: 原始点云 (含噪声、离群点)
  ↓
Isolation Forest 异常检测
  ↓
输出: 过滤后的干净点云
```

**配置类**：`PointCloudFilterConfig`

| 参数 | 类型 | 说明 |
|------|------|------|
| `strategy` | str | 过滤策略 (`aggressive_outlier_removal`) |
| `outlier_fraction` | float | 异常点比例 (0.01~0.05) |
| `min_points` | int | 最小有效点数 |

**策略**：Isolation Forest 隔离森林算法，自动识别并移除离群点。

### 3.5 抓取点估计

```
输入: 过滤后的点云
  ↓
策略选择 (centroid / top_plane)
  ↓
输出: 抓取点坐标 [x, y, z]
```

**配置类**：`GrippingPointEstimatorConfig`

| 策略 | 说明 | 适用场景 |
|------|------|---------|
| `centroid` | 计算点云质心 | 通用位置查询 |
| `top_plane` | RANSAC 拟合顶部平面 | 顶部抓取 |

| 参数 | 类型 | 说明 |
|------|------|------|
| `strategy` | str | 估计策略 |
| `ransac_iterations` | int | RANSAC 迭代次数 |
| `distance_threshold_m` | float | 点到平面距离阈值 |
| `top_percentile` | float | 顶部 Z 轴高度百分位 |
| `min_points` | int | 最小点数 |

---

## 4. 工具系统

```
LangChain 工具 (LLM 可调用的工具)
│
├── GetObjectGrippingPointsTool      # 抓取点工具（核心工具）
│     ├── 参数: object_name, debug
│     ├── 依赖: DetectionService + SegmentationService
│     ├── 三段流水线: 点云提取 → 过滤 → 抓取点估计
│     └── 返回: 格式化字符串，包含抓取点坐标
│
├── GetObjectPositionsTool           # 物体位置工具（委托工具）
│     ├── 参数: object_name
│     ├── 委托给: GetObjectGrippingPointsTool + default_grasp 预设
│     └── 返回: 物体质心位置
│
├── GetDetectionTool                 # 检测工具
│     ├── 参数: camera_topic, object_names
│     ├── 依赖: DetectionService
│     └── 返回: 检测到的物体列表 + 边界框
│
└── GetDistanceToObjectsTool         # 距离工具
      ├── 参数: camera_topic, depth_topic, object_names
      ├── 依赖: DetectionService
      ├── 两段流水线: 检测 → 距离计算
      └── 返回: 物体名称 + 距离 (米)
```

### 工具服务依赖

```
GetObjectGrippingPointsTool / GetObjectPositionsTool
    ├── 需要: /detection (DetectionService)
    └── 需要: /segmentation (SegmentationService)

GetDetectionTool
    └── 需要: /detection (DetectionService)

GetDistanceToObjectsTool
    └── 需要: /detection (DetectionService)
```

### 工具参数配置

所有工具支持 ROS 2 参数覆盖，参数前缀为 `perception.gripping_points`：

```
perception.gripping_points.camera_topic        → "/camera/rgb/image_raw"
perception.gripping_points.depth_topic         → "/camera/depth/image_raw"
perception.gripping_points.camera_info_topic   → "/camera/rgb/camera_info"
perception.gripping_points.target_frame        → "base_link"
perception.gripping_points.source_frame        → "camera_link"
perception.gripping_points.timeout_sec         → 60.0
perception.gripping_points.conversion_ratio    → 1.0
```

---

## 5. 服务系统

```
ROS 2 服务节点 (独立进程，提供视觉能力)
│
├── BaseVisionService                # 服务基类
│     ├── 权重管理 (自动下载、缓存)
│     ├── 模型加载 (带错误处理)
│     ├── ROS 2 服务注册
│     └── 服务名解析 (新名称 + 旧名称兼容)
│
├── DetectionService                 # 检测服务
│     ├── 新服务名: /detection
│     ├── 旧服务名: /grounding_dino_classify (兼容)
│     ├── 模型: GroundingDINO
│     ├── 权重: groundingdino_swint_ogc.pth
│     └── 回调: _classify_callback(request, response)
│
└── SegmentationService              # 分割服务
      ├── 新服务名: /segmentation
      ├── 旧服务名: /grounded_sam_segment (兼容)
      ├── 模型: SAM2 Hiera Large
      ├── 权重: sam2_hiera_large.pt
      └── 回调: _segment_callback(request, response)
```

### 服务启动流程

```
run_perception_services.py
    │
    ├── 1. rclpy.init()
    │
    ├── 2. 创建两个 ROS 2 Connector (单线程执行器)
    │       ├── detection_connector
    │       └── segmentation_connector
    │
    ├── 3. 并行下载模型权重
    │       ├── Thread 1: 下载 GroundingDINO 权重
    │       └── Thread 2: 下载 SAM2 权重
    │
    ├── 4. 加载模型
    │       ├── DetectionService._initialize_model()
    │       └── SegmentationService._initialize_model()
    │
    └── 5. 注册 ROS 2 服务 + 等待 shutdown 信号
```

### 服务名双注册机制

```
enable_legacy_service_names = true  (默认)
    │
    ├── DetectionService 注册:
    │     ├── /detection (新名称)
    │     └── /grounding_dino_classify (旧名称，兼容)
    │
    └── SegmentationService 注册:
            ├── /segmentation (新名称)
            └── /grounded_sam_segment (旧名称，兼容)

enable_legacy_service_names = false
    │
    ├── DetectionService 注册: /detection
    └── SegmentationService 注册: /segmentation
```

---

## 6. 模型注册表

```
模型注册系统 (支持模型切换，无需改代码)
│
├── 检测模型注册表 (_DETECTION_REGISTRY)
│     │
│     +-- "grounding_dino" → (GDBoxer, config_path)
│     │
│     +-- get_model("grounding_dino")
│     │     → 返回 (GDBoxer 类, "configs/gdino_config.py")
│     │
│     └── list_available_models()
│           → ["grounding_dino"]
│
└── 分割模型注册表 (_SEGMENTATION_REGISTRY)
      │
      +-- "grounded_sam" → (GDSegmenter, None)
      │
      +-- get_model("grounded_sam")
      │     → 返回 (GDSegmenter 类, None)
      │
      └── list_available_models()
            → ["grounded_sam"]
```

**切换模型的方式**（ROS 2 参数）：

```python
node.set_parameter("model_name", "grounding_dino")  # 检测模型
node.set_parameter("model_name", "grounded_sam")    # 分割模型
```

---

## 7. 预设配置

```
预设系统 (快速选择过滤 + 估计策略组合)
│
├── default_grasp              # 默认抓取预设
│     ├── 过滤: aggressive_outlier_removal, 5% 异常点
│     └── 估计: centroid (质心)
│
├── precise_grasp              # 精确抓取预设
│     ├── 过滤: aggressive_outlier_removal, 1% 异常点
│     └── 估计: top_plane, 500 RANSAC 迭代, 5mm 阈值
│
└── top_grasp                  # 顶部抓取预设
      ├── 过滤: aggressive_outlier_removal, 5% 异常点
      └── 估计: top_plane, 顶部 5% Z 轴高度
```

**使用方式**：

```python
from rai_perception.components.perception_presets import apply_preset

filter_config, estimator_config = apply_preset("top_grasp")
tool = GetObjectGrippingPointsTool(
    connector=connector,
    filter_config=filter_config,
    estimator_config=estimator_config,
)
```

---

## 8. 关键 API

```
rai_perception 公开 API
│
├── 工具
│     ├── GetObjectGrippingPointsTool        # 抓取点工具
│     ├── GetObjectGrippingPointsToolInput   # 工具输入 schema
│     ├── GetDetectionTool                   # 检测工具
│     └── GetDistanceToObjectsTool           # 距离工具
│
├── 服务
│     ├── DetectionService                   # 检测服务
│     ├── SegmentationService                # 分割服务
│     └── BaseVisionService                  # 服务基类
│
├── 组件
│     ├── PointCloudFromSegmentation         # 点云提取组件
│     ├── PointCloudFromSegmentationConfig   # 点云提取配置
│     ├── PointCloudFilter                   # 点云过滤组件
│     ├── PointCloudFilterConfig             # 点云过滤配置
│     ├── GrippingPointEstimator             # 抓取点估计组件
│     └── GrippingPointEstimatorConfig       # 抓取点估计配置
│
├── 工具函数
│     ├── depth_to_point_cloud               # 深度图转点云
│     ├── discover_camera_topics             # 发现摄像头话题
│     └── wait_for_perception_dependencies   # 等待感知依赖就绪
│
├── 常量
│     ├── GDINO_SERVICE_NAME                 # "grounding_dino_classify"
│     ├── GDINO_NODE_NAME                    # "grounding_dino_node"
│     ├── GSAM_SERVICE_NAME                  # "grounded_sam_segment"
│     └── GSAM_NODE_NAME                     # "grounded_sam_node"
│
└── 已废弃 (向后兼容)
      ├── GroundingDinoAgent   → 使用 DetectionService
      └── GroundedSamAgent     → 使用 SegmentationService
```

---

## 9. 部署与启动

```
部署流程
│
├── 1. 安装依赖
│     └── uv sync --group perception
│
├── 2. 启动感知服务
│     └── python run_perception_services.py
│           │
│           ├── 自动下载模型权重 (首次运行)
│           ├── 加载 GroundingDINO + SAM2 模型
│           └── 注册 ROS 2 服务 /detection + /segmentation
│
├── 3. 在 Agent 中使用工具
│     └── tools = [GetObjectPositionsTool(connector=connector)]
```

### 完整感知工具执行流程

```
用户 → LLM: "厨房有什么？"
    │
    ▼
Agent 调用 GetObjectPositionsTool(object_name="chair")
    │
    ├── Step 1: GetObjectPositionsTool 委托给 GetObjectGrippingPointsTool
    │           (使用 default_grasp 预设 = centroid 策略)
    │
    ▼
    ════════════════════════════════════════════════
    Stage 1: 点云提取 (PointCloudFromSegmentation)
    ════════════════════════════════════════════════
    │
    ├── 1a. 获取 RGB 图像
    │     └── connector.receive_message("/camera/rgb/image_raw")
    │
    ├── 1b. 获取深度图
    │     └── connector.receive_message("/camera/depth/image_raw")
    │
    ├── 1c. 获取相机内参
    │     └── connector.receive_message("/camera/rgb/camera_info")
    │
    ├── 1d. 调用 /detection 服务
    │     ├── 发送: RAIGroundingDino.Request(image, classes="chair")
    │     └── 接收: 检测框列表 [x, y, w, h]
    │
    ├── 1e. 调用 /segmentation 服务
    │     ├── 发送: RAIGroundedSam.Request(image, detections)
    │     └── 接收: 像素级掩码列表
    │
    ├── 1f. 用掩码提取深度值
    │     └── masked_depth[mask == 255] = depth[mask == 255]
    │
    └── 1g. 反投影为 3D 点云
          └── depth_to_point_cloud(masked_depth, fx, fy, cx, cy)
              → Nx3 数组 [X, Y, Z]
    │
    ▼
    ════════════════════════════════════════════════
    Stage 2: 点云过滤 (PointCloudFilter)
    ════════════════════════════════════════════════
    │
    ├── 隔离森林算法检测离群点
    ├── 移除 5% 异常点 (default_grasp 预设)
    └── 输出: 过滤后的点云
    │
    ▼
    ════════════════════════════════════════════════
    Stage 3: 抓取点估计 (GrippingPointEstimator)
    ════════════════════════════════════════════════
    │
    ├── centroid 策略
    ├── 计算点云中心: mean(points[:, 0:2])
    └── 输出: 质心坐标 [x, y, z]
    │
    ▼
坐标变换: camera_link → base_link
    ├── 获取 TF 变换 (ROS 2 transform)
    ├── R = quaternion_to_rotation_matrix(qw, qx, qy, qz)
    └── points_target = points @ R.T + translation
    │
    ▼
返回给 LLM: "Detected 2 chairs at (0.5, 1.2, 0.3) and (0.8, -0.5, 0.2)"
```

### 权重管理

```
模型权重缓存路径: ~/.cache/rai/vision/weights/
│
├── groundingdino_swint_ogc.pth      (约 340 MB)
│     └── 来源: https://github.com/IDEA-Research/GroundingDINO/releases/
│
└── sam2_hiera_large.pt              (约 2.5 GB)
      └── 来源: https://dl.fbaipublicfiles.com/segment_anything_2/

首次运行自动下载，后续运行直接加载缓存。
可通过 download_weights.py 脚本手动下载。
```

### 向后兼容

```
已废弃组件 → 替代方案
│
├── agents/ 目录        → services/ 目录
├── GroundingDinoAgent  → DetectionService
├── GroundedSamAgent    → SegmentationService
├── BaseVisionAgent     → BaseVisionService
├── vision_markup/      → algorithms/
│
└── 旧服务名兼容:
      ├── /grounding_dino_classify  → /detection
      └── /grounded_sam_segment     → /segmentation
      (通过 enable_legacy_service_names 参数控制)
```
