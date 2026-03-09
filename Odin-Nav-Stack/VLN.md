# VLM Client — `VLN.py`

## 概述
`VLN.py`（类 `VLMClient`）实现了一个基于视觉大模型（VLM）的 ROS 节点：
- 从相机话题读取图像，将预处理后的图像以 Data-URL（base64）形式发送到 VLM（使用 OpenAI 兼容 client，调用 `qwen3-vl-plus`）；
- 根据模型返回的 JSON 指令（例如 `{"command": "left"}`）发布短命令到 `/cmd_str`；
- 将带注释的图像发布到 `/<image_topic>/annotated_image` 及 `/.../annotated_image/raw` 以便可视化；
- 统计并记录 token 用量到本地文件（`~/vlm_token_usage.json`）。

节点默认名称：`vlm_client`。

## 工作流程（简要）
1. 订阅图像（`TOPIC_IMAGE`，默认 `/odin1/image/compressed`），缓存最新帧；
2. 驱动后台 worker，按最小间隔 `REQUEST_THROTTLE_SEC`（默认 1s）触发一次请求：
   - 将图像缩放到 `IMG_SIZE`（默认 320×240），调用 `image_pre_process()` 在图像上绘制两条竖线（用于场景分区）；
   - 根据 `COMPRESS_TYPE`（WEBP/JPEG/PNG）压缩并用 base64 编码为 Data-URL；
   - 构造 VLM prompt（脚本内有示例 prompt），并把 image Data-URL 和文本 prompt 一并发送给 VLM（通过 `client.chat.completions.create`）；
   - 解析模型返回的 JSON（期望包含 `command` 字段），将 `command` 字符串封装为 `std_msgs/String` 发布到 `/cmd_str`（当 `command != "no command"`）；
   - 在图像上绘制识别结果文本并发布到注释图像话题；
   - 更新并记录 token 使用统计信息（内存计数 + 写入临时/持久文件）。

## 订阅 / 发布
- 订阅：
  - `TOPIC_IMAGE`（默认 `/odin1/image/compressed`，类型 `sensor_msgs/CompressedImage` 或 `sensor_msgs/Image`，由常量 `TOPIC_IMAGE` 决定）
- 发布：
  - `TOPIC_CMD_STR`（默认 `/cmd_str`，类型 `std_msgs/String`）——模型返回的命令文本
  - `TOPIC_GOAL`（默认 `/neupan/goal`，类型 `geometry_msgs/PoseStamped`）——节点预留但当前代码中未明确使用
  - 注释图像：`<TOPIC_IMAGE>/annotated_image`（`CompressedImage` 或 `Image`）和 `.../raw`

## 主要参数 / 常量
- `TOPIC_IMAGE`：图像话题（默认 `/odin1/image/compressed`）
- `TOPIC_CMD_STR`：命令输出话题（默认 `/cmd_str`）
- `TOPIC_GOAL`：预留导航目标话题（默认 `/neupan/goal`）
- `IMG_SIZE`：送模大小，默认 `(320, 240)`
- `COMPRESS_TYPE`：图片压缩类型（枚举：NONE/JPEG/WEBP），默认 `WEBP`
- `REQUEST_THROTTLE_SEC`：请求最小间隔（秒），默认 `1.0`
- 压缩参数：`JPEG_QUALITY`, `WEBP_QUALITY`, `WEBP_LOSSLESS`

## 模型 / API 与凭据
- 使用 `openai.OpenAI` 客户端（脚本将 `DASHSCOPE_API_KEY` 环境变量作为 `api_key`），并将 `base_url` 设置为 `https://dashscope.aliyuncs.com/compatible-mode/v1`（兼容 Qwen3-VL 系列）。
- 在部署前须设置环境变量：

```bash
export DASHSCOPE_API_KEY="<your_api_key>"
```

并保证节点能访问外部网络或内部兼容代理。

## 注释图像与发布逻辑
- `image_pre_process()` 会在缩放后的图像上绘制两条蓝色竖线（默认在 35% 和 65% 宽度位置，线宽为图像宽度的 1%），用于引导模型判断左右/中三区域。
- `_publish_annotated_image()` 将在图像左下角绘制返回的 `command` 文本，并按 `COMPRESS_TYPE` 选择压缩格式后发布（或通过 `cv_bridge` 转为 `sensor_msgs/Image`）。

## Token 统计与持久化
- 每次 VLM 响应后从 `response['usage']` 中读取 token 统计（total/prompt/completion，及 prompt 的 text/image tokens），累计保存在实例变量并周期性写入 `~/vlm_token_usage.json`（写文件使用原子替换策略）。

## 错误处理与鲁棒性
- CvBridge 转换失败会记录错误并跳过该帧；
- 如果模型返回的内容无法解析为 JSON，会记录错误并把 `result` 设为空字典；
- 请求有最小时间间隔保护以避免频繁调用带来费用或限流问题；
- 退出时会尝试写入 token 使用记录（`atexit.register(self._on_exit)`)。

## 依赖
- Python 库：`openai`（兼容的 SDK）、`opencv-python`、`cv_bridge`、`rospy`、`loguru`、`numpy` 等；
- 网络访问与有效 `DASHSCOPE_API_KEY`。

## 使用示例
- 启动节点：

```bash
# 先设置 API KEY
export DASHSCOPE_API_KEY="..."
# 运行（在已 source ROS 环境下）
rosrun <your_package> VLN.py
```

- 观察输出：订阅 `/cmd_str` 获取文本命令，订阅 `/<image_topic>/annotated_image` 查看带注释的图片。

## 注意事项 / 建议
- 请确保 token 用量与调用频率（`REQUEST_THROTTLE_SEC`）与预算相匹配；
- 对于隐私或敏感数据，请注意将图片发送到外部服务的合规性；
- 当部署到离线或高延迟网络环境时，可以考虑本地化推理或降低请求频率；
- `TOPIC_GOAL` 在当前文件中定义但未实际使用，若需要直接发布导航目标，可扩展逻辑将 `command` → `PoseStamped`。

---

文档位置：`ros_ws/doc/VLN.md`（已生成）

