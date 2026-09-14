# 多机协同：基于 MQTT 的分布式领队选举设计

本文回答两个问题：

1. 多机任务分配（`capability_mission_planner`）在 MQTT 星型拓扑下应该怎么组织？
2. 分布式选举时，"选举结果以谁为准"？原领队掉线又回归导致双领队怎么办？

---

# 1. 结论先行

现有方案（每台机器人部署 planner，上报中心服务器，中心选领队，领队算完回传，中心下发）**大方向合理，但有一个角色错位**：中心服务器同时承担了"通信"和"仲裁"两件事。

一旦要求"分布式选举"，中心服务器就必须退化成**纯 broker**，不再有"选领队"的业务逻辑。

> **核心原则：算法集中 ≠ 部署集中。**

`capability_mission_planner` 是全局优化（A\* 代价 + CBS 时空协调），本质就是集中式算法。硬改成分布式（拍卖 / CBBA）会付出巨大代价且结果更差。正确做法是：

* **求解逻辑集中**：一次算出全局较优解；
* **部署形态分布**：谁来算，由机器人自己选出，不依赖任何固定服务器。

这就是"领队"模式的本质，也是 RMF 一类系统的通行做法。

## 1.1 目标架构

```text
平台 ──> broker ──> 全体机器人（任务请求）

各机心跳/状态  status（retained + LWT）
        ↓
领队本地跑 planner
        ↓
广播 plan（retained）
        ↓
各机取自己那份 ──> TaskHub /start_route
```

broker 只是邮局，**没有中心决策者**。

---

# 2. 一个决定可实现性的关键判断：这里不需要 Raft

这一点想清楚，代码量能降一个数量级。

**MQTT 是星型拓扑，所有通信必经 broker。** 于是：

* 能连上 broker ⇒ 所有机器人看到**同一份消息流**（同一份成员视图）；
* 连不上 broker ⇒ 与整个集群失联，什么都做不了，不存在"半边集群继续干活"。

> **真正的不对称网络分区在星型拓扑下不成立 ⇒ 不会脑裂 ⇒ 不需要 term / votedFor / quorum / 日志复制。**

所以选举只需要四样东西：**心跳 + LWT + 租约 + 静态优先级 tie-break**。这是完全确定性的：给定成员集合，领队唯一，且所有机器人独立算出同一个答案。

需要保留的唯一一致性护栏是 **epoch（单调递增纪元号）**，但它放在**任务层**和**租约层**，而不是做成一套完整的共识协议。

---

# 3. 心智模型：领队不是"身份"，是"租约"

这是理解整个设计的入口，也是回答"以谁为准"的关键。

不要有"我是领队"这个持久状态。真实状态只有一条：

> **某节点在时刻 T 之前持有 epoch=E 的租约。**

于是"以谁为准"有了明确答案：

> **不以任何节点的判断为准，以 broker 上那条 retained 的 `leader` 消息为准。**

注意 broker 是**纯存储**，不含任何选举逻辑——它不是"仲裁者"，只是一块所有人都能读写的共享黑板。每个节点独立读同一块黑板、独立套用同一条规则，因此结论必然一致。

**类比**：这正是 Kubernetes 领导选举的做法——`coordination.k8s.io/Lease` 对象存在 etcd 里，抢锁就是更新 Lease 的 `holderIdentity` + `renewTime`，没有 Raft、没有投票。也与 Raft 中"日志过旧的节点拿不到多数票"异曲同工，只是星型拓扑下不需要多数派。

---

# 4. Topic 与报文设计

```text
fh/fleet/v1/robot/<sn>/status     retain=1  QoS=1  will={"online":false}
fh/fleet/v1/leader                retain=1  QoS=1  （不设 LWT）
fh/fleet/v1/election              retain=0  QoS=1  （可选：claim / nack 公告）
fh/fleet/v1/mission/<mid>/request retain=1  QoS=1  平台或任意机器人发起
fh/fleet/v1/mission/<mid>/intent  retain=1  QoS=1  两阶段：意图
fh/fleet/v1/mission/<mid>/ack     retain=0  QoS=1  两阶段：成员 ack
fh/fleet/v1/mission/<mid>/plan    retain=1  QoS=1  分配结果
```

## 4.1 status（全队唯一的事实来源）

```json
{
  "sn": "x30",
  "online": true,
  "ts": 1785142812000,
  "pose": { "map_id": "map_000", "x": 12.5, "y": 3.1, "yaw": 0.4 },
  "battery": 0.82,
  "mode": "auto",
  "work_state": "idle",
  "capabilities": ["camera_visible", "gas"],
  "route_progress": { "mission_id": "m-12", "epoch": 3, "waypoint": 5 },
  "versions": { "planner": "1.4.0", "maps": "sha1:ab12cd.." }
}
```

## 4.2 leader（租约）

```json
{
  "leader": "x30",
  "epoch": 7,
  "lease_until": 1785142815000,
  "view": ["x30", "x31", "x32"]
}
```

## 4.3 plan（分配结果）

```json
{
  "mid": "m-12",
  "epoch": 8,
  "leader": "x31",
  "planner_version": "1.4.0",
  "map_digest": "sha1:ab12cd..",
  "input_digest": "sha1:9f3e..",
  "assignments": { "x30": {...}, "x31": {...} }
}
```

## 4.4 参数表

| 项 | 值 | 说明 |
|---|---|---|
| 心跳周期 | 1s | |
| 状态过期 | 3s | 3 × 心跳 |
| 领队租约 / 续租 | 3s / 1s | |
| 上线静默期 `T_join` | 2s | 2 × 心跳 |
| 选举错峰 | rank × 200ms | 防止同时抢 |
| claim 冲突窗口 | 300ms | |
| intent ack 超时 | 5s | |
| MQTT keepalive | 15s | LWT 触发的下界 |
| 领队故障切换 | ≈ 3.5s | 租约 3s + 抖动 |

---

# 5. 选举状态机

```text
alive_view = { sn : status 未过期(3s) && online }
expected   = min(alive_view, key=(config.priority, sn))     ← 确定性，无需投票

FOLLOWER :
  leader 租约有效 && leader ∈ alive_view      -> 保持
  leader 租约过期                              -> 按 rank 错峰等待后转 CANDIDATE
  expected != me 但 expected 长期未声明领队    -> 将其"健康降级"，从候选集剔除后重算

CANDIDATE :
  发布 leader{leader=me, epoch=known_max+1, lease_until=now+3s}
  收到更高优先级 (priority, sn) 的 claim      -> 立即让位，回 FOLLOWER
  收到更低优先级的 claim                      -> 忽略（对方会自己让位）

LEADER :
  每 1s 续租；主动下线时发布 lease_until=now 让位
  每次收到 leader 消息 -> 自我校验（见 §7.2）
```

* **无分区 ⇒ 成员视图在一个心跳周期内收敛 ⇒ `expected` 唯一**，所以基本不需要"抢占回合"。偶发瞬态冲突靠 `(priority, sn)` 比较一句话解决。
* 优先级写进配置（`config/fleet.yaml` 的 `robot_priority`），可按算力 / 电量 / 常驻性排，不必依赖硬件 id。

---

# 6. 核心场景：原领队 A 掉线、B 上位、A 回归

这是最容易想不清楚的地方，逐步展开。

```text
t0  A(priority=1) 掉线
    → broker 立即代发 LWT，status(A) = offline (retained)

t1  B 发现 lease_until 已过
    → 发 leader{leader=B, epoch=8, lease_until=t1+3s} (retained)

t2  全员收到，接受 epoch=8；A 不在，收不到

t3  A 重新上线，订阅 fh/fleet/v1/leader
    → broker 立刻把 retained 的 {leader=B, epoch=8} 推给 A
    → A 比较：epoch 8 > 我记忆中的 7
    → 放弃，成为 follower，本地 epoch 更新为 8

t4  A 虽然 priority 最高，但在 B 的租约过期前不发起 claim

t5  B 若健康，持续续租，A 永远只是 follower
```

> **关键在第 t3 步：A 上线不是"宣布我是领队"，而是"先读黑板学当前纪元"。**

只要这一步做了，A 就不可能自封。它依赖两个必须做对的细节：

1. **上线静默期 `T_join`（2s）**：上线后先只观察不发言，读全量 retained status 建成员视图 + 读 leader 学 epoch。静默期结束仍无合法领队才发起 claim。顺带解决"新机器人入网引发全队震荡"。
2. **claim 必须 epoch+1**：A 即使真要抢，也只能发 `epoch = max(已知epoch) + 1`，绝不能复用旧 epoch。旧领队在协议层面无法用旧身份复活。

---

# 7. 防双领队的三道机制

## 7.1 机制一：epoch 持久化（防"失忆"）

A 如果是**进程重启**而非断网，内存里的 epoch 没了。若从 0 开始，就能用低 epoch 抢到领队。

```python
# 与 algorithm_mqtt_engine.py 一致，用 sqlite 持久化
epoch = max(persisted_epoch, epoch_from_retained_leader)

# claim 时
new_epoch = epoch + 1
```

retained 消息在 broker 无持久化重启时会丢，本地持久 epoch 是它的兜底；两者取 max，双向保险。

## 7.2 机制二：领队自我驱逐 fencing（防"假死"）⭐

> **比"掉线"更危险的是 A 没掉线，只是卡住了。**

GC 停顿、CPU 打满、网络抖动 → A 的续租发不出去，但进程还活着、`is_leader` 仍为 true，还在跑 planner。3 秒后 B 上位，此时 A 缓过来继续发 plan → **真·双领队，且没有任何一方"认为自己错了"**。

解法：领队自己也订阅 `fh/fleet/v1/leader`（broker 会把消息回环给自己），每次收到就校验：

```python
def on_leader_msg(self, msg):          # 领队和 follower 跑同一段代码
    if msg.epoch > self.epoch:         # 有人更新了租约
        self.step_down()               # 立刻降级
        self.abort_planning()          # 中止正在跑的 planner，丢弃结果，不发布
    elif msg.leader != self.sn and msg.epoch == self.epoch:
        self.step_down()               # 同 epoch 被别人占了（并发 claim 落败）
```

**`abort_planning()` 是重点**：降级必须能取消已经在跑的规划，否则"降级了但结果还是发出去了"等于没降级。领队在发布 plan 前也要再校验一次自己仍是合法领队。

## 7.3 机制三：epoch 单调栅栏（防"僵尸 plan"）

即使前两条全部失效，还有最后一道：

```python
def on_plan(self, msg):
    if msg.epoch < self.accepted_epoch:
        return                          # 拒绝旧纪元的 plan，静默丢弃 + 计数上报
    self.accepted_epoch = msg.epoch
    self.apply(msg.assignments[self.sn])
```

**单调性不变量**：一旦接受过 epoch=8，就永久拒绝 epoch≤7 的任何消息。旧领队 A 就算把 plan 发出来了，也没有任何机器人会执行。

三道机制的分工：

| 机制 | 保证的性质 | 失效后果 |
|---|---|---|
| epoch 持久化 | 活性 | 低 epoch 抢占成功 |
| 领队自我驱逐 | 活性 | 双领队持续存在 |
| epoch 单调栅栏 | **安全性** | —— |

> 前两层保证"尽快恢复单一领队"，最后一层保证"即使重叠也只浪费一次规划，不会产生错误行为"。

---

# 8. 并发 claim 的裁决规则

极端情况下两节点同时 claim（同时上线、都读不到 retained）。裁决规则必须**对所有人一致**：

```text
胜者 = max by (epoch 大者胜, priority 小者胜, sn 字典序小者胜)
```

因为所有节点看到同一份消息、套用同一条规则，所以即使在 A 看来自己赢了、在 C 看来 B 赢了，收敛一轮后也会一致——落败方在 §7.2 的 `on_leader_msg` 里自我驱逐。星型拓扑下这个收敛是**一个消息往返**（几十毫秒）的事。

---

# 9. 判定规则汇总（可直接贴进代码注释）

```text
节点 X 是合法领队 ⟺ 同时满足：
  1. broker 上 fh/fleet/v1/leader 的 retained 消息中 leader == X
  2. 该消息 lease_until > now
  3. 该消息 epoch == 全网已知最大 epoch

任一节点任一时刻发现三条不全成立 → 立即降级为 follower + 中止规划
任一节点想当领队 → 必须发 epoch = 已知最大 + 1 的 claim，并持续续租
```

---

# 10. 工程落点

照抄 `inspection_platform_bridge` 已有的 engine / bridge 分离模式：

```text
algorithm_mqtt_engine.py      ← 纯逻辑，publish/回调注入，可 pytest 单测
algorithm_mqtt_bridge_node.py ← paho + ROS 接线
```

照搬成：

```text
fleet_election_engine.py      ← 成员视图 + 选举状态机（零 ROS 依赖，pytest 全覆盖）
fleet_coordinator_node.py     ← paho 订阅/发布 + ROS 服务客户端
```

* **领队态**：MQTT 状态 → 拼 mission YAML → 调 `/capability_mission_planner/plan` → 读 `plan.json` → 切片 → 广播
* **非领队态**：收 `plan` → 取自己那份 → 落盘成 route YAML → 调 `/task_hub_node/start_route`

已有可复用基础：`platform_mqtt_bridge_node.py:80` 的 `sn` 参数、`:93-96` 已有的 `fleet_state_topic` / `robot_heartbeat_topic`。

## 10.1 落地时最容易踩的坑

1. **LWT 只给 `status` 设，绝不给 `leader` 设。** 否则任何一台 follower 掉线都会把领队信息清空，引发全队误选举。
2. **retained + 通配订阅 `fh/fleet/v1/robot/+/status`**：新领队上任、新机器人入网、断线重连，都能瞬间拿到全量成员视图，不需要额外"同步期"协议。这是 MQTT 白送的能力，要用满。
3. **版本对齐**：planner 部署在多台机器上一定会漂移，地图包也可能不同步。`plan` 必须带 `planner_version` + `map_digest`，follower 校验不一致就拒绝执行并上报——否则会出现"按 A 版本算的路线在 B 版本地图上撞墙"。
4. **一致性快照用两阶段，不要"想到就算"**：领队先发 `intent{mid, epoch, members[], input_digest}`，等成员 ack（同意 / 忙 / 电量低），再算再发 `plan`。否则会出现"给正在手动模式的机器人派活"。
5. **计划下发用一条广播，不要按机器人拆 topic**。所有机器人必须基于同一个 plan 版本行动，一条 retained 消息天然保证原子性；按机拆分可能出现一半旧一半新。payload 控制在 256KB 内。
6. **时间同步是硬前提**：CBS 产出的是带 `arrival_tick` 的调度，各机时钟不同步则时刻表毫无意义。**先上 chrony/NTP 再谈多机协调**。同时记住 CBS 只是参考调度，真实执行仍靠 `LiveReservationTable` + Nav2 重规划兜底，不要当硬实时。

## 10.2 一个具体的阻塞点

`StartRoute.srv` 只有 `route_config_path` 一个字段，`task_hub_services.cpp:484` 也只认路径、不支持内联 YAML。所以本机桥必须先把收到的 YAML 落盘再传路径。

**建议**：给 `StartRoute.srv` 加一个可选 `route_yaml` 字段（向后兼容），比落盘干净，也省掉文件版本管理。

---

# 11. 分阶段落地计划

| 阶段 | 内容 | 验收标准 |
|---|---|---|
| **0 前置** | 统一地图包 + planner 版本、NTP、`sn` 唯一、broker 定为 mosquitto 2.x | 三机 `date` 偏差 < 100ms |
| **1 成员视图** | status 上报 + LWT + retained，先只做只读 fleet monitor | 杀掉任意进程，3s 内全队视图更新 |
| **2 选举** | 静态优先级 + 租约，起 3 进程模拟，`kill -9` / 断网注入 | 切换 < 4s，全队 leader 一致，无双领队 |
| **3 领队规划** | mission assembler 拼 YAML → 调 planner → 读 plan.json | 单机上能出多机 plan |
| **4 下发执行** | intent/ack 两阶段 → plan 广播 → 落盘 → `start_route` | 3 台真机跑通一次巡检 |
| **5 容错** | epoch 提升重规划、领队切换后靠 retained 恢复、失败单机降级 | 规划中途杀领队，任务不中断 |

---

# 12. 失效边界

诚实起见，有三种情况上述机制会打折：

1. **broker 重启且未开持久化** → retained `leader` 丢失 → 只剩本地持久 epoch 兜底，可能出现**短于一个租约期（3s）** 的双领队窗口。§7.3 保证安全，只是多算一次规划。解法：mosquitto 开 `persistence true`。
2. **时钟漂移** → 租约依赖 wall clock，必须上 chrony。且**租约判断用消息里的 `lease_until` 绝对值对比，不要用"我收到消息的时间"**，能显著降低对时钟的依赖。
3. **跨站点多 broker 桥接** → 星型无分区的前提失效，必须引入 quorum（多数派）。epoch 机制保留但不够。当前单站点 N ≤ 20 不需要。

---

# 13. 明确不要做的事

* 不要在 MQTT 上实现全序广播（QoS1 只保证单发布者有序，够用了）。
* 不要一上来做分布式优化（拍卖 / CBBA）——已经有全局求解器，那是自找麻烦。
* 不要让 broker 侧跑任何业务进程，否则又变回集中式，只是换了个名字。
* 不要给 `leader` topic 设 LWT（见 §10.1 第 1 条）。

---

# 14. 待确认事项

1. **机器规模 N** 与是否跨站点 / 跨 broker？
2. **broker 型号与 MQTT 版本**：mosquitto 2.x 支持 MQTT5；若能用 MQTT5 的 `message expiry` + `session expiry`，状态过期逻辑可以少写不少。
3. **任务集是平台预置的静态 YAML，还是含动态状态（电量 / 位姿）？**
   如果完全静态，"各机本地跑同一个确定性 planner、各取所需"可以**彻底取消领队**，这是条更短的捷径，值得先确认。
