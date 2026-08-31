# @/home/jazzy/drive_ws/src/nav_bridge 完成了哪些功能，输出 markdown 到 /home/jazzy/agent_ws/src/legged_docs/nav-real/nav_bridge 下。

问题:
1. 我现在想新增 nav_bridge 适配智元 d1 max，这是 d1 max 的 sdk: @/home/jazzy/drive_ws/src/Agibot_D1_Max(ubuntu22.04 的 sdk，sdk 文档 @/home/jazzy/drive_ws/src/Agibot_D1_Max/ubuntu22.04(x86_64_arm64)/RobotSDK-0.1.1/docs)，请分析并说明你的适配方案，写入 /home/jazzy/agent_ws/src/legged_docs/nav-real/nav_bridge 下

2. 不要继续扩展当前 NavBridgeBase，应先引入 RobotBackend(进行真正的后端抽象)，再迁移 X30；完成 x30 迁移后根据 @/home/jazzy/drive_ws/src/Agibot_D1_Max 迁移 d1 max，但注意 nav_bridge 最终跑在 arm64 上

3. 你不要链接 @/home/jazzy/drive_ws/src/Agibot_D1_Max 上的 lib，你如果需要使用 @@/home/jazzy/drive_ws/src/Agibot_D1_Max 中的文件，直接复制进 nav_bridge 下某目录(看你觉得哪里合适，或者 third_party/)

4. 我已经编译了 @/home/jazzy/drive_ws/src/Agibot_D1_Max/ubuntu22.04(x86_64_arm64)/RobotSDK-0.1.1/example/build，这些例子分别代表什么？我已经通过无线网连接 192.168.234.1(ssh robot@192.168.234.1 密码: bot)，但发现运行"./data 192.168.234.1 8081"出现"[ERROR] Connect failed: Robot ShakeHand failed [ERROR] Not connected. State: 3"，请查找原因。

5. 刚才适配的 @/home/jazzy/drive_ws/src/Agibot_D1_Max 是老版本的，我拿到实际可用的新版本 @/home/jazzy/drive_ws/src/RobotSDK-0.2.1，里面的 example/build/control 例程该如何使用以控制机器狗(按键分别表示什么)，并更新 @/home/jazzy/agent_ws/src/legged_docs/nav-real/nav_bridge/D1_MAX_SDK_EXAMPLES_AND_HANDSHAKE.md

6. nav_bridge 刚才新增 d1 max 支持依据 @/home/jazzy/drive_ws/src/Agibot_D1_Max(版本 0.1.1)，而 @/home/jazzy/drive_ws/src/RobotSDK-0.2.1 才是最新的，请根据最新 sdk 更新代码和链接库

7. 请根据你最新的适配方案，更新 @/home/jazzy/agent_ws/src/legged_docs/nav-real/nav_bridge/D1_MAX_ADAPTATION_PLAN.md，说明你对 d1 max 的实际适配，并和 x30 比较

8. 我看 nav_bridge 中还是有 nav_bridge_base.hpp、x30_nav_bridge.hpp 等文件，说明 x30 还是用的以前的适配方案，没有接入 robot_backend? 这符合预期吗，先讨论，不要修改源码。
