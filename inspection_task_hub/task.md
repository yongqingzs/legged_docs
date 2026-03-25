## 任务中枢设计
为了应对机器人巡检任务，需要设计一个任务中枢来协调和管理各项巡检活动。中间件为 ros2-jazzy。消息类型和内容由该模块指定。其具备以下功能:
1. 导航点管理: 维护不同的导航路线(可以用不同的 yaml 文件管理)，导航路线下有相应的导航点序列(导航点为三维坐标)，并通过特定的消息类型(由你指定)发送给导航节点。
2. 导航点任务管理: 每个导航点有相应的任务(写在 yaml 文件里)，为任务1、任务2、任务3等(任务数量不固定，有不同的任务类型)。不同任务类型由不同的任务节点执行，不是在任务中枢里执行，但需要任务中枢发送任务指令给相应的任务节点(消息类型、消息内容由任务中枢维护)。
3. 任务状态监控: 监控各个任务节点的状态(通过订阅相应的消息)。例如: 在导航点1中，有任务1、2,完成任务1才能进入任务2，任务2完成后才能进入下一个导航点。如果执行错误，发出任务执行失败的消息(失败原因、失败时间、任务类型等)
4. 任务日志记录: 记录每个导航点的任务执行情况，包括任务开始时间、结束时间、执行结果等信息。日志可以存储在本地文件中，便于后续分析和优化。
你需要编译通过该模块，该模块放在 ~/nav_ws/src 下，编译指令为 cd ~/nav_ws && MAKEFLAGS="-j4" colcon build --packages-up-to XX。
并且需要有一个完整的导航和任务执行流程测试，完成后生成一个总体文档(仅一个)。


## 任务中枢优化
mark_task_completed 当前不会实际执行，其执行应该与具体的任务类型相关。你当前需要实现以下任务类型的处理。
1. 定点拍照任务，发送双光云台的指定位置和角度(消息类型和内容由你指定)，当对方定点拍照节点完成任务后，接受相关任务完成的消息(消息类型和内容由你指定)，确定任务完成。
完成后执行 mark_task_completed。
在 test 中添加 routes.yaml 的全流程测试，你之前搞错这个测试的含义了。应该有个专门的测试节点，其会将 yaml 中的 waypoints、tasks 依次触发一遍，并且有不同的情况(比如可恢复故障、不可恢复故障等)。完成后编译和测试通过。不要新增 md 文件，有什么直接和我说。


## 任务中枢优化1
task_hub_workflow_test.cpp 不要放在 src 下，放在 test 或其他位置。GimbalControl.msg 和 PhotoTaskResult.msg 等和具体任务相关的 msg 可以放在 msg/ 下专门的 task 文件夹里。yaml 维护的航迹点应该是 x、y、z 和姿态四元数(Navigation2的全局规划消息类型)。另外: task_hub_workflow_test.cpp 中的实现有问题，不应该直接去调用 coordinator_.XX 等内部函数，而是像真实情况一样，发送消息去逐步触发(任务类型可先只用拍照任务)。完成后请编译测试通过，不要生成 md，有什么直接和我说。


## 任务中枢优化2
现在任务中枢的实现存在以下问题:
1. task 的触发可以绕过 waypoints，但实际上所有 task 都必须在到达相应的 waypoints 后才执行
2. waypoints 的交互也走 publish 模式，其应该最好走 action 模式，相当于让中枢知道其运行时反馈
3. 我不理解为什么 task_hub_node 中有 on_photo_task_result 函数，那么对于拍照结果的处理就在总节点中进行了，而 task_coordinator 中又有 execute_photo_task 和 on_photo_task_result，实际task_coordinator 没进行实质性工作。你的实现思想是什么？是否应该将 task 相关逻辑的处理都放在 task_coordinator。
4. task_coordinator execute_photo_task 时应该是向外放送双光云台的控制信息(放给双光云台控制节点，其具体和双光云台交互)，但我并没有看到。
你实现完后编译通过，并根据这些实现覆写 task_hub_workflow_test.cpp，使其能够测试消息触发流程。


## 任务中枢优化3
这是之前实现的任务中枢: "为了应对机器人巡检任务，需要设计一个任务中枢来协调和管理各项巡检活动。中间件为 ros2-jazzy。消息类型和内容由该模块指定。其具备以下功能:
1. 导航点管理: 维护不同的导航路线(可以用不同的 yaml 文件管理)，导航路线下有相应的导航点序列(导航点为三维坐标)，并通过特定的消息类型(由你指定)发送给导航节点。
2. 导航点任务管理: 每个导航点有相应的任务(写在 yaml 文件里)，为任务1、任务2、任务3等(任务数量不固定，有不同的任务类型)。不同任务类型由不同的任务节点执行，不是在任务中枢里执行，但需要任务中枢发送任务指令给相应的任务节点(消息类型、消息内容由任务中枢维护)。
3. 任务状态监控: 监控各个任务节点的状态(通过订阅相应的消息)。例如: 在导航点1中，有任务1、2,完成任务1才能进入任务2，任务2完成后才能进入下一个导航点。如果执行错误，发出任务执行失败的消息(失败原因、失败时间、任务类型等)
4. 任务日志记录: 记录每个导航点的任务执行情况，包括任务开始时间、结束时间、执行结果等信息。日志可以存储在本地文件中，便于后续分析和优化。
你需要编译通过该模块，该模块放在 ~/nav_ws/src 下，编译指令为 cd ~/nav_ws && MAKEFLAGS="-j4" colcon build --packages-up-to XX。"
但现在该中枢存在很多问题，例如:
1. waypoints 的交互应该通过 action 机制，而不是 publish; 只有每个 waypoints 下的 task 执行完了才能进入下一个任务
2. task_hub_node 和 task_coordinator 职责不清，如 task_hub_node 中有大量 photo 任务的逻辑，这些具体任务是否应该都放在 task_coordinator？你的看法如何？
3. test/task_hub_workflow_test 对 yaml 文件的测试并不成功，其需要通过消息触发将全流程走完
完成后编译，并且要运行通过 task_hub_workflow_test(需要将所有 waypoints 和相应的 tasks 都执行完)


## 任务中枢优化4
当前项目存在的问题:
1. yaml 中 gimbal_pose 的形式是不是过于繁琐，x y z qx 等，和 position orientation 的表示不一致。
2. definitions.hpp 中 struct TaskDef 的表示是否过于冗余，为什么有两个 timeout_seconds？如果继续增加任务类型，TaskDef 会持续变大，是否有更好的表现方式。
3. 现有 yaml 是发送消息后读取，能否实现 yaml 的动态读取？也就是可以通过发送 yaml 的地址，动态加载新的 yaml，并且不会影响现有功能。
4. task_hub_workflow_test.cpp 中只体现了一种情况，其实实现多个例程，将不同的情况都覆盖，比如:
    a. 某一 task 没有成功
    b. 某一 waypoint 没有到达
    c. 任务中枢暂停并重新执行
    d. 任务中枢停止
    e. max_retry 是否有用
等等，根据你的思考继续添加。
完成后，请在 ~/nav_ws 下编译并运行通过。


## 任务中枢优化5
当 route 执行完后，我再次发 route: "[INFO] [1774409929.039760179] [task_hub_node]: [INFO] All waypoints completed successfully
[WARN] [1774409935.971827293] [task_hub_node]: Route is already active. Please call /cancel_route before calling /start_route again." 这种其实不对，当 route 执行完后，应该认为已经 cancel_route。
另外，会出现 waypoint 重复输出的情况"[INFO] [1774409810.504688391] [task_hub_node]: [INFO] Starting navigation to waypoint 1
[INFO] [1774409811.605467373] [task_hub_node]: [INFO] Starting navigation to waypoint 1"