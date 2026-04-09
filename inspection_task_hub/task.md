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


## 任务中枢优化6
现在和 waypoints 交互用的是自定义接口，现在想替换成 nav2_msgs/action/NavigateToPose 的 action 接口。而和云台控制交互的也是自定义接口，现在想替换成 control_msgs/FollowJointTrajectory 接口，从而减少额外依赖。并且修改 task_hub_workflow_test.cpp，使测试用例更新。完成后编译通过"cd ~/nav_ws && cbu inspection_task_hub --symlink-install"，并且能正常运行，不完成请不要结束。


## 任务中枢优化7
我将替代的自定义消息相关依赖去掉后，出现错误:"FileNotFoundError: [Errno 2] No such file or directory: '/home/jazzy/nav_ws/build/inspection_task_hub/rosidl_generator_type_description/inspection_task_hub/msg/WaypointStatus.json'"，请修复并编译，并能正常运行。另外云台控制应该就是两个角度: "pan_joint（水平） tilt_joint（俯仰）",现在云台六自由度不符合实际需求。


## 任务中枢优化8
像 yaml 中的 gimbal_pose 也不应该用六自由度了，而是 pan_joint, tilt_joint。并且我发现发送给导航节点的点位似乎不是从 yaml 中读取的，请确认这件事情并修复。编译通过，并能正常运行。


## 任务中枢优化9
当前任务中枢有一些需要修改项:
1. 当前，每到一个 waypoint，就执行任务，任务执行完毕后，才触发下一个 waypoint。需要修改成: 既可能是执行任务，也可能是停留一段时间(在 yaml 里控制)。
2. 执行时间问题: 目前任务均是到达 waypoint 后执行，也存在到达前执行(类似 point1 和 point2 之间的任务)。
3. 需要增加循环跳转节点的功能: 完成 yaml 里的 waypoint 后，可以跳转到指定的 waypoint 持续执行，同时需要考虑循环跳转的逻辑性问题(次数、终止)。
4. 实际执行 waypoint 巡检前后，需要发送启动/停止命令，告知导航节点，以接入机器狗控制。启动/停止命令使用官方的 Trigger 消息。
完成后编译通过"cd ~/nav_ws && cbu inspection_task_hub --symlink-install"，并且能正常运行，不完成请不要结束。


## 任务中枢优化10
OK，干的很好。当前只有拍照任务，但其实这是两个任务，需要重新拆开:
1. 云台任务: 调整云台姿态(俯仰、偏航)
2. 拍摄任务: 分两种，一是录像，需要告知录制时间; 二是拍照，包括次数、间隔时间、焦距等。
这些消息交互都用 server/action 进行。
完成后编译通过"cd ~/nav_ws && cbu inspection_task_hub --symlink-install"，并且能正常运行，不完成请不要结束。


## 任务中枢优化11
生成一个新的测试节点，放在 test 下，它的目的是为了可交互测试。例如: 任务中枢节点发送 waypoint 请求，该节点可以人为在终端选择是否通过; task 等也类似。需要涵盖各种情况，进而更好地接近真实巡检场景。
完成后编译通过"cd ~/nav_ws && cbu inspection_task_hub --symlink-install"，并且能正常运行，不完成请不要结束。


## 任务中枢优化12
需要修复项:
1. 当前 Trigger service 不触发也可以进入下一环节，但实际上 Trigger service 必须触发成功才可以，因为如果启动不成功，就不应该发点。
2. 交互测试节点进入发点、执行任务这些交互后，"查询状态、暂停、恢复"这些功能都无法使用了。


## 任务中枢优化13
现在 gimbal 任务只发云台姿态调整信息，但其实还应该在该调整信息中添加"校准类型"、"校准提示"，告知云台是否要根据校准类型(如: 仪表)进行微调，并包含一个校准提示(这是一个 string 类型，具体信息根据实际情况变动，如: A(表示A仪表))。并且修改 test 下的测试用例和 README。完成后编译通过"cd ~/nav_ws && cbu inspection_task_hub --symlink-install"，并且能正常运行，不完成请不要结束。


## 任务中枢优化15
现在 gimbal 任务发的是云台的姿态角，还有一种方式: 发送云台和目标的点云地图坐标位置。添加这种方式，并设置成可选择项(姿态角或双坐标)，并用相应的类型进行标记(和发姿态角区分)。说明:
1. 双坐标中的目标位置在 yaml 里写入
2. 双坐标中的云台位置先从 waypoint 交互中的最后 feedback 中读取机器狗位置，然后加上云台的安装偏差
3. 云台安装偏差写在 yaml 里
并且修改 test 下的测试用例和 README。完成后编译通过"cd ~/nav_ws && cbu inspection_task_hub --symlink-install"，并且能正常运行，不完成请不要结束。


## 任务中枢优化16
现在有新增需求项:
1. 当前使用 "navigate_to_pose" 和导航服务通信，现在改为 "multi_map_navigate_to_pose"，并且 frame_id 从 yaml 中读入。
2. 增加并行任务可选项，但需要维护一个集合(任务对)，只有这个集合里的任务对才能并行。现在可以并行的任务只有 "云台移动(不微调) + 录像"。
3. 在发送 Trigger 成功后，还需要新建一个 client，它将 yaml 中维护的所有点(包含 frame_id )发出，以供导航验证这些点的可通行性(并不是实际运行点)。
同时修改 test 下的测试用例和 README。完成后编译通过"cd ~/nav_ws && cbu inspection_task_hub --symlink-install"，并且能正常运行，不完成请不要结束。


## 任务中枢优化17
终端1: "ros2 run inspection_task_hub task_hub_node"，终端2: "ros2 run inspection_task_hub task_hub_workflow_test"，实际运行后会出现"[INFO] [1775614399.931081612] [task_hub_node]: Sent gimbal goal for task 101 (type=angle, camera=visible, calibration_type=instrument, calibration_hint=A)
[ERROR] [1775614399.934455759] [task_hub_node.rclcpp_action]: unknown goal response, ignoring...
[INFO] [1775614399.935895064] [task_hub_node]: [INFO] Gimbal result task_id=101, success=false
[INFO] [1775614399.935934672] [task_hub_node]: [INFO] Task 101 failure observed, retry=1/1
[ERROR] [1775614399.935952366] [task_hub_node]: Task failed: Task 101 failed: Gimbal task failed: FollowJointTrajectory failed, error_code=-1, error_string=state=FAILED reason=camera camera_open_failed: pipeline_not_ready_at_startup detail=timeout waiting for camera image publishers
[INFO] [1775614400.032173704] [task_hub_node]: Trigger inspection_stop succeeded: interactive mock inspection_stop ok
"，请进行详细的测试，解决相关问题。


## 任务中枢优化18
现在有一些待修改的问题:
1. 我用 ctrl + c 退出 /task_hub_interactive_test_node，发现在一段时间里仍然有 /task_hub_interactive_test_node 残余。
2. 比如有两个任务是不支持并行的，但我在 yaml 中设置成并行，运行起来却无阻碍。这是不合理的，而且现在并行逻辑是可靠的吗，为什么我改成 allow_parallel: True 后似乎也是单步执行。在读取 yaml 后就应该做一次分析，比如两个或更多设置为并行的任务是否可通过，不可通过就返回该 yaml 有问题，而不是继续执行。
如果有些测试项你难以通过测试用例覆盖，你可以在终端直接发送消息进行测试。


## 任务中枢优化19
现在有一些待修改项:
1. 我故意修改 yaml 发送错误的并行对，但发现 task_hub_node 直接卡在这:
"[INFO] [1775618981.146469531] [task_hub_node]: Received start_route request: /home/jazzy/nav_ws/install/inspection_task_hub/share/inspection_task_hub/config/routes.yaml
"
2. 构思各种情况，测试项尽量覆盖全。如果难以通过测试用例覆盖，可以在终端直接发送消息进行测试。


## 任务中枢优化20
现在有一些待优化项:
1. 当前的暂停和恢复机制有些问题。我发送暂停后，应当给各个 action(当前在执行的任务模块，包括在前往下一个点的导航) 发送 cancel，待模块回应后(需要注意并行任务问题)才真正显示暂停，而本地应该存储待完成的任务/导航点。待恢复后，重新发送原先(未完成)的任务，终端也应该有相应的显示。
2. 在本地维护一个当前任务进度表，以记录当前情况。可以放在 inspection_task_hub/ 的日志文件夹(日志文件夹最好不要和 logs/ 重名)，日志文件夹可以从外部指定。
3. 在本地也记录 Event log，放在 inspection_task_hub/ 的日志文件夹。
同时修改 test 下的测试用例和 README。完成后编译通过"cd ~/nav_ws && cbu inspection_task_hub --symlink-install"，并且能正常运行，不完成请不要结束。


## 任务中枢优化21
当前 task_hub_node.cpp 过于臃肿了，你应该做一个功能拆解，将其功能放在各个模块中(新的或旧的)，你可以新建模块。
同时修改 test 下的测试用例和 README。完成后编译通过"cd ~/nav_ws && cbu inspection_task_hub --symlink-install"，并且能正常运行，不完成请不要结束。


## 任务中枢优化22
我测试 pause/resume 功能的时候出现问题。终端1: "ros2 run inspection_task_hub task_hub_node"，终端2: "ros2 run inspection_task_hub task_hub_interactive_test"。终端2 显示"
[interactive-test] command> p
pause_route => success=false, message='Failed to pause route: cancel/ack from active goals timed out' 
[interactive-test] command> r
resume_route => success=false, message='Failed to resume route'
[interactive-test] command> r
resume_route => success=false, message='Failed to resume route'
"，终端1 显示 "[INFO] [1775705769.420286050] [task_hub_node]: Received pause_route request
[INFO] [1775705780.444325740] [task_hub_node]: Received pause_route request
[INFO] [1775705792.828055179] [task_hub_node]: Received resume_route request
[INFO] [1775705797.623385059] [task_hub_node]: Received resume_route request
"， pause_route 直接卡住，而 resume_route 却能在 pause_route 未完成的时候接收。
完成后编译通过"cd ~/nav_ws && cbu inspection_task_hub --symlink-install"，并且能正常运行，不完成请不要结束。