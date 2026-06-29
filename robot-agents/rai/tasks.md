## task1
功能需求:  
需要添加生产用的持久化记忆层，分为长期记忆、短期记忆，使用现有 langchain 工具完成，但需要独立的模块进行记忆层的管理。普通会话放在短期记忆层，一些特征信息放在长期记忆层(比如语义地图中各点位的位置)。
使用 postgres 数据库进行管理, 数据库路径需要可通过 config.yaml 配置。  
请基于 /home/user/codes/Intern/rai/examples/rosbot-xl-demo.py 演示效果(uv run)。

- 记忆层描述
1. 短期记忆（或称线程范围记忆）
通过在会话中维护消息历史记录来跟踪正在进行的对话。LangGraph 将短期记忆作为代理状态的一部分进行管理。状态使用检查点持久化到数据库中，以便随时恢复线程。当图被调用或步骤完成时，短期记忆会更新，并且在每个步骤开始时都会读取状态。
2. 长期记忆
长期记忆存储用户特定数据或应用程序级别的数据，这些数据跨会话存储，并在不同的对话线程之间共享。它可以在任何时间、任何线程中被调用。记忆的作用域限定在任何自定义命名空间内，而不仅仅局限于单个线程 ID。

- postgres 容器使用示例
见 /home/user/codes/Intern/rai/demo/for_langchain/short_post1.py

- langchain 记忆工具相关说明
见 /home/user/codes/Intern/legged_docs/robot-agents/rai/memory/langchain.md

## task2
现在记忆层的实现有问题，记忆层说明:
1. 系统记忆：像 /home/user/codes/Intern/rai/examples/embodiments/rosbotxl_embodiment.json 这种是系统记忆，始终在上下文中。
2. 短期记忆压缩策略：按照一定 tokens 数进行摘要，token 阈值（如 8192）。checkpoint 里保存完整摘要链，state 中只保留: summary + 最近 N 轮。
3. 记忆 schema： 两种记忆 schema，在 store 里用不同 namespace 区分：
  ("default", "robot_user", "facts")    →  {"text": "prefers dark mode", "ts": ...}
  ("default", "robot_user", "spatial")  →  {"location": "kitchen", "pose": {...}, "objects": [...]}
先不做单独索引，纯文本 embedding 搜索够用。
4. 压缩后需要恢复原始对话，不过恢复时 LLM 读取摘要而不是全部信息。 checkpoint 保存了每个步骤的快照，如果需要调试，get_state_history(config) 能看到每一步的完整状态（包括压缩前的）。但 LLM 在恢复 session 时只看到摘要。
5. agent 会自行判断当前情况(prompt 是否有语义特征信息，如包含哪些语义地图)，调用相关 tool 进行存储。
6. agent 对长期记忆层有增删改查的能力。删除需要用户确认。

## task3
当前长期记忆的考虑主要是因为机器人长期记忆不会很长，全量注入以减少嵌入模型的使用。现有的删除流程存在问题: 
1. 我在页面给 agent 发送"删除当前长期记忆" 
2. agent 显示调用记忆删除工具，但工具返回"需要用户确认yes"（但没有交互按键弹出）, agent 显示"需要用户确认" 
3. 我输出 yes，agent 再次调用记忆删除工具，工具再次返回"需要用户确认yes"（但没有交互按键弹出）  
这个流程明显有问题，你怎么看？

## task4
比较合理的方案是：
- forget_memory 工具只负责生成待确认删除请求，并把 confirmation key 结构化保存。
- UI 在检测到 ToolMessage 或任意消息中的 confirmation key 后，进入 pending 状态。
- pending 状态下，用户输入 yes/confirm 时，绝不能再送进 agent，而是直接调用 confirm_deletion(key)。
- 页面最好显示明确的确认/取消按钮，而不是让用户猜要输入什么。
- regex 应改成能匹配 UUID，例如 r"Confirmation key:\s*([a-fA-F0-9-]+)"，或者更稳的是工具返回 JSON/结构化内容，不靠正则扒文本。

另外，工具描述里写了 “Re-invoke with the confirmation_key to confirm deletion”，但 ForgetMemoryToolInput 只有 query 字段，没有
confirmation_key 字段。这也会误导模型。当前设计实际是“UI 确认”，不是“agent 二次调用确认”。这部分文案应该删掉或改掉。

## task5
运行"uv run streamlit run examples/rosbot-xl-memory-demo.py"，agent 显示:
1. 调用两次删除工具，分别删除两个地点
2. 但是没有弹出按钮，让我确认删除
进程直接失败，并且显示"Agent error: Error code: 400 - {'error': {'message': "This model's maximum context length is 262144 tokens. However, you requested 0 output tokens and your prompt contains at least 262145 input tokens, for a total of at least 262145 tokens. Please reduce the length of the input prompt or the number of requested output tokens. (parameter=input_tokens, value=262145)", 'type': 'BadRequestError', 'param': 'input_tokens', 'code': 400}}".
我现在的想法是先不搞这么复杂的确认逻辑了，agent 拥有删除长期记忆的权限。请修复，并且你需要实际测试过可用再交付。

## task6
现在这个长期记忆存储工具存在问题，出错过程:
1. 我让 agent 记住两个点位的位置: "记住两个位置: Kitchen (center): (-0.2175, -0.8775, 0.0), Living Room (center): (-0.82, 3.525, 0.0)."
2. agent 调用 save_location
3. tool output 为空，终端显示 error:"
    "msg": "Input should be a valid dictionary",
    "input": "{\"x\": -0.82, \"y\": 3.525, \"z\": 0.0}""
请分析这个问题，告诉我你的解决方案，先不要修改代码。

## task7
实施方案:
1. 删除 prompt 对 SaveLocationToolInput.pose 的简单约束
2. 用更严格的结构化类型，现在 pose: Optional[dict] 太松也太模糊。更稳的是改成明确的 PoseInput(x, y, z) 这种 Pydantic 模型，模型更容易按结构输出，验证也更清楚。
完成并实际测试通过。

## task8
完成短期记忆修改方案:
1. 外层 graph 管短期记忆，内层 ReAct 无状态化。
2. 每轮传给带 operator.add reducer 的 graph 时，只传新增 HumanMessage，不要传完整历史。
3. session 切换时明确从 checkpoint 读取并恢复 UI messages/summary。
4. summary 应按 thread_id 存储和恢复，不要只放 Streamlit 当前 session。
5. 短期记忆压缩后要保证 checkpoint 中也保存压缩后的状态，而不是继续保存无限增长的原始消息。

## task9
待办:
1. 短期记忆、长期记忆那里添加删除选项
2. 添加新会话这个按钮是无效的
3. 长期记忆那里是否需要添加按钮？

完成方案:
1. 修复 + New Session 无效。
2. 给短期记忆 session 增加删除按钮。
3. 给长期记忆增加可查看列表和单条删除。

## task10
出错过程:
1. 我和 agent 对话"你好"
2. agent 出现"Agent error: Error code: 400 - {'error': {'message': 'System message must be at the beginning.', 'type': 'BadRequestError', 'param': None, 'code': 400}}"
待办:
1. 再次进入网页时，对话应该使用最新那个，而不是再次新增
2. 长期记忆的 user 应该也有删除选项

完成方案:
1. 先修 SystemMessage 顺序问题，这是当前阻塞。
2. 再修默认进入最新 session，避免无意创建 session。
3. 最后加 user 级长期记忆删除。

## task11
分析问题，先不要修改代码:
1. 按添加 session 后，页面像直接卡住一样，Short-Term Memory 框下之前的 session 不见了(刷新页面又有了)
2. 现在按钮是"删除用户记忆"，但我觉得"删除用户更合适"似乎更合适，并且希望有添加用户的功能

完成方案:
1. Session selectbox 永远渲染，新建 session 只是插入并选中新 thread id。
2. 把按钮改成 Delete User，删除 profile + facts + spatial。
3. 新增 user profile namespace，例如：
  (config.namespace, "__users__", "profiles")
4. _get_user_ids() 合并：
  - profile 中的 user_id
  - 现有 memory namespace 推导出的 user_id
5. sidebar 增加 New User 输入框和 Add User 按钮。
6. 删除当前 user 后切回 default 或第一个剩余 user。


## task12
分析问题:
1. 我选择 robot_user 删除，但是 User 还是显示有 robot_user，再次刷新页面，还是有 robot_user
2. 当前添加 user 的样式有些丑陋(有个单独的 new user 框)，是 streamlit 的限制吗(不能弹出新框，或别的表现形式)，还是设计的问题


## task13
分析问题:
1. 我发现删除 robot_user 这种行为还是无法成功，不刷新还有 robot_user，刷新也有


## task15
这个 demo: /home/user/codes/Intern/rai/examples/rosbot-xl-memory-demo.py 里有什么内容可以抽象出来作为通用项的，从而在别的 demo/应用 中复用

最值得抽象的通用项

1. Memory-aware graph builder
    当前 examples/rosbot-xl-memory-demo.py:167 基本是通用的：
    - 外层 graph 管 checkpoint
    - 内层 ReAct 无状态
    - enrich_prompt -> summarize -> react
    - summary 压缩
    - system prompt 临时注入，不写 checkpoint
    - long-term memory 注入

    可抽成类似：

    create_memory_react_agent(
        memory_mgr,
        llm,
        tools,
        system_prompt_builder,
        user_id,
        namespace,
        token_threshold,
        keep_recent,
    )

2. 短期记忆 session 管理
    这些都应抽到 rai.memory 或 rai.frontend.streamlit：
    - _get_session_ids
    - _get_latest_session_id
    - _delete_session
    - _graph_config
    - _load_thread_state
    - welcome fallback 逻辑可以参数化

    这些和 ROSBot 完全无关。
3. 长期记忆 user/profile 管理
    现在 user profile 已经是通用概念：
    - _user_profile_namespace
    - _add_user_profile
    - _get_user_ids
    - _delete_user_long_term_memory
    - tombstone 逻辑
    - _list_long_term_memory_items
    - _format_long_term_item

    建议抽成 rai.memory.users 或 rai.memory.profile_store。
    特别是 tombstone 逻辑不该散落在 demo 文件里，否则别的 app 也会踩同样坑。
4. Long-term memory rendering / formatting
    facts 和 spatial 的格式化可以抽：

    format_memory_item(schema, key, value) -> str

    这个用于 sidebar、debug UI、CLI 都有价值。

5. Streamlit sidebar memory controls
    这部分也高度可复用：
    - session selectbox
    - new session button
    - delete session button
    - user selectbox
    - add user popover/form
    - delete user button
    - facts/locations list + delete item

    可抽成：

    render_memory_sidebar(memory_mgr, graph, namespace) -> MemoryUISelection

    返回：

    dataclass MemoryUISelection:
        thread_id: str
        user_id: str
        messages: list
        summary: str

6. Embodiment JSON loader
    _load_embodiment() 也可复用，但它更像 “robot/system profile loader”，不是 memory 核心。可以放到 demo util 或 rai.whoami 相关位置。
    不过它现在假设 JSON 字段是 description/rules/capabilities，可复用性中等。

不建议抽象的内容

- SYSTEM_PROMPT_TEMPLATE 里 ROSBot XL 的身份、能力描述。
- EMBEDDING_PATH。
- WaitForSecondsTool() 是否加入工具列表，这取决于 demo。
- 页面标题、欢迎文案、sidebar 文案。
- “长期记忆全量注入”的具体上限值：可以参数化，但不要固定成框架默认。

推荐模块边界
我会这样拆：

src/rai_core/rai/memory/
  session.py        # thread/session checkpoint helpers
  users.py          # user profile/tombstone helpers
  long_term.py      # list/format/delete facts/spatial items
  graph.py          # create_memory_react_agent / MemoryState

src/rai_core/rai/frontend/
  memory_streamlit.py  # render_memory_sidebar / memory UI controls

demo 最终应该只剩：

- ROSBot prompt template
- embodiment path
- tool list
- 调用通用 create_memory_react_agent
- 调用通用 render_memory_sidebar
- chat render/input

优先级建议

1. 先抽 users.py 和 session.py，因为它们最稳定、风险最低。
2. 再抽 Streamlit sidebar，这能立刻减少 demo 文件复杂度。
3. 最后抽 graph builder，因为这里和不同 agent/tool/prompt 的接口设计要更谨慎。


## task16
分析问题:
让 agent 记忆地点时，会固定调用失败一次，第二次才成功:
"2026-05-27 22:52:18 user rai.agents.langchain.core.tool_runner[907388] INFO Running tool: save_location, args: {'description': 'The toilet located at the specified coordinates.', 'location_name': 'Toilet', 'objects': '[]', 'pose': '{"x": 0, "y": 0, "z": 3.0}'}
2026-05-27 22:52:19 user rai.agents.langchain.core.tool_runner[907388] INFO 
                                    Validation error in tool save_location:
                                    SaveLocationToolInput
                                    Number of errors: 1
                                    Errors:
                                    [
  {
    "type": "list_type",
    "loc": [
      "objects"
    ],
    "msg": "Input should be a valid list",
    "input": "[]"
  }
]"

修复方案：
1. 给 SaveLocationToolInput.objects 加 field_validator("objects", mode="before")
    - None 保持 None
    - list 原样返回
    - str 时尝试 json.loads(value)
    - 解析后如果是 list，返回
    - 如果是空字符串，也可以按 [] 处理
    - 其他情况交给 Pydantic 报错
2. _run 的类型也同步放宽成：

    objects: Optional[list[str] | str] = None

    虽然 Pydantic 会先处理，但保持 schema 和运行签名一致更清楚。

3. 补测试：
    - objects=[] 正常
    - objects="[]" 正常
    - objects='["sink", "door"]' 正常
    - 非 list JSON，例如 objects='{"a":1}' 应该失败
4. prompt 可以顺手加强，但不应该只靠 prompt：
    - 明确 objects 是数组，例如 objects: []
    - 但模型仍可能输出字符串，所以 validator 才是根治。


## task17
分析问题:
1. agent 调用工具时，页面上会显示"完成工具调用"以及工具调用的情况。但是页面刷新或重新加载后，"完成工具调用"框就会变成空白。

修复方案:
1. 不把 callback 生成的 tool status 当作历史 UI。
    它只适合显示“正在调用工具”的实时过程。
2. 刷新后从 checkpointed AIMessage.tool_calls 和 ToolMessage 重建工具调用展示。
    也就是历史工具调用应该由持久消息渲染，而不是依赖 callback 状态。
3. 主聊天区渲染消息时增加 ToolMessage/AIMessage tool_calls 的处理：
    - AIMessage.tool_calls 展示“调用工具：xxx”和参数
    - 匹配后续 ToolMessage.tool_call_id 展示结果
    - 这样刷新后也能完整恢复
4. callback 的实时 status 可以保留，但只用于当前轮流式反馈。
    当前轮结束后，最终 UI 应以 result["messages"] 重新渲染的持久消息为准。


## TODO18
1. ruff 检查(OK，未记录)
2. 短期记忆定期合并的测试(OK，未记录)

## task18
分析问题:
rai_whoami 模块是 rai 针对 RAG 知识库设计的，但现在还未应用在 @/home/jazzy/rai/examples/rosbot-xl-memory-demo.py。请你查看他的实现，你觉得 rai_whoami 实现得如何？如果要加到该 demo 中，实施方案是什么？

RAG 主要问题:
1. RAG 检索结果格式太粗糙
    QueryDatabaseTool._run() 直接返回 str(self.vdb_client.similarity_search(...))。这会把 Document(...) 的 repr 扔给 LLM，缺少清晰的 source、score、片段格式。demo 中最好封装成更干净的输出。
2. 构建向量库时没有显式 chunking
    src/rai_whoami/rai_whoami/vector_db/faiss.py:42 直接 FAISS.from_documents(data.documentation, ...)。如果 PDF/MD 文档很长，检索粒度会很差。更标准的是先用 text splitter 切 chunk。
3. 加载 FAISS 使用 allow_dangerous_deserialization=True
    src/rai_whoami/rai_whoami/vector_db/faiss.py:66。本地可信文件可以接受，但不应加载不可信来源的 vector store。
4. Embodiment 生成依赖 LLM，一次性拼接全部文档
    src/rai_whoami/rai_whoami/processors/preprocessors/docs.py:31 把所有文档 page content 拼到一个 prompt。文档稍大就容易超 context，也缺少分阶段抽取/合并。
5. 测试覆盖不足
    我没有看到 rai_whoami 专门的测试目录。现在如果要接到 demo，建议先补最小测试，避免路径、FAISS 文件缺失、tool 输出格式这些问题运行时才暴露。

建议方案:
1. 先用 EmbodimentInfo.from_file 替换手写 JSON 解析。
2. 增加 optional query_robot_docs tool。
3. 包装/优化 QueryDatabaseTool 输出格式。
4. 补测试。
5. 后续再考虑把 vector DB 构建和路径配置放进 config.toml。

疑问:
1. 用 EmbodimentInfo.from_file 替换手写 JSON 解析，EmbodimentInfo 的来源是什么？这样会不会增加复杂程度，因为 embodiment_info 其实到底是手写的。
2. query_robot_docs 问题:
```txt
我前面说的 query_robot_docs 不是要重新造一个检索系统，而是建议在 demo 里包装/重命名这个已有工具，让它更适合 agent 使用。

原因是现在 QueryDatabaseTool 的默认定义比较泛：
name = "query_database"
description = "Query the database with a natural language query"

这对 agent 来说语义不够明确。它不知道这个 database 是什么，什么时候该用，和 memory database 有什么区别。

在 memory demo 里已经有长期记忆工具：
save_fact
save_location
forget_memory

如果再加一个叫 query_database 的工具，agent 可能会混淆：
- 这是查用户记忆？
- 查机器人说明书？
- 查位置？
- 查所有数据库？

所以我的建议是：底层仍然用 rai_whoami.tools.QueryDatabaseTool，但在 demo 里把它配置成更明确的工具名和描述，例如：
from rai_whoami.tools import QueryDatabaseTool

query_robot_docs = QueryDatabaseTool(
    root_dir=str(whoami_root),
    embeddings_model=embeddings,
    k=4,
)

query_robot_docs.name = "query_robot_docs"
query_robot_docs.description = (
    "Search the robot's static whoami documentation, including hardware specs, "
    "sensors, capabilities, URDF/documentation details, and operating limits. "
    "Use this for robot documentation questions, not for user preferences or "
    "long-term conversation memory."
)
```
3. rai_whoami 用的是什么文档加载器，功能如何？
rai_whoami 用的是 LangChain Community 的文档加载器

根据你之前的思路，完成 rai_whomi 集成的实际方案:
1. embodiment_info 继续使用原有的手写 JSON 解析
2. 增加 optional query_robot_docs tool。
3. 包装/优化 QueryDatabaseTool 输出格式。
4. 补测试。
5. 把 vector DB 构建和路径配置放进 config.toml。


## ques18-1
rai_whoami 用的是什么文档加载器，功能如何？
    rai_whoami 用的是 LangChain Community 的文档加载器

不足:
1. 没有 chunking(缺乏文本分块的功能)
现在是 loader 加载出什么文档，就直接用于 FAISS。长 PDF、长 MD 检索粒度会偏粗，召回质量可能差。

2. Markdown 只是普通文本
.md 用 TextLoader，不会理解标题层级、代码块、表格结构。

3. URDF/Xacro 只是纯文本
没有解析 robot links/joints/sensors/transmissions 等结构化信息。LLM 可以读，但检索和抽取都不够稳。

4. PDF 质量取决于 PyPDFLoader
扫描版 PDF、复杂表格、多栏布局效果可能不好。

5. 没有 metadata 强化
检索结果没有很好保留/格式化 source、页码、章节等信息，后续给 agent 用时可解释性一般。

按照你之前的思路，完成完善 rai_whomi 的方案:
1. 构建 FAISS 前加 RecursiveCharacterTextSplitter
2. 查询工具输出时格式化 source/page/content，不要直接返回 Document(...) 的字符串表示
3. 使用 MarkdownHeaderTextSplitter+RecursiveCharacterTextSplitter 应对 Markdown

NOTE: 在 task18 前完成


## task19
请跑通 /home/jazzy/rai/examples/rosbot-xl-memory-demo.py 的 rag 功能，rag 数据库可以先用你伪造的，重要的是功能本身的正确性。


## task20
我发现如果在项目文件夹下不新建 data/ 目录，sqlite 就会产生报错？

@/home/jazzy/rai/examples/rosbot-xl-memory-demo.py 何时会去查询rag,是固定触发还是agent自己决定？
- RAG 是可用工具，由 agent 自主触发；当前没有每轮固定 retrieval。

我使用 @/home/jazzy/rai/examples/robotxl_whomi/documentation/man.md 和当前 config.yaml 验证 rag,但是显示"  File "/home/jazzy/
rai/src/rai_whoami/rai_whoami/vector_db/faiss.py", line 109, in _build
    raise ValueError("No documents found")
ValueError: No documents found",什么原因?

```
simple_model = "gpt-5.4"
complex_model = "gpt-5.4"
embeddings_model = "gpt-5.4"
base_url = "https://api.sudocode.chat/v1"
```

## task21
@/home/jazzy/rai/examples/rosbot-xl-memory-demo.py 这个demo里有哪些功能是可以复用的,以抽象出来作为通用工具.先不要修改代码,先分析
1. Whoami/RAG 工具创建

    现在这些逻辑都在 demo 里：
    - RobotDocsConfig
    - RobotDocsQueryTool
    - load_robot_docs_config
    - _has_whoami_vector_db
    - _create_robot_docs_tool

    这些不应该属于 rosbot-xl-memory-demo.py。更适合放到 rai_whoami，例如：

    from rai_whoami.tools import create_robot_docs_tool
    from rai_whoami.config import load_whoami_config

    理想 API：

    tool = create_robot_docs_tool(
        root_dir="examples/robotxl_whomi",
        embeddings_model=embeddings,
        build_vector_db=True,
        k=4,
        name="query_robot_docs",
    )

    这块复用价值最高，因为任何 agent 都可能需要“静态机器人文档 RAG”。

2. Whoami 向量库检测/构建

    当前 demo 自己判断：

    generated/index.faiss
    generated/index.pkl
    generated/vdb_kwargs.json

    这应该成为 rai_whoami 的能力，例如：

    has_vector_db(root_dir)
    ensure_vector_db(root_dir, embeddings_model=None)

    这样 demo 不需要知道 FAISS 文件细节。

3. Memory-aware agent factory

    build_memory_agent() 现在做了很多通用事：
    - 创建 LLM
    - 创建 memory tools
    - 注入 long-term memory
    - 创建 create_memory_react_agent
    - 可选加入 RAG tools

    可以抽象成类似：

    create_persistent_memory_agent(
        memory_mgr=memory_mgr,
        llm=llm,
        tools=[...],
        system_prompt_builder=...,
        namespace="default",
        user_id="default",
    )

    或更具体一点：

    create_memory_agent_with_tools(
        memory_mgr,
        llm,
        base_tools,
        system_prompt_builder,
        user_id,
        namespace,
    )

    这样 demo 只负责提供 ROSBot 的 embodiment/prompt，而不是拼装 memory graph。

4. System prompt section builder

    当前 prompt 里混了四类内容：
    - embodiment
    - long-term memory
    - memory tool 使用规则
    - robot docs RAG 使用规则

    可以抽象成小工具：

    render_memory_system_prompt(
        embodiment=...,
        long_term_memory=...,
        enable_robot_docs=True,
    )

    或更灵活：

    PromptSectionBuilder()
        .add_embodiment(...)
        .add_long_term_memory(...)
        .add_memory_tool_instructions()
        .add_robot_docs_instructions()

    这样不同 demo 可以共享规则文本，减少 prompt 漂移。

已经部分抽象，但还可以继续收口

5. Streamlit memory UI

    现在 render_memory_sidebar() 和 render_chat_messages_with_tools() 已经在 rai.frontend.memory_streamlit 里，这是好的。

    demo 里仍有一些可复用 UI 逻辑：
    - sidebar 里显示 tool calls
    - st.chat_input() 后构造 HumanMessage
    - 调用 streamlit_invoke
    - 把 checkpointed result 写回 st.session_state

    可以抽成：

    run_memory_chat_turn(
        graph,
        sidebar_state,
        context,
    )

    或：

    render_memory_chat_app(...)

    不过这个优先级低于 RAG/tool factory，因为 Streamlit app 通常会有定制需求。

6. MemoryManager 初始化

    initialize_memory_mgr() 里有通用逻辑：
    - 读取 memory config
    - 尝试加载 embeddings
    - embeddings 失败时降级
    - start() / setup()

    但它现在混了 st.error / st.warning / st.stop()，所以不能直接复用到非 Streamlit 环境。

    可以拆成纯函数：

    create_memory_manager_from_config(config, embeddings=None)

    Streamlit 层只负责展示错误。

不建议抽象，保留在 demo 里

- EMBEDDING_PATH
- _welcome_message()
- ROSBot XL 的具体 system prompt 语气
- “You are a ROSBot XL robot assistant...” 这类角色设定
- demo 的 Streamlit title/page icon
- 是否加入 WaitForSecondsTool()

这些是 demo/机器人特定逻辑，抽象后反而会让通用层带上 ROSBot 假设。

推荐抽象顺序

1. 先把 RAG 相关移到 rai_whoami：
    - WhoamiConfig
    - load_whoami_config
    - has_vector_db
    - ensure_vector_db
    - create_robot_docs_tool

2. 再把 memory agent 拼装逻辑移到 rai.memory：
    - 通用 memory tools 创建
    - long-term memory prompt injection
    - optional extra tools

3. 最后再考虑 Streamlit chat loop 抽象。


## todo22
连续工具调用失败的处理
已合并到 task23 完成


## task23
分析出错过程(该过程两次复现):
1. 我和 agent 说: "你好，你肥吗？"
2. agent 调用  query_robot_docs，tool 返回:
"Result 1
Source: examples/rosbotxl_whoami/documentation/man.md
Page: unknown
Content:
rosbot-xl 参数
- 相机参数
相机大小为 114514 * 1919"
3. agent 不再输出，像卡住一样

完成你的思路:
做通用 guard，但默认策略保守，只限制明显异常的情况。然后给 query_robot_docs、memory tools 这种容易重复调用的工具加更严格 policy。这样既解决循环卡死，也不会破坏正常的多工具任务。


## task24
在 @/home/jazzy/rai/examples/rosbot-xl-memory-demo.py 的基础上，将 @/home/jazzy/rai/examples/rosbot-xl-demo.py 的工具添加进来(除了 rai_perception 相关)。需要修改哪些内容？


## task25
openai 接口修改

按照你的思路，实现实际方案:
```yaml
[openai]
base_url = "http://localhost:8000/v1"
api_key = "EMPTY"
model = "default-chat-model"

[openai.vlm]
base_url = "http://10.0.88.103:11451"
api_key = "EMPTY"
model = "qwen27b-mtp"

[openai.embeddings]
base_url = "http://10.0.88.103:11452"
api_key = "EMPTY"
model = "nomic"
```
并修改 @/home/jazzy/rai/src/rai_core/rai/frontend/configurator.py，使其和新配置保持一致


## task26
我使用 configurator.py 尝试，模型配置为:
```
[vendor]
simple_model = "openai.vlm"
complex_model = "openai.vlm"
embeddings_model = "openai.embeddings"
```
，但测试配置时出现:
```
step_renderer[st.session_state.current_step]()
File "/home/jazzy/rai/src/rai_core/rai/frontend/configurator.py", line 1032, in review_and_save
test_results[name] = test()
File "/home/jazzy/rai/src/rai_core/rai/frontend/configurator.py", line 987, in test_recording_device
index = [device["name"] for device in devices].index(device_name)
ValueError: 'default' is not in list
```


## task27
错误出现在: 我运行" uv run streamlit run examples/rosbot-xl-memory-demo.py "页面,就产生 openai_api_key 的错误

根据你的思路，完成该方案:
1. OpenAIEmbeddings 初始化继续使用 endpoint 自己的 api_key。
2. return_kwargs=True 时也返回同一个 api_key。
3. 只对 vendor.startswith("openai") 的 embeddings kwargs 增加 api_key。
4. 如果 api_key 为空，则可以不写入，继续让 SDK 按环境变量兜底。
5. 测试补一条：openai.embeddings endpoint 在没有 OPENAI_API_KEY 环境变量时，通过 return_kwargs=True 返回的 kwargs 可以重新构造 OpenAIEmbeddings。


## task28
@/home/jazzy/rai/examples/rosbot-xl-memory-demo.py @/home/jazzy/rai/examples/rosbotxl_whoami/documentation/man.md 现在我是用 robot-xl-memory-demo 测试 RAG 检索能力,但是文档太简单了.请建立更详细复杂的文档,并测试当前demo的 RAG 检索能力.


结果:
我在 examples/rosbotxl_whoami/documentation/man.md 建了更完整的 ROSBot XL 文档，内容包括：

- 机器人尺寸、重量、电池、续航
- RGB-D 相机、雷达、IMU、里程计
- Nav2 导航坐标系与目标点配置
- ROS 2 topics / frames / actions
- memory demo 可用工具说明
- 运行限制、安全规则、故障排查
- 常见问题速答

同时重建了本地 RAG 向量库：

examples/rosbotxl_whoami/generated/index.faiss
examples/rosbotxl_whoami/generated/index.pkl
examples/rosbotxl_whoami/generated/vdb_kwargs.json

构建结果：

Document chars: 6074
Chunks: 39
Max chunk chars: 238
Index size: 39

注意：examples/rosbotxl_whoami/documentation/man.md 和 generated/ 当前被 .gitignore 忽略，所以不会出现在 git status 里。现在 git
status 只显示 config.toml 有已有修改。

测试结论：

- “尺寸/重量”检索可用，能返回 435 mm、330 mm、310 mm、约 5.2 kg。
- “相机 topic/分辨率/帧率”检索可用，能返回 /camera/camera/color/image_raw、1280 x 720、30 FPS。
- “导航坐标系 / navigate_to_pose”检索基本可用，正确结果在前几名。
- “当前位置应该查文档还是实时工具”可用，能返回 get_ros2_robot_position / get_current_pose。
- “玻璃门/反光表面”自然语言 query 排序仍不稳定；关键词式 query 能检到正确段落，但普通口语问法容易被错误章节挤到前面。

这个测试说明：当前 demo 的 RAG 链路本身能工作，Markdown 分块和 FAISS 索引也能正常构建；主要弱点是当前 nomic embedding 对中文自然
问句的排序不够稳定，后续如果要更可靠，建议加 hybrid/BM25 检索或 reranker。


## task29
这是我本地部署的大模型:
```
model_id: str = "qwen27b-mtp",
base_url: str = "http://10.0.20.16:11451/v1/",
api_key: str = "empty",
```
我想测试其视觉检测能力，数据集在 /home/jazzy/study/Qwen3-VL/demo/测试数据，分别是安全穿戴和跑冒滴漏测试。请在 demo/ 建立测试脚本，测试其视觉检测能力，并输出检测结果、检测事件等到某一文件。完成后先测试前面几张，以验证测试脚本是否正确。


## task30
当前 rai 集成的 tools/skill 有哪些，我想基于 @/home/jazzy/task_ws/src/gimbal_stub_hub 添加一个"简单云台拍照"工具，该工具会进行云台归中，然后拍照。
可以在 gimbal_stub_hub 添加一个新的消息接口(该消息类似来自 ros2 官方)，完成所述功能。
你的实现思路如何，这个 skill 是否符合规范？先讨论，不要修改源码.

实现方案:
1. 先在 gimbal_stub_hub 增加一个“归中后拍照”的 action。
2. 再在 RAI 里加一个对应的 BaseROS2Tool 包装。
3. 最后在 agent prompt 里把它描述成“简单云台拍照工具”，让 agent 直接调用。
该方案中“归中后拍照”的 action 能用官方 ros2 消息吗？先讨论，不要修改源码.

问题:
1. 我的意思是，"归中后拍照" agent 不管拍照模块具体如何实现，而只管外部暴露的接口.
2. agent 至少要从这次拍照 action 中获取图片路径/图片数据，action 用什么比较好

问题:
1. 如果新增一个 action，比如 CaptureCenteredPhoto.action，会破坏 rai 原先的依赖吗？是否在 rai 基础上新建一个包，类似 @/home/jazzy/rai/examples/rosbot-xl-memory-demo.py(但和 demo 不同，是新建一个单独的包)，对 rai 本身的修改只涉及 agent 本身的基础功能.这样设计会不会更合理些，客观分析，不要有主观偏向.

2. 你理解得不对，可能是我没有表述清楚. 我的意思是这个新 demo 是针对巡检设计的，其依赖 rai 和新建 CaptureCenteredPhoto.action（这个 action 可以放在 inspection_interfaces 下），这个 demo 本身就算一个 uv 项目. 这样设计会不会更合理些，客观分析，不要有主观偏向.


## task31
rai_inspection_agent 下 CenterGimbalAndCaptureTool 这个 tool，只会在页面显示图片地址; 但应该类似 rai/GetROS2ImageConfiguredTool，其能够直接在页面显示出拍摄的图片。评估一下实施可能性和合理性，先讨论，不要修改源码

完成你提供的方案:
CenterGimbalAndCaptureTool 目前是在巡检 agent 项目里自己封装的，不是 RAI 核心通用工具。
所以最合适的做法不是把它做成纯路径输出，而是让它：
1. 继续返回路径字符串用于审计/日志
2. 同时附带 artifact.images
3. 如果有多张图，就都放进 artifact


## task32
查看 rai_inspection_agent 的提示词工程(具体提示词部分)，我感觉实现有些问题(冗余)，请进行评估，先讨论，不要修改源码

完成你觉得更合理的方向是：
- base prompt 只保留角色、任务目标、embodiment
- 单独一个“硬规则”段，只放歧义消解，例如：
    - 用户说“拍巡检照片” -> center_gimbal_and_capture
    - 用户说“播放检测到气体泄漏” -> 对应喇叭/告警工具

- 单独一个“限制”段，只说明什么不能做、什么没有
- 不再在多个 section 里重复“你能导航/你能看图”这种基础能力


你这块改得有些问题,先不要改. 先改新问题: 我使用 CenterGimbalAndCaptureTool 工具发现问题: 图像显示了一会,就直接消失.

修法:
在回放逻辑里支持渲染 HumanMultimodalMessage.images. 如果这个修复是通用修复，请使用通用修复方式(rai中修复); 如果是 rai_inspection_agent 的问题，请单独修复


完成你建议的修复方向，并测试通过：
- 在回放逻辑里补一个 HumanMultimodalMessage 分支
- 直接渲染 msg.content 之外的 msg.images
- 最好抽一个通用 helper，给两个 Streamlit 入口共用


你刚才添加的 multimodal 修复有问题，页面显示过往图片会使用:
```
Image returned by a tool call 8yfwPIfXMe5VC2dbDAdMR2vgHpvncOc7
```
这样看起来很割裂。工具调用获取的图片应该在工具显示栏，如: "Tool: center_gimbal_and_capture".
请进行评估，先讨论，不要修改源码


完成你建议的方案，并测试通过：
- 回放层识别 HumanMultimodalMessage
- 不直接渲染成 user message
- 改为把它和对应的 ToolMessage 绑定，放进 Tool: ... 的 expander 里显示图片


## task33
我在 cat@10.0.40.137(密码: cat) 上运行 
```
uv run streamlit run rai_inspection_agent/app.py
```
rai/rai_inspection_agent 均在 ~ 下;
tool 调用的图像不显示，请分析原因，先讨论，不要修改源码


这次 tool result 的 image_uri 在 cat 主机上确实真实存在，直接在 cat@10.0.40.137(密码: cat) 修复 rai 的这个问题，并测试通过，可以使用 uv run streamlit run rai_inspection_agent/app.py 查看你的修复结果


Sorry，我让你访问的主机错误了，应该是 cat@10.0.40.85(密码: cat)，137 主机上不是最新提交，请重新修复


## task34
你这个巡检场景里，center_gimbal_and_capture 的图通常既有展示价值，也可能有后续判断价值，所以现在这种做法能工作，但如果图片较大或频繁拍照，后面最好拆成两层：
- 主上下文只保摘要或引用
- 原图放外部 artifact 存储

问题:
1. "artifact 存储"指什么，他和现在 checkpoints.db、store.db 的关系是什么？
2. 但我在远程 cat@10.0.40.85(密码: cat) 并没有看到 artifact_database.pkl，但当前页面确实能渲染"云台归中拍照"的图片，请为我指明他的远程路径
3. 那么 artifact_database.pkl 体现在当前项目的哪里？
- artifact_database.pkl 更像是一个旧的/旁路的附件缓存实现，不是现在巡检页面的主存储。

完成你所说的更合理的分层，直接在远程 cat@10.0.40.85(密码: cat) 实现并测试通过：
- 上下文里只保留“模型需要理解的最小图像信息”
- 原图放到独立 artifact 存储
- checkpoint 里只存引用或摘要

完成建议放在 data/artifacts/ 目录，而不是单个 pkl 文件，直接在远程 cat@10.0.40.85(密码: cat) 实现并测试通过：
- 每个 tool_call_id 一个目录
- 原图单独文件
- 元数据单独索引文件

问题:
1. 你改了之后，agent交互中显示:
```
我无法直接"看到"或分析图像内容。虽然拍摄成功并保存了图像到 /home/cat/Workspace/task_ws/captured_images/req_20260626_140445_011/capture_raw_iter1.jpg，但我没有图像处理或计算机视觉工具来分析这张照片中的具体场景、物体或环境细节。
```
现在 agent 无法获取图像数据了吗？那放在 artifacts 中有什么意义？告诉我现在的链路。

2. 当前最新的实现存在疑问: 
为了获取图像数据，是做成两个工具，那么为什么不只使用 center_gimbal_and_capture 工具完成？你单独拆出一个新工具的目的是什么？

3. 我希望能在 rai 的 @/home/jazzy/rai/examples/rosbot-xl-memory-demo.py 页面中输入对话框的左边加入一个加号按钮，其可以上传图片。说明：
- 图片不会存储在 checkpoint 中，但 agent 可以分析当前上传的图片内容。
- 做成通用的方式，不只是针对 xl-mermory-demo，还要能够复用到 inspection_rai_agent 等其他项目和模块  
评估实施方案，先讨论，不要修改源码

问题3的实施方案：
- 新增通用 helper，例如 rai.frontend.chat_input：
    - render_multimodal_chat_input(...)
    - 返回 text、images_base64、uploaded_file_names
    - 内部用 st.chat_input(accept_file=...)

- 改 rai.frontend.memory_streamlit.render_memory_chat_input()：
    - 使用新 helper
    - checkpoint 输入仍是 HumanMessage
    - 临时图片通过 MemoryAgentContext 或 RunnableConfig.configurable 传入 graph

- 改 rai.memory.graph.MemoryAgentContext：
    - 增加可选 transient_images
    - run_react() 中只对 inner agent 注入 HumanMultimodalMessage

- 改 rai.frontend.streamlit.run_streamlit_app()：
    - 同样用新 helper
    - session history 保存文本
    - 当前 invoke 临时使用 multimodal 最新消息

- rosbot-xl-memory-demo.py 和 inspection_rai_agent：
    - 理想情况下无需业务代码感知图片上传
    - 它们继续调用 render_memory_chat_input(...)
    - 自动获得上传图片能力

5. 当前 streamlit 页面确实可以输入图片，但存在以下问题:
- 在输入框里图片无法放大预览看
- 图片输入给 agent 后，页面里只有文字如: "请查看这张图片"，但具体图片内容无法看到  
评估实施方案，先讨论，不要修改源码


问题5实施方案：
- 默认：session_state 保存上传图片，满足当前页面显示和放大。
- 可配置：启用 artifact backend 后，把上传图片保存到 data/artifacts/user_uploads/...，用于可回放审计。
1. 在 rai.frontend.chat_input 的 ChatInputSubmission 增加附件元数据：
    - images
    - file_names
    - mime_types
    - attachment_id
2. 在 memory_streamlit.render_memory_chat_input() 和普通 run_streamlit_app() 中：
    - checkpoint 仍只保存 HumanMessage(text)
    - 当前 invoke 仍临时传 HumanMultimodalMessage
    - 同时把图片保存到 UI attachment store
3. 在 render_chat_messages_with_tools() / run_streamlit_app() 回放用户消息时：
    - 渲染文字
    - 如果该 turn 有上传图片附件，显示缩略图
    - 用 st.expander 或 st.dialog 提供大图预览
4. 测试重点：
    - checkpoint 中没有 base64 图片
    - LLM 当前调用能收到 HumanMultimodalMessage.images
    - 页面渲染用户消息时能拿到附件并调用 st.image
    - rerun 后 session_state 中的附件仍能显示


问题:
1. 你修改之后，我在 rai_inspection_agent 上传图片直接显示"很抱歉，我目前无法分析这张图片。"并且也无法在输入时放大预览图片

