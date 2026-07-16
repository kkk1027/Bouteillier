"""
agent_graph.py
使用 LangGraph 接入 DeepSeek API，让 AI 能够调用你的导航工具
"""

import os
from typing import Literal

from langchain_openai import ChatOpenAI
from langchain_core.messages import SystemMessage, HumanMessage
from langgraph.graph import StateGraph, MessagesState, START, END
from langgraph.prebuilt import ToolNode, tools_condition
from langgraph.checkpoint.memory import InMemorySaver

# 导入你之前定义的工具
from autopatrol_robot.tools import ALL_TOOLS, cleanup_ros


# ============================================================
# 第一步：配置 DeepSeek API
# ============================================================
DEEPSEEK_API_KEY = os.environ.get("DEEPSEEK_API_KEY", "")
DEEPSEEK_BASE_URL = "https://api.deepseek.com/v1"

if not DEEPSEEK_API_KEY:
    print("⚠️  环境变量 DEEPSEEK_API_KEY 未设置，Agent 将无法调用 DeepSeek API")

# 初始化模型（DeepSeek 兼容 OpenAI 接口）
llm = ChatOpenAI(
    base_url=DEEPSEEK_BASE_URL,
    api_key=DEEPSEEK_API_KEY,
    model="deepseek-chat",  # 使用 DeepSeek V4
    temperature=0.3,        # 较低温度让决策更准确
)


# ============================================================
# 第二步：绑定工具到模型
# ============================================================
llm_with_tools = llm.bind_tools(ALL_TOOLS)


# ============================================================
# 第三步：定义系统提示词（告诉 AI 它是什么角色）
# ============================================================
SYSTEM_PROMPT = """你是一个 ROS2 机器人控制助手。你可以通过调用工具来控制机器人。

可用的工具：

【地点查询】
- list_locations: 查看所有可用地点（预定义 + 自定义）
- list_saved_locations: 查看自定义保存的地点

【自定义地点管理】
- save_current_location: 保存当前位置为自定义地点（当用户说"这里是XXX"时）
- delete_saved_location: 删除指定的自定义地点
- clear_all_saved_locations: 清空所有自定义地点

【导航工具】
- navigate_to_location: 导航到预定义地点（起点/厨房/客厅/影音房）
- navigate_sequence: 依次导航到多个地点（当用户说"先去A再去B"、"从A到B再到C"时）
- navigate_to_saved_location: 导航到保存的自定义地点（先去自定义找，找不到回退预定义）
- navigate_to_xy: 导航到任意坐标
- cancel_navigation: 取消当前导航
- return_to_start: 返回起点（不触发舵机播报）
- get_navigation_status: 查询导航状态

【系统状态】
- initialize_pose: 初始化AMCL定位（地图加载后、首次导航前调用，告诉机器人"你现在在起点"）
- get_current_position: 获取当前位置
- get_battery_status: 查询电量
- list_predefined_points: 列出预定义航点坐标

【看护模式/跟随模式】
- start_patrol_mode: 开启看护模式（当用户说"开启看护模式"、"启动跟随"时）
- stop_patrol_mode: 关闭看护模式（当用户说"关闭看护模式"、"停止跟随"时）
- get_patrol_mode_status: 查询看护模式运行状态

规则：
1. 当用户说"这里是XXX"时，调用 save_current_location
2. 当用户说"删除XXX"时，调用 delete_saved_location
3. 当用户说"清空所有地点"时，调用 clear_all_saved_locations
4. 当用户说"去XXX"时，先调用 navigate_to_saved_location（它会自动回退到预定义地点）
5. 当用户说"先去A再去B"、"依次去A和B"等连续多个地点时，调用 navigate_sequence
6. 当用户问"有哪些地点"时，调用 list_locations
7. 当用户说"开启看护模式"时，调用 start_patrol_mode
8. 当用户说"关闭看护模式"时，调用 stop_patrol_mode
9. 当用户问"到了吗"、"导航状态"时，调用 get_navigation_status
10. 回复要简洁友好

【重要】导航工具已改为非阻塞：
- 当你调用导航工具后，工具会立即返回 "📍 正在前往..." 类消息
- 你必须立即将此消息告知用户，不要等待（导航已在后台执行）
- 后续到达/返回通知会由系统自动推送给用户
- 如下使用流程：
  用户: "去厨房"  →  你调用 navigate_to_saved_location("厨房")  →  工具返回 "📍 正在前往厨房..."
  →  你回复: "好的，正在前往厨房！"  ← 到此结束，不要等待"""

def agent_node(state: MessagesState):
    """Agent 决策节点：理解用户意图，决定调用哪个工具"""
    messages = state["messages"]

    print(f"\n[AgentNode] === 进入 agent_node ===")
    print(f"[AgentNode] 收到 {len(messages)} 条消息:")
    for i, m in enumerate(messages):
        mtype = type(m).__name__
        has_tc = hasattr(m, 'tool_calls') and m.tool_calls
        tc_names = [tc.get('name') for tc in m.tool_calls] if has_tc else None
        content_preview = str(getattr(m, 'content', ''))[:80]
        print(f"[AgentNode]   [{i}] {mtype} tc={tc_names} content={content_preview!r}")
    
    # 添加系统提示（只加一次）
    if not any(isinstance(m, SystemMessage) for m in messages):
        messages = [SystemMessage(content=SYSTEM_PROMPT)] + messages
    
    print(f"[AgentNode] 调用 DeepSeek API (消息数={len(messages)})...")
    response = llm_with_tools.invoke(messages)

    # 打印 LLM 返回的内容
    print(f"[AgentNode] DeepSeek 返回: type={type(response).__name__}")
    if hasattr(response, 'tool_calls') and response.tool_calls:
        print(f"[AgentNode] tool_calls: {[tc.get('name') for tc in response.tool_calls]}")
    if hasattr(response, 'content'):
        print(f"[AgentNode] content: {str(response.content)[:200]!r}")

    return {"messages": [response]}


# ============================================================
# 第四步：构建状态图
# ============================================================
def build_agent():
    """构建并编译 Agent"""
    
    # 创建工具执行节点
    tool_node = ToolNode(tools=ALL_TOOLS)
    
    # 构建图
    graph_builder = StateGraph(MessagesState)
    graph_builder.add_node("agent", agent_node)
    graph_builder.add_node("tools", tool_node)
    
    # 添加边
    graph_builder.add_edge(START, "agent")
    graph_builder.add_conditional_edges(
        "agent",
        tools_condition,  # 如果有工具调用请求，去 tools 节点；否则结束
        {"tools": "tools", END: END}
    )
    graph_builder.add_edge("tools", "agent")  # 工具执行完回到 agent
    
    # 添加记忆（支持多轮对话）
    memory = InMemorySaver()
    
    return graph_builder.compile(checkpointer=memory)


# 创建全局 agent 实例
agent = build_agent()


# ============================================================
# 第五步：交互式测试入口
# ============================================================
if __name__ == "__main__":
    print("=" * 50)
    print("🤖 ROS2 机器人 Agent 已启动")
    print("输入指令，输入 'quit' 退出")
    print("=" * 50)
    print("\n可用命令示例：")
    print("  • 开启看护模式")
    print("  • 关闭看护模式")
    print("  • 看护模式开了吗")
    print("  • 去厨房")
    print("  • 这里是厕所")
    print("  • 去厕所")
    print("=" * 50)
    
    # 会话配置（用于记住对话历史）
    config = {"configurable": {"thread_id": "test-session"}}
    
    while True:
        user_input = input("\n👤 你: ")
        if user_input.lower() in ['quit', 'exit', 'q']:
            print("再见！")
            break
        
        try:
            # 调用 Agent
            result = agent.invoke(
                {"messages": [HumanMessage(content=user_input)]},
                config=config
            )
            
            # 获取最终回复
            response = result["messages"][-1].content
            print(f"🤖 Agent: {response}")
            
        except Exception as e:
            print(f"❌ 错误: {e}")
    
    # 清理 ROS2 资源（包括看护模式进程）
    print("\n正在清理资源...")
    cleanup_ros()