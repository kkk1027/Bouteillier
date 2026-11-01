"""
agent_graph.py
使用 LangGraph 接入 DeepSeek API，让 AI 能够调用你的导航工具
"""

import os
import random
from typing import Literal

import httpx
from langchain_openai import ChatOpenAI
from langchain_core.messages import SystemMessage, HumanMessage, AIMessage
from langgraph.graph import StateGraph, MessagesState, START, END
from langgraph.prebuilt import ToolNode, tools_condition
from langgraph.checkpoint.memory import InMemorySaver

# 导入你之前定义的工具
from autopatrol_robot.tools import (
    ALL_TOOLS, LOCAL_TOOLS, get_tools_for, cleanup_ros,
)
from autopatrol_robot.persona import load_persona


# ============================================================
# 第一步：配置 DeepSeek API
# ============================================================
DEEPSEEK_BASE_URL = os.environ.get("DEEPSEEK_BASE_URL",
                                   "https://api.deepseek.com/v1")

# 占位值：新版 openai SDK(3.x) 在 api_key 为空时会直接抛
#   openai.OpenAIError: Missing credentials
# 导致整个 web_controller 起不来。给个占位值让服务能启动，
# 真正发起对话时由 web_controller 提前拦下并给明确提示。
_PLACEHOLDER_KEY = "sk-placeholder-no-ai"


def _candidate_key_files():
    """按优先级列出可能存有 API Key 的文件。

    只依赖环境变量是很脆弱的：直接 `python3 web_controller.py` 启动时
    根本不会读 .bashrc，于是程序静默落到占位值，用户看到的却是
    「401 Authentication Fails, Your api key: ****o-ai is invalid」——
    这个 `o-ai` 正是 placeholder 的尾巴，极具误导性（本项目实际踩过）。
    所以这里额外支持几处「落到磁盘」的配置。
    """
    here = os.path.dirname(os.path.abspath(__file__))       # .../autopatrol_robot
    pkg_root = os.path.dirname(here)                        # .../src/autopatrol_robot
    ws_root = os.path.dirname(os.path.dirname(pkg_root))    # .../fishbot_ws
    home = os.path.expanduser("~")
    return [
        os.path.join(pkg_root, ".env"),
        os.path.join(ws_root, ".env"),
        os.path.join(ws_root, "deepseek.env"),
        os.path.join(home, ".config", "fishbot", "deepseek.env"),
        os.path.join(home, ".fishbot_deepseek_env"),
    ]


def _parse_env_file(path):
    """从 .env 风格文件里读 DEEPSEEK_API_KEY，支持 export 前缀与引号。"""
    try:
        with open(path, "r", encoding="utf-8") as fp:
            for raw in fp:
                line = raw.strip()
                if not line or line.startswith("#"):
                    continue
                if line.startswith("export "):
                    line = line[len("export "):].strip()
                if not line.startswith("DEEPSEEK_API_KEY"):
                    continue
                _, _, value = line.partition("=")
                value = value.strip().strip('"').strip("'")
                if value:
                    return value
    except OSError:
        pass
    return ""


def _load_api_key():
    """按优先级加载 Key，返回 (key, 来源描述)。"""
    env_key = os.environ.get("DEEPSEEK_API_KEY", "").strip()
    # 忽略 run_web_controller.sh 以前注入的占位值，否则会把它当真 Key
    if env_key and not env_key.startswith("sk-placeholder"):
        return env_key, "环境变量 DEEPSEEK_API_KEY"

    for path in _candidate_key_files():
        file_key = _parse_env_file(path)
        if file_key and not file_key.startswith("sk-placeholder"):
            return file_key, "配置文件 %s" % path

    return "", ""


DEEPSEEK_API_KEY, DEEPSEEK_KEY_SOURCE = _load_api_key()

# ============================================================
# 模型来源选择：本地 Ollama / 远程 DeepSeek
# ------------------------------------------------------------
# FISHBOT_LLM 取值：
#   local    —— 只用本地 Ollama（推荐，零 API 费用）
#   deepseek —— 只用远程 DeepSeek API
#   auto     —— 默认。先探测本地 Ollama 是否在跑，在跑就用本地，
#               否则回退到 DeepSeek。
#
# 本地默认 qwen2.5:3b：实测 23 个工具的选择准确率 20/20，
# 首次加载约 90s（进显存），之后每次 0.4~0.7s。
# ============================================================
LOCAL_BASE_URL = os.environ.get("OLLAMA_BASE_URL",
                                "http://127.0.0.1:11434/v1")
LOCAL_MODEL = os.environ.get("FISHBOT_LOCAL_MODEL", "fishbot-qwen:3b")
LLM_MODE = os.environ.get("FISHBOT_LLM", "auto").strip().lower()

# 本地请求绝不能走代理：Ollama 在 127.0.0.1，走了代理会直接被拒。
# （系统里存在 http_proxy/socks 代理，因此这里显式 trust_env=False。）
for _k in ("NO_PROXY", "no_proxy"):
    _v = os.environ.get(_k, "")
    if "127.0.0.1" not in _v:
        os.environ[_k] = (",".join(filter(None, [_v, "127.0.0.1,localhost"])))

_llm_timeout = httpx.Timeout(60.0, connect=15.0)
_llm_http_client = httpx.Client(trust_env=False, timeout=_llm_timeout)
_llm_http_async_client = httpx.AsyncClient(trust_env=False, timeout=_llm_timeout)


def _probe_local_llm(timeout=1.5):
    """探测本地 Ollama 是否在线并且有模型。

    很短的超时：Ollama 没启动时连接会被立刻拒绝，不会拖慢启动；
    真的要等 1.5s 说明机器负载异常，那就当没装本地模型处理。
    """
    try:
        with httpx.Client(trust_env=False, timeout=timeout) as c:
            r = c.get(LOCAL_BASE_URL.rstrip("/") + "/models")
            if r.status_code != 200:
                return False
            try:
                names = [m.get("id", "") for m in r.json().get("data", [])]
            except Exception:  # noqa: BLE001
                return True   # 服务通了就算可用，模型名可能报不上来
            if not names:
                return False
            # 配的本地模型不在列表里也允许（Ollama 会自动拉同名 tag）
            return True
    except Exception:  # noqa: BLE001
        return False


def _decide_provider():
    if LLM_MODE == "local":
        return "local"
    if LLM_MODE == "deepseek":
        return "deepseek"
    # auto：本地优先
    return "local" if _probe_local_llm() else "deepseek"


LLM_PROVIDER = _decide_provider()
DEEPSEEK_KEY_MISSING = not bool(DEEPSEEK_API_KEY)

if LLM_PROVIDER == "local":
    # 本地模式不需要 API Key，占位值只是防止 SDK 报 Missing credentials
    if not DEEPSEEK_API_KEY:
        DEEPSEEK_API_KEY = _PLACEHOLDER_KEY
    LLM_MODEL = LOCAL_MODEL
    LLM_BASE_URL = LOCAL_BASE_URL
    print("=" * 62)
    print("🖥️  AI 模型来源：本地 Ollama")
    print("   模型   ：%s" % LOCAL_MODEL)
    print("   地址   ：%s" % LOCAL_BASE_URL)
    print("   费用   ：0（不调用云端 API）")
    print("   切换   ：export FISHBOT_LLM=deepseek 可改回云端")
    print("=" * 62)
elif DEEPSEEK_KEY_MISSING:
    DEEPSEEK_API_KEY = _PLACEHOLDER_KEY
    LLM_MODEL = "deepseek-chat"
    LLM_BASE_URL = DEEPSEEK_BASE_URL
    print("=" * 62)
    print("⚠️  未找到 DEEPSEEK_API_KEY，且本地 Ollama 不可用")
    print("   → AI 对话功能不可用；网页控制、导航、地图不受影响。")
    print()
    print("   方案 A（推荐，免费）：启动本地模型")
    print("     ~/.local/ollama/bin/ollama serve &")
    print("     ollama pull qwen2.5:3b")
    print()
    print("   方案 B：配置 DeepSeek API Key（任选其一）")
    print("     echo 'DEEPSEEK_API_KEY=sk-你的key' \\")
    print("          > ~/.config/fishbot/deepseek.env")
    print("     或在工作空间建 .env，或 export DEEPSEEK_API_KEY=...")
    print("=" * 62)
else:
    LLM_MODEL = "deepseek-chat"
    LLM_BASE_URL = DEEPSEEK_BASE_URL
    print("✅ AI 模型来源：DeepSeek 云 API（Key 来源：%s）" % DEEPSEEK_KEY_SOURCE)

# web_controller 用这个判断"AI 到底能不能用"，与具体厂商无关
AI_UNAVAILABLE = DEEPSEEK_KEY_MISSING and LLM_PROVIDER != "local"

# 初始化模型（统一走 OpenAI 兼容接口，本地/云端只是 base_url 不同）
llm = ChatOpenAI(
    base_url=LLM_BASE_URL,
    api_key=DEEPSEEK_API_KEY,
    model=LLM_MODEL,
    temperature=0.3,        # 较低温度让决策更准确
    http_client=_llm_http_client,
    http_async_client=_llm_http_async_client,
    # 置空 socket_options：否则 langchain-openai 会注入自定义 httpx transport，
    # 并在检测到代理环境变量时打印长篇 WARNING。我们已经自带 http_client，
    # 无需它再做任何 transport 层面的干预。
    http_socket_options=(),
)


# ============================================================
# 第二步：绑定工具到模型
# ------------------------------------------------------------
# 本地 3B 只能吃 18 个工具，给满 25 个会确定性选错（实测 12/14）。
# 详见 tools/__init__.py 里 LOCAL_TOOLS 的注释。
# ============================================================
ACTIVE_TOOLS = get_tools_for(LLM_PROVIDER)
llm_with_tools = llm.bind_tools(ACTIVE_TOOLS)

if LLM_PROVIDER == "local":
    print("🔧 本地模型工具数：%d（完整 %d）" % (
        len(ACTIVE_TOOLS), len(ALL_TOOLS)))


# ============================================================
# 第三步：定义系统提示词（告诉 AI 它是什么角色）
# ------------------------------------------------------------
# 身份设定统一由 persona.py 提供，本地/云端共用：
#   * 有 ~/.config/fishbot/persona.txt 就用它（优先）
#   * 否则云端用完整版人设，本地 3B 用精简版
# ============================================================
PERSONA_TEXT = load_persona(compact=(LLM_PROVIDER == "local"))

SYSTEM_PROMPT = PERSONA_TEXT + """

你可以通过调用工具来控制机器人。

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
- initialize_pose: **重置** AMCL 定位，强制认为车此刻在原点（破坏性操作，
  只在用户明确说"初始化位姿/初始化定位/重置定位"时才用；
  ⚠️ 用户说"回起点/回家"时**绝对不要**用它，那是 return_to_start 的事）
- get_current_position: 获取当前位置
- get_battery_status: 查询电量
- list_predefined_points: 列出预定义航点坐标

【雷达跟随】
- start_lidar_follow: 开启雷达跟随，小车跟着前方 40cm 内出现的障碍物走，
  保持约 50cm（超过65cm前进、小于50cm后退）
- stop_lidar_follow: 停止雷达跟随
- get_lidar_follow_status: 查询雷达跟随状态

※ 关于跟随（当前只支持雷达跟随）：
  * 用户说"雷达跟随""用雷达跟着我""跟着我""跟上我" → start_lidar_follow
  * 用户说"看护模式""摄像头跟着我" → 告诉用户视觉跟随暂时不可用，
    建议改用雷达跟随；**不要**编造工具调用
  * 雷达跟随会控制底盘，与导航互斥，开启时会自动停掉导航

【管家角色表现】—— 纯视觉效果，不控制任何硬件
- set_butler_mood: 设置页面上管家机器人的动作，可选在头顶气泡里显示一句短话
  · 可用动作：approve（认可点头）/ think（思考）/ deny（否认摇头）
    / scratch（困惑挠头）/ happy（微笑）/ wave（打招呼）
    / listen（倾听）/ idle（回到待机）
  · **几乎每次回复都建议调一次**，让角色的肢体语言和你说话的语气一致
  · 例：用户下达指令准备执行 → approve；查询等待中 → think；
    操作失败/说法不对 → deny；信息不够需要澄清 → scratch
  · 气泡文字必须很短（≤14 个汉字），且不要每次都填
- butler_say: 只让角色头顶冒一句口语化的短话（可配合动作）
- randomize_butler_idle: 随机播放一个待机小动作

规则：
1. 当用户说"这里是XXX"时，调用 save_current_location
2. 当用户说"删除XXX"时，调用 delete_saved_location
3. 当用户说"清空所有地点"时，调用 clear_all_saved_locations
4. 当用户说"去XXX"时，**只调用** navigate_to_saved_location 这一个工具
   （它会自动回退到预定义地点）
   ★ 它**只去、不回来**。车到了就停在那儿，不会自己跑回起点。
     用户想回起点时会单独说「回起点」，那时才调 return_to_start。
5. 当用户说"先去A再去B"、"依次去A和B"等连续多个地点时，调用
   navigate_sequence（支持自定义地点）
   ★ 多点导航会：到 A 后**停车等待用户按按钮确认**，按下后才去 B；
     到 B（最后一个）直接结束。若等按钮超时 → 整个任务取消。
     这是车端自动做的，**你不需要**调其他工具配合。
6. 当用户问"有哪些地点"时，调用 list_locations
7. 当用户说"跟着我"、"雷达跟随"时，调用 start_lidar_follow
8. 当用户说"停止跟随"时，调用 stop_lidar_follow
9. 当用户问"到了吗"、"导航状态"时，调用 get_navigation_status
10. 回复要简洁友好
11. 回复的同时顺手调一次 set_butler_mood，让管家做出符合语气的表情；
    连续两次不要用同一个动作，也不要每次都填 bubbles 文字
12. 当用户问天气（"天气怎么样""会下雨吗""明天冷不冷"）时，
    调用 get_weather；用户没说城市就留空 city 参数，用默认城市
13. 当用户问日期、时间、星期几，或话题涉及"今天/今年/最近"时，
    先调用 get_current_datetime —— 你的训练数据有截止时间，
    不要凭记忆猜今天几号
14. 工具查不到的事（如实时新闻、股价）就诚实说查不到，
    不要编造，也不要声称自己"无法联网"

【最重要】一轮只调一个工具：
- 除了 set_butler_mood / butler_say 这类纯表情工具，
  其它工具**每轮最多只调用一个**
- 绝对不要同时调用多个导航工具（会互相抢占，导致任务失败）
- 特别注意：navigate_to_saved_location **不会**自动返回起点；
  用户只说“去XXX”时，不要额外调用 return_to_start
- 用户说"去XXX"时，只需 navigate_to_saved_location 一个工具；
  **不要**先调用 save_current_location

【重要】车正在导航时用户说了新目的地：
- 直接调用导航工具即可，**不需要**先调 cancel_navigation ——
  系统会自动中断上一个任务（"说新目的地 = 改主意"）
- **不要**告诉用户"请先取消当前任务"；也**不要**因为当前任务是
  回起点就跟着调 return_to_start
- 如果工具返回里出现了旧任务的名字（例如"已有导航任务进行中
  （目标：回起点）"），那是旧任务的残留信息，**不是**用户的新意图，
  不要照抄这句话，也不要据此改调 return_to_start

【重要】导航工具已改为非阻塞：
- 当你调用导航工具后，工具会立即返回 "📍 正在前往..." 类消息
- 你必须立即将此消息告知用户，不要等待（导航已在后台执行）
- 后续到达/返回通知会由系统自动推送给用户
- 如下使用流程：
  用户: "去厨房"  →  你调用 navigate_to_saved_location("厨房")  →  工具返回 "📍 正在前往厨房..."
  →  你回复: "好的，正在前往厨房！"  ← 到此结束，不要等待"""


# ============================================================
# 本地小模型专用精简提示词
# ------------------------------------------------------------
# 上面那份 SYSTEM_PROMPT 为云端大模型而写，把 23 个工具的说明又
# 抄了一遍——而工具描述本身会通过 bind_tools 的 schema 再发一次。
#
# 对 3B 模型这是致命的：
#   完整提示   SYSTEM_PROMPT 2001 字符 → prompt 3254 tokens → 模型
#              直接输出 EOS（content 为空、tool_calls 为空）
#   精简提示   约 700 字符          → prompt 2402 tokens → 稳定命中
#
# 实测（fishbot-qwen:3b, 23 工具）：精简版 6/6 次全部正确调用，
# 单轮 0.5~0.6s；完整版 0/6。所以这里按 provider 分流。
# ============================================================
SYSTEM_PROMPT_COMPACT = PERSONA_TEXT + """

工具映射：
- 去X → navigate_to_saved_location
- 先去A再去B/依次去 → navigate_sequence
- 回起点/回去 → return_to_start
- 这里是X/记成X → save_current_location
- 在哪 → get_current_position
- 电量 → get_battery_status
- 天气 → get_weather
- 今天几号/几点 → get_current_datetime
- 雷达跟随/跟着我走 → start_lidar_follow
- 停止跟随 → stop_lidar_follow
- 说句话吧/让管家说 → butler_say（不是 set_butler_mood）
- 取消 → cancel_navigation

规则：
1. 用户在提要求时必须先调工具，不要只回文字
2. 回复只用一句话
3. 导航工具异步，调用后不要等待
4. 每次回复同步调一次 set_butler_mood
5. 【去X ≠ 回起点】用户说"去厕所/去厨房/去X"时只调
   navigate_to_saved_location，**绝对不要**调 return_to_start，
   也不要调 initialize_pose（那是重置定位，会让车定位跑到原点）
6. 车正在导航时用户又说了新目的地 → 直接调导航工具即可，
   系统会自动中断旧任务，**不需要**先调 cancel_navigation
7. 假如工具返回里有"请先取消"之类的提示，**不要**照抄，
   也不要因此改成回起点；按用户说的目的地重新调导航即可

禁止用文字描述代替调用工具。"""

# 当前实际使用的提示词
ACTIVE_SYSTEM_PROMPT = (SYSTEM_PROMPT_COMPACT if LLM_PROVIDER == "local"
                       else SYSTEM_PROMPT)


# ============================================================
# 小模型友好：历史裁剪
# ------------------------------------------------------------
# 3B 级别的模型在十几轮之后会「看到太多上下文直接吐 EOS」，
# 表现为 content 为空、也没有 tool_calls。云端大模型没这问题，
# 但本地小模型必须有上限。
#
# 裁剪时必须保证 tool_call 与 tool_result 成对：
# 只留 AIMessage(tool_calls) 却把对应的 ToolMessage 丢掉，或者反过来
# 只留 ToolMessage，OpenAI 兼容接口都会直接返回 400。
# ============================================================
MAX_HISTORY_MESSAGES = int(os.environ.get("FISHBOT_MAX_HISTORY", "12"))


def _strip_past_tool_turns(messages):
    """丢弃「以前轮次」留下的工具往返，只保留当前轮的。

    ============================================================
    为什么要剥离（2026-10-31 实机日志实锤）
    ============================================================
    用户先说「帮我回到原点」，模型正确调用 return_to_start，
    历史里于是留下完整的三段：

        Human: 帮我回到原点
        AI   : tool_calls=[return_to_start]
        Tool : 📍 正在返回起点...
        AI   : 📍 我会尽快回到起点，主人。
        Human: 回起点                    ← 新的一轮

    3B 模型把最后那条 AI 文本当成了「回答范例」，第二轮直接照抄：

        AI   : 📍 我会尽快回到起点位置，请稍等片刻。   ← 没调工具

    这就是少数样本污染（in-context imitation）。
    系统提示里已经写了「必须调工具」「禁止用文字描述代替调用工具」，
    对 3B 模型依然压不住 —— 结构性问题只能靠结构解决。

    ============================================================
    为什么只剥离「最后一轮之前」的部分
    ============================================================
    图里是 tools → agent，工具执行完还会回到本节点。
    那条路径下模型**必须**看到 AI(tool_calls) + Tool(result)
    才能总结工具结果。所以：

      * 最后一条 HumanMessage **之后**（当前轮）→ 原样保留
      * 最后一条 HumanMessage **之前**（历史轮）→ 剥掉工具往返

    保留 HumanMessage 序列，多轮上下文（比如「我叫小明」）不丢。

    可用环境变量 FISHBOT_KEEP_TOOL_HISTORY=1 关闭本行为。
    """
    if not messages:
        return list(messages)
    if os.environ.get("FISHBOT_KEEP_TOOL_HISTORY", "") == "1":
        return list(messages)

    last_human = -1
    for i in range(len(messages) - 1, -1, -1):
        if isinstance(messages[i], HumanMessage):
            last_human = i
            break

    if last_human <= 0:
        return list(messages)          # 没有更早的历史，无需处理

    head = messages[:last_human]
    tail = messages[last_human:]       # 当前轮：连工具往返一起保留

    cleaned = []
    i = 0
    while i < len(head):
        m = head[i]

        # 孤立的工具结果（对应的调用已不在）—— 留着会让接口报 400
        if type(m).__name__ == "ToolMessage":
            i += 1
            continue

        # ============================================================
        # 空 AIMessage 必须丢掉（2026-11-01 实机日志实锤）
        # ------------------------------------------------------------
        # 现象：checkpoint 里躺着一对
        #     Human: 回起点
        #     AI   : ''            ← content 空、也没有 tool_calls
        # 它来自「空回复兜底」：模型吐 EOS，重试也失败，空消息被存了下来。
        #
        # 危害：模型看到「用户说话 → 助手沉默」这种畸形样本，
        # 会削弱「用户提要求就必须调工具」的约束，后面更容易复读旧意图。
        # 而且 OpenAI 兼容接口对空 assistant content 本来就不友好。
        # ============================================================
        if (isinstance(m, AIMessage)
                and not getattr(m, "tool_calls", None)
                and not str(getattr(m, "content", "") or "").strip()):
            i += 1
            continue

        # 一整轮工具调用：AI(tool_calls) + Tool(...) + AI(最终文本)
        if isinstance(m, AIMessage) and getattr(m, "tool_calls", None):
            i += 1
            while i < len(head) and type(head[i]).__name__ == "ToolMessage":
                i += 1
            # 该轮的最终纯文本回答也要丢 —— 它才是被模仿的那个范例
            if i < len(head):
                nxt = head[i]
                if (isinstance(nxt, AIMessage)
                        and not getattr(nxt, "tool_calls", None)):
                    i += 1
            continue

        cleaned.append(m)
        i += 1

    if len(cleaned) < len(head):
        print("[AgentNode] 剥离历史工具回合 %d → %d 条（当前轮保留）"
              % (len(head), len(cleaned)))
    # ★ 折叠必须在 head+tail 拼接之后做：head 末尾和 tail 开头
    #   往往就是同一句话（用户连说两次「去厕所」）。
    return _collapse_user_runs(cleaned + tail)


# ============================================================
# 连续 HumanMessage 折叠
# ------------------------------------------------------------
# 【为什么必须折叠 —— 2026-11-01 实机日志实锤】
#
# 剥离掉所有工具回合后，剩下的全是用户自己说的话，于是变成：
#
#     Human: 回起点
#     Human: 回起点
#     Human: 这里是厕所，你记住
#     Human: 回起点
#     Human: 去厕所
#     Human: 去厕所          ← 当前轮
#
# 六条用户消息一条助手回复都没有 —— 这是任何 LLM 都没见过的畸形
# 对话格式。3B 模型直接做了「多数表决」：五条里三条是"回起点"，
# 于是对"去厕所"也回了 return_to_start（实机日志 [19] 就是这么错的）。
#
# ★ 另一个放大因素：本项目每轮回复都要顺带调 set_butler_mood，
#   于是**每一轮都是工具回合**，剥离后一条助手回复都不剩。
#   所以这不是偶发情况，而是每轮都发生。
#
# 处理：同一段连续的 HumanMessage 只保留**最后一条**（最新意图覆盖
# 旧意图），被顶掉的语句压成**一条背景摘要**放回去 —— 这样
# "我叫小明"这类事实记忆不会丢。
#
# 摘要里会**丢掉已经执行过的指令**（回起点/去X/取消…）：
#   那些指令早就执行完了，再让模型看见只会让它重复执行。
# ============================================================
_HISTORY_BACKGROUND_PREFIX = "（以下是更早轮次说过的话，仅作背景参考，不是当前指令）"

# 已经执行完、再看见只会诱发重复执行的历史语句特征
_HISTORY_NOISE_WORDS = (
    "取消", "停下", "停止", "跟随", "导航", "去", "到", "往", "回",
)


def _is_spent_command(text: str) -> bool:
    """判断一条历史用户消息是不是「已经执行完的指令」。"""
    if any(w in text for w in _HISTORY_NOISE_WORDS):
        return True
    return False


def _collapse_user_runs(messages):
    out = []
    displaced = []
    for m in messages:
        if (isinstance(m, HumanMessage) and out
                and isinstance(out[-1], HumanMessage)):
            txt = str(getattr(out[-1], "content", "") or "").strip()
            if txt and not _is_spent_command(txt):
                displaced.append(txt)
            out[-1] = m          # 最新一条覆盖旧的一条
            continue
        out.append(m)

    if not displaced:
        return out

    seen = []
    for t in displaced:
        if t not in seen:
            seen.append(t)
    seen = seen[-4:]

    note = HumanMessage(content="%s：%s" % (_HISTORY_BACKGROUND_PREFIX,
                                           " ／ ".join(seen)))
    print("[AgentNode] 连续用户消息已折叠，保留背景摘要 %d 条" % len(seen))

    # 摘要放在**当前指令之前**，且必须独立成条 ——
    # agent_node 取 last_user_text 时用的是「最后一条 HumanMessage」，
    # 合并进同一条会把历史里的地名混进意图判断。
    idx = len(out) - 1
    while idx >= 0 and not isinstance(out[idx], HumanMessage):
        idx -= 1
    if idx < 0:
        return out
    out.insert(idx, note)
    return out


def _trim_history(messages, limit=None):
    """保留最近 limit 条消息，并保证工具调用链完整。"""
    limit = limit or MAX_HISTORY_MESSAGES
    if len(messages) <= limit:
        return list(messages)

    kept = list(messages[-limit:])

    # 开口处不能是 ToolMessage：它前面那条带 tool_calls 的 AIMessage
    # 已经被裁掉了，孤立的工具结果会让接口报错。
    while kept and type(kept[0]).__name__ == "ToolMessage":
        kept.pop(0)

    # 开口处若是「带 tool_calls 的 AIMessage」，必须确认后面还有对应的
    # 工具结果，没有就一起删掉。
    while kept and getattr(kept[0], "tool_calls", None):
        if any(type(m).__name__ == "ToolMessage" for m in kept[1:]):
            break
        kept.pop(0)

    # 兜底：万一被裁空了，退回最后一条用户消息
    if not kept:
        for m in reversed(messages):
            if isinstance(m, HumanMessage):
                return [m]
        return list(messages[-1:])

    if len(kept) < len(messages):
        print(f"[AgentNode] 历史裁剪 {len(messages)} → {len(kept)} 条")
    return kept


# ============================================================
# 本地模型专用：破坏 prompt cache 的短标记
# ------------------------------------------------------------
# 【现象】连续问不同的问题时，大约 1/4 的请求会返回空
#   （content 为空、tool_calls 为空，耗时仅 0.1 秒）。
#   「说句话吧」甚至返回过上一个问题残留的内容。
#
# 【原因】Ollama 0.40 的 prompt cache：当两次请求只有最后一句
#   user 内容不同时，它会复用上一次的 KV cache，但复用的结果
#   是错的（模型相当于看不到真正的当前问题）。
#
# 【验证】12 条测试：
#   不加标记            9/12  （0.37s/条，快的都是错的）
#   在 system 末尾加标记 12/12 （1.35s/条）
#   在 system 开头加标记 10/12 （1.42s/条）
#
# 【代价】每次多约 1 秒 prefill。对聊天交互可接受，
#   换来的是工具调用不出错——错一次就是「车不动」，代价更高。
# ============================================================
def _nonce_prompt(text):
    """给本地模型的系统提示词加一个每次不同的短尾巴。"""
    if LLM_PROVIDER != "local":
        return text
    return text + "\n#%05d" % random.randint(0, 99999)


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

    # ---- 小模型友好：裁剪历史 ----
    # 3B 模型在十几轮之后会「看到太多上下文直接吐 EOS」，
    # 表现为 content 为空且没有 tool_calls（实测踩过）。
    # 这里只保留最近若干条，并且必须保证 tool_call / tool_result 成对，
    # 否则 OpenAI 兼容接口会直接 400。

    # ★ 先剥离「上一轮」的工具往返，再裁剪长度。
    #   顺序不能反：_trim_history 只按条数粗切，切完可能恰好
    #   留下一段「AI 最终文本」，那段才是被模仿的范例。
    messages = _strip_past_tool_turns(messages)
    messages = _trim_history(messages)

    # 添加系统提示（只加一次）
    if not any(isinstance(m, SystemMessage) for m in messages):
        messages = [SystemMessage(content=_nonce_prompt(ACTIVE_SYSTEM_PROMPT))] + messages

    print(f"[AgentNode] 调用 {LLM_PROVIDER} 模型 {LLM_MODEL} "
          f"(消息数={len(messages)})...")
    response = llm_with_tools.invoke(messages)

    # 打印 LLM 返回的内容
    print(f"[AgentNode] 模型返回: type={type(response).__name__}")
    if hasattr(response, 'tool_calls') and response.tool_calls:
        print(f"[AgentNode] tool_calls: {[tc.get('name') for tc in response.tool_calls]}")
    if hasattr(response, 'content'):
        print(f"[AgentNode] content: {str(response.content)[:200]!r}")

    # ---- 空回复兜底 ----
    # 小模型偶尔会「一句话都不说」。对话就断了，用户会以为程序卡死。
    # 这里重试一次：去掉历史、只留系统提示 + 最后一句话。
    if (not getattr(response, "tool_calls", None)
            and not str(getattr(response, "content", "") or "").strip()):
        print("[AgentNode] ⚠️ 模型返回空，去掉历史重试一次")
        last_human = None
        for m in reversed(messages):
            if isinstance(m, HumanMessage):
                last_human = m
                break
        if last_human is not None:
            try:
                response = llm_with_tools.invoke([
                    SystemMessage(content=_nonce_prompt(ACTIVE_SYSTEM_PROMPT)),
                    last_human,
                ])
                print(f"[AgentNode] 重试结果: tool_calls="
                      f"{[tc.get('name') for tc in (getattr(response, 'tool_calls', None) or [])]} "
                      f"content={str(getattr(response, 'content', ''))[:120]!r}")
            except Exception as e:  # noqa: BLE001
                print(f"[AgentNode] 重试失败: {e}")

    # ============================================================
    # 单轮 tool_calls 互斥收敛（2026-11-01）
    # ------------------------------------------------------------
    # 实测故障：用户说「去厕所」，3B 模型一次吐出 3 个工具：
    #   ['save_current_location', 'return_to_start',
    #    'navigate_to_saved_location']
    # 后果是三重灾难：
    #   1. save_current_location 把**当前位置**存成了「厕所」→ 数据污染
    #   2. return_to_start 起了一个导航进程
    #   3. navigate_to_saved_location 再起一个 →
    #      「已有导航任务进行中（目标：回起点）」，真正的任务失败
    #
    # 为什么必须在代码层拦：3B 模型的工具选择是**确定性错误**，
    # 同一句话每次都错，调 prompt 只能降低概率，不能消除。
    # 而且用户说的目的地永远比 return_to_start 更明确，
    # 额外调 return_to_start 纯属重复。
    #
    # 规则：
    #   - 导航类工具之间互斥，只保留模型输出的**第一个**
    #   - 同轮出现导航类时，丢弃 save_current_location
    #     （"去X"里不存在"顺便把当前位置记成X"的合理意图）
    #   - 非控制类工具（set_butler_mood / get_weather 等）原样保留
    # ============================================================
    if getattr(response, "tool_calls", None):
        # 1) 先做语义防护：拦掉被"起点"二字误触发的 initialize_pose
        #    （它会把 AMCL 位姿重置到 (0,0)，破坏性最强，必须最先处理）
        last_user_text = ""
        for m in reversed(messages):
            if isinstance(m, HumanMessage):
                last_user_text = str(getattr(m, "content", "") or "")
                break
        response = _guard_initialize_pose(response, last_user_text)

        # 2) 用户说了能查到的目的地，模型却要回起点/重置定位 → 强制纠正
        response = _guard_nav_target(response, last_user_text)

        # 3) 再收敛多个互斥工具
        response = _dedupe_exclusive_tool_calls(response)

    # ---- 绝不让空 AIMessage 落进 checkpoint ----
    # 空消息进了历史会变成「Human: X / AI: ''」这种畸形样本，
    # 削弱"提要求必须调工具"的约束（实机日志里确实出现过）。
    if (not getattr(response, "tool_calls", None)
            and not str(getattr(response, "content", "") or "").strip()):
        print("[AgentNode] ⚠️ 最终仍为空回复，替换为兜底话术（不存空消息）")
        response = AIMessage(content="我在听，主人。请再说一次？")

    return {"messages": [response]}


# 会「抢占底盘 / 导航进程」的工具：同一轮只能执行一个。
#
# 值 = 优先级，数字越小越优先。
#
# 【为什么需要优先级，而不是简单取第一个】
# 实测模型输出顺序是
#   ['save_current_location', 'return_to_start',
#    'navigate_to_saved_location']
# 如果按顺序保留第一个导航工具，留下的是 return_to_start —— 但用户说的
# 是「去厕所」，正确意图就是**只去厕所**。
#
# 关键事实：只要模型给出了**明确的**目的地，就该优先执行它；
# return_to_start 视为被顺手多带的低价值项。
#
# ★ 2026-11-01 修订：navigate_* 已**不再**自动回起点
#   （那条流程已按用户要求彻底删除）。
#   这条优先级规则依然成立 —— 用户嘴里的目的地永远比"回起点"更重要。
_EXCLUSIVE_NAV_TOOLS = {
    # ---- 0：明确的导航目的地（最高优先级）----
    "navigate_to_saved_location": 0,
    "navigate_to_location": 0,
    "navigate_to_predefined_point": 0,
    "navigate_to_xy": 0,
    "navigate_sequence": 0,
    # ---- 1：进入跟随模式 ----
    "start_patrol_mode": 1,
    "start_lidar_follow": 1,
    # ---- 2：停止 / 取消类 ----
    "stop_patrol_mode": 2,
    "stop_lidar_follow": 2,
    "cancel_navigation": 2,
    # ---- 3：导航回起点（navigate_* 已自带回程，多半是附带误调）----
    "return_to_start": 3,
    # ---- 4：最低 —— 重置定位是破坏性操作，最容易被"起点"二字误触发 ----
    "initialize_pose": 4,
}

# 与导航同时出现时应当丢弃的工具：会写坏地点数据
_DROP_WITH_NAV_TOOLS = {
    "save_current_location",
}

# ============================================================
# initialize_pose 的语义防护（2026-11-01）
# ------------------------------------------------------------
# 实测故障：用户说「回起点」，模型调了 initialize_pose。
# 原因：工具描述里有"起点"二字，3B 模型看到"回**起点**"就选了它。
# 后果：initialize_amcl_pose() 直接把 AMCL 位姿重置成 (0,0,3.14)，
#       表现为「车瞬间重定位到起点」而不是「开过去」——
#       而且如果车实际不在原点，之后所有导航都是错的。
#
# 上面 _EXCLUSIVE_NAV_TOOLS 的优先级只能处理"多个工具同时出现"的情况；
# 如果模型**只**吐了 initialize_pose 一个，去重逻辑不会触发。
# 所以这里再加一层基于**用户原话**的判断。
#
# 规则：
#   用户明确说要初始化  → 放行（这是个正当操作）
#   用户没说初始化      → 丢弃 initialize_pose
#                        若同时能看出导航语义，补一个 return_to_start
# ============================================================

# 用户明确要求重置定位的说法
_INIT_POSE_INTENT_WORDS = (
    "初始化位姿", "初始化定位", "初始化amcl", "初始化 amcl",
    "重置定位", "重设定位", "设置初始位置", "位姿初始化",
)

# 用户想要「回起点」的说法（导航语义，绝不能用 initialize_pose）
_GO_HOME_INTENT_WORDS = (
    "回起点", "返回起点", "回到起点", "回家", "回原点", "返回原点",
    "回去", "回出发",
)

# 「回 X」里属于"回家"语义的关键词 —— 出现这些就不当作具体目的地
_HOME_OBJECT_WORDS = ("家", "起点", "原点", "出发", "去的地方")

# 目的地提取前缀，长前缀必须排在短前缀前面
_DEST_PREFIXES = (
    "带我去", "带我到", "导航到", "导航去", "前往", "去一下",
    "去一趟", "去", "到", "往",
)

# 目的地后面这些词要截掉
_DEST_TAIL_SPLIT = (
    "吧", "呢", "啊", "呀", "哦", "嘛", "好吗", "行吗", "谢谢",
    "。", "，", "！", "？", "、", ",", ".", "!", "?", " ",
)


def _extract_destination(text: str):
    """从「去厕所」「带我去厨房」里抠出目的地名词，抠不出返回 None。

    ★ 只用于**纠错**，不用于主动决策：
      抠出来的词还必须能在 LocationStore / LOCATION_MAP 里查到，
      才会被采纳（见 _resolve_destination）。这样「去一下」这种
      残句、以及"我去了"这种陈述句都不会误触发。
    """
    if not text:
        return None
    t = text.strip()
    for p in _DEST_PREFIXES:
        idx = t.find(p)
        if idx < 0:
            continue
        rest = t[idx + len(p):]
        for sep in _DEST_TAIL_SPLIT:
            cut = rest.find(sep)
            if cut >= 0:
                rest = rest[:cut]
        rest = rest.strip()
        if rest:
            return rest
    return None


def _resolve_destination(name: str):
    """目的地名词 → 是否是可导航的已知地点。是则返回规范化名称。

    只有**已知地点**才允许触发纠错，避免把闲聊里的"去"当成导航。
    """
    if not name or len(name) > 12:
        return None
    if any(w in name for w in _HOME_OBJECT_WORDS):
        return None                      # 「回家 / 回起点」交给 return_to_start
    try:
        from autopatrol_robot.tools.nav_tools import LOCATION_MAP
        from autopatrol_robot.location_store import get_location_store
    except Exception:                    # noqa: BLE001
        return None
    if name in LOCATION_MAP:
        return name
    try:
        if get_location_store().find_index(name) is not None:
            return name
    except Exception:                    # noqa: BLE001
        pass
    return None


def _make_nav_call(name: str, call_id: str = "guard_nav"):
    return {"name": "navigate_to_saved_location",
            "args": {"name": name},
            "id": call_id}


# 所有「有明确目的地」的导航工具
_NAV_DEST_TOOLS = (
    "navigate_to_saved_location", "navigate_to_location",
    "navigate_to_predefined_point", "navigate_to_xy",
    "navigate_sequence",
)


def _already_heading_somewhere(calls) -> bool:
    """本轮是否已经有"去某地"的工具调用（决定要不要补一个）。"""
    return any(tc.get("name") in _NAV_DEST_TOOLS for tc in calls)


def _replace_response_calls(response, new_calls):
    """AIMessage 的 tool_calls 是只读属性，用 model_copy 造新对象。"""
    try:
        return response.model_copy(update={"tool_calls": new_calls})
    except Exception:                                  # noqa: BLE001
        try:
            response.tool_calls = new_calls
        except Exception:                              # noqa: BLE001
            pass
        return response


def _guard_initialize_pose(response, user_text: str):
    """防止"回起点"被误解成"把定位重置到起点"。"""
    if not getattr(response, "tool_calls", None):
        return response

    has_init = any(tc.get("name") == "initialize_pose"
                   for tc in response.tool_calls)
    if not has_init:
        return response

    text = (user_text or "").lower()

    # 用户确实说要初始化 -> 正常放行
    if any(w in text for w in _INIT_POSE_INTENT_WORDS):
        return response

    dest = _resolve_destination(_extract_destination(text) or "")
    wants_home = any(w in text for w in _GO_HOME_INTENT_WORDS)

    if dest:
        # 用户其实说的是「去厕所」→ 换成真正的导航
        new_calls = [tc for tc in response.tool_calls
                     if tc.get("name") != "initialize_pose"]
        # 模型可能已经给了 navigate_*，别再加一个（会变成重复调用）
        if not _already_heading_somewhere(new_calls):
            new_calls.append(_make_nav_call(dest))
        print("[AgentNode] 🛡️ 用户说的是「去%s」（不是重置定位），"
              "已把 initialize_pose 换成 navigate_to_saved_location" % dest)
    elif wants_home:
        # 用户说"回起点" -> 换成 return_to_start
        new_calls = [tc for tc in response.tool_calls
                     if tc.get("name") != "initialize_pose"]
        # 模型可能已经给了 return_to_start，别再加一遍（会触发去重日志）
        if not any(tc.get("name") == "return_to_start" for tc in new_calls):
            new_calls.append({
                "name": "return_to_start",
                "args": {},
                "id": "guard_ret_home",
            })
        print("[AgentNode] 🛡️ 用户说的是回起点（不是重置定位），"
              "已把 initialize_pose 换成 return_to_start")
    else:
        # 既没说要初始化、也没看出导航语义 -> 直接丢掉这个破坏性操作
        new_calls = [tc for tc in response.tool_calls
                     if tc.get("name") != "initialize_pose"]
        print("[AgentNode] 🛡️ 用户未要求初始化定位，"
              "已丢弃 initialize_pose（破坏性操作）")

    if not new_calls:
        # 全被丢掉了：给一句提示，避免空回复
        return AIMessage(content="需要我初始化定位吗？这会重置当前位置。")

    return _replace_response_calls(response, new_calls)


# ============================================================
# 导航意图强制校正（2026-11-01）
# ------------------------------------------------------------
# 实机日志（agent_node 原始输出）：
#   [14] 用户: 去厕所   → [15] AI: navigate_to_saved_location   ✓
#   [16] Tool: ❌ 启动导航失败：已有导航任务进行中（目标：回起点）
#   [17] AI  : 请先取消当前的导航任务，我才能回起点。
#   [18] 用户: 去厕所   → [19] AI: return_to_start              ✗✗✗
#
# 第 19 步是致命错误：用户明明说"去厕所"，模型却让车回起点。
# 根因有两层（都已在别处修掉）：
#   1. nav_manager 硬拒绝新任务 → 产生带"回起点"字样的错误消息；
#   2. 剥离历史后剩下一串裸 HumanMessage，"回起点"在数量上占多数。
#
# 但即使修了上面两层，3B 模型的工具选择依然是概率性的。所以再加
# 一层**确定性兜底**：只要用户话里有**能查到的目的地**，就绝不允许
# 这一轮去执行 return_to_start / initialize_pose。
#
# 只在模型**选错**时介入，模型选对了（navigate_*）完全不动。
# ============================================================
_WRONG_FOR_DEST = {"return_to_start", "initialize_pose"}


def _guard_nav_target(response, user_text: str):
    """用户说了明确目的地，模型却要回起点/重置定位 → 强改为去那儿。"""
    if not getattr(response, "tool_calls", None):
        return response

    names = [tc.get("name") for tc in response.tool_calls]
    if not any(n in _WRONG_FOR_DEST for n in names):
        return response            # 模型没选错，不干预

    dest = _resolve_destination(_extract_destination(user_text or "") or "")
    if not dest:
        return response

    new_calls = [tc for tc in response.tool_calls
                 if tc.get("name") not in _WRONG_FOR_DEST]
    if not _already_heading_somewhere(new_calls):
        new_calls.append(_make_nav_call(dest))

    print("[AgentNode] 🛡️ 用户说的是「去%s」，模型却选了 %s → "
          "已强制改为 navigate_to_saved_location" % (dest, names))
    return _replace_response_calls(response, new_calls)


def _dedupe_exclusive_tool_calls(response):
    """收敛单轮 tool_calls：导航类互斥，避免抢进程 + 数据污染。"""
    calls = list(response.tool_calls)
    names = [c.get("name") for c in calls]

    nav_calls = [c for c in calls if c.get("name") in _EXCLUSIVE_NAV_TOOLS]
    if not nav_calls:
        return response

    # 按优先级挑一个（同级则保留模型先给出的那个）
    kept_nav = min(nav_calls,
                   key=lambda c: _EXCLUSIVE_NAV_TOOLS[c.get("name")])
    dropped = []

    new_calls = []
    for c in calls:
        name = c.get("name")
        if name in _EXCLUSIVE_NAV_TOOLS:
            if c is kept_nav:
                new_calls.append(c)
            else:
                dropped.append(name)
            continue
        if name in _DROP_WITH_NAV_TOOLS:
            dropped.append(name)
            continue
        new_calls.append(c)

    if not dropped:
        return response

    print("[AgentNode] ⚠️ 同一轮多个冲突工具 %s → 只保留 %s（丢弃 %s）"
          % (names, kept_nav.get("name"), dropped))

    # AIMessage 的 tool_calls 是只读属性，用 model_copy 造一个新对象
    try:
        return response.model_copy(update={"tool_calls": new_calls})
    except Exception:                                  # noqa: BLE001
        # 极老版本的 langchain 没有 model_copy，退回直接赋值
        try:
            response.tool_calls = new_calls
        except Exception:                              # noqa: BLE001
            pass
        return response


# ============================================================
# 第四步：构建状态图
# ============================================================
def build_agent():
    """构建并编译 Agent"""
    
    # 创建工具执行节点
    tool_node = ToolNode(tools=ACTIVE_TOOLS)
    
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
    print("  • 跟着我")
    print("  • 停止跟随")
    print("  • 雷达跟随开着吗")
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