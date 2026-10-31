"""
tools/butler_tools.py
让 AI 助手直接驱动页面上的管家角色（表情 / 动作 / 头顶气泡）。

设计意图
--------
把「角色行为」做成工具之后，AI 不再只是返回一段文字，而是能同时：
  * 挑一个合适的动作（认可／思考／否认／挠头／微笑／打招呼／倾听）
  * 说一句很短的话显示在角色头顶

于是聊天回复的**语气**和角色的**肢体语言**能对齐 —— 用户说"去厨房"，
AI 可以一边调 navigate 工具，一边 `set_butler_mood('approve')` 表示收到。

实现方式
--------
前端通过 socket 事件 `butler_action` 接收动作，消息体：
    {"action": "approve", "text": "好的"}
本模块只负责把 AI 的调用转成一次 socketio.emit。

注意：所有动作都是纯前端视觉效果，**不会驱动电机**，与安全约束无关。
"""

from typing import Literal, Optional

from langchain_core.tools import tool
from pydantic import BaseModel, Field


# 前端 Butler 控制器支持的完整动作集（必须与 control.html 里
# Butler 的 ACTIONS 常量保持一致，否则会被忽略降级成 idle）
ACTIONS = (
    'idle',      # 待机（回到呼吸动画）
    'happy',     # 微笑 + 蹦一下
    'approve',   # 点头两下 + 月牙眼
    'deny',      # 摇头 + X 眼
    'think',     # 歪头托腮 + 嘟嘴
    'scratch',   # 挠头 + 头微摇
    'wave',      # 挥手打招呼
    'listen',    # 歪头倾听
)

_ACTION_HINT = "、".join(ACTIONS)


def _emit(action: str, text: Optional[str] = None) -> None:
    """把动作推给前端。

    延迟导入 socketio：tools 包在 web_controller 之外也可能被导入
    （例如单元测试），那种场景下没有 socketio 实例，这里静默跳过即可，
    不能让一个纯视觉动作把整个 AI 调用链搞崩。
    """
    try:
        from autopatrol_robot import web_controller as wc
        payload = {'action': action}
        if text:
            payload['text'] = text
        wc.socketio.emit('butler_action', payload)
        print('[Butler] emit action=%s text=%r' % (action, text))
    except Exception as exc:                        # noqa: BLE001
        print('[Butler] 推送失败（忽略）: %s' % exc)


class ButlerActionInput(BaseModel):
    action: Literal[
        'idle', 'happy', 'approve', 'deny',
        'think', 'scratch', 'wave', 'listen',
    ] = Field(description="动作：%s" % _ACTION_HINT)
    text: Optional[str] = Field(
        default=None,
        description="可选，气泡短句，≤14 汉字。不填就不显示气泡。",
    )


@tool("set_butler_mood", args_schema=ButlerActionInput)
def set_butler_mood(action: str, text: Optional[str] = None) -> str:
    """让管家机器人做表情动作。纯视觉，不控制硬件。

    几乎每次回复都可顺手调一次，让肢体语言和语气一致。
    挑选：执行指令→approve，等待→think，失败→deny，
    需澄清→scratch，成功→happy。不要连续两次用同一个动作。
    """
    if action not in ACTIONS:
        return '⚠️ 不支持的动作: %s（可用: %s）' % (action, _ACTION_HINT)
    if text:
        text = str(text).strip()
        # 气泡空间有限，超长会被前端裁掉，这里主动截断更可控
        if len(text) > 20:
            text = text[:20] + '…'
    _emit(action, text)
    return '✅ 管家动作已设为 %s%s' % (
        action, ('，气泡："%s"' % text) if text else '')


class ButlerSayInput(BaseModel):
    text: str = Field(description="要在角色头顶气泡显示的一句话，控制在 14 个汉字以内。")
    action: Optional[str] = Field(
        default='happy',
        description="配合的动作，默认 happy。可选：%s" % _ACTION_HINT,
    )


@tool("butler_say", args_schema=ButlerSayInput)
def butler_say(text: str, action: Optional[str] = 'happy') -> str:
    """让管家机器人头顶冒出一句短话（附带一个动作）。

    和你返回给用户的正文不同：气泡是**角色在说话**，应当口语化、
    拟人化，例如「我这就去看看」「有点难办呢」。
    正文照常写完整回答，气泡只是点睛。
    """
    act = action if action in ACTIONS else 'happy'
    t = (text or '').strip()
    if not t:
        return '⚠️ 气泡内容不能为空'
    if len(t) > 20:
        t = t[:20] + '…'
    _emit(act, t)
    return '✅ 管家说："%s"（动作 %s）' % (t, act)


# ---- 下一轮待机行为，供「让 AI 每隔几秒自主行动」使用 ----
class ButlerIdleInput(BaseModel):
    pass


@tool("randomize_butler_idle", args_schema=ButlerIdleInput)
def randomize_butler_idle() -> str:
    """让管家随机挑一个待机小动作（播放一次）。

    用于「角色长时间没人理、自己找点事做」的场景。
    一般不需要 AI 主动调 —— 前端已经内置了本地待机轮换，
    这个工具是给后端定时任务或特殊编排用的。
    """
    import random
    pool = ['happy', 'think', 'scratch', 'deny', 'wave', 'listen']
    act = random.choice(pool)
    _emit(act, None)
    return '✅ 管家随机待机动作: %s' % act


BUTLER_TOOLS = [set_butler_mood, butler_say, randomize_butler_idle]
