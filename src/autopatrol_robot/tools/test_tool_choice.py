#!/usr/bin/env python3
"""
tools/test_tool_choice.py
本地小模型工具选择 / 文本化工具调用恢复 回归测试

事故链（2026-11-01，用户提供的两轮 agent_node 原始日志）：

    用户: 这里是书房，你记住
    AI  : 'odge_to_saved_location("书房")'     ← 只是文本，tool_calls 为空
    用户: 回起点
    AI  : 'return_to_start()'                  ← 同样只是文本

后果：
    1. 车一步不动 —— 没有任何工具真的被执行；
    2. 这段文字作为 AIMessage 进了 checkpoint，成了下一轮的「回答范例」，
       模型接着模仿，越错越离谱。

根因两层（都已修）：
    A. 系统提示词从 827 字符涨到 1101 字符（为了把「去X ≠ 保存X」讲清楚）。
       实测这个 3B 模型当场退化：该调工具时只应声「知道了，主人。」。
       → 提示词改回等长短版本，讲道理挪到工具描述和代码层。
    B. 工具 schema 共 6056 字符，18 个工具。同样拖垮模型：
       「回起点」0/5 次能调出工具。
       → 精简全部工具描述，schema 降到约 5300。
    C. 兜底：_recover_textual_tool_calls 把写成文本的调用恢复成真调用。

本测试分三部分：
    Ⅰ 离线单测：解析 / 守卫逻辑（不联网、不调模型）
    Ⅱ 规模检查：提示词长度、schema 体积不许再涨回去
    Ⅲ 真实模型（可选）：设 FISHBOT_TEST_LLM=1 才跑，需要 ollama 在跑
"""

import os
import sys
import json

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

PASS = 0
FAIL = 0
FAILED = []


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ✅ {name}")
    else:
        FAIL += 1
        FAILED.append(name)
        print(f"  ❌ {name}   {detail}")


def section(title):
    print(f"\n{'=' * 66}\n{title}\n{'=' * 66}")


import autopatrol_robot.agent_graph as AG                            # noqa: E402
from autopatrol_robot.agent_graph import (                            # noqa: E402
    _parse_textual_call,
    _match_tool_name,
    _recover_textual_tool_calls,
    _guard_save_intent,
    _guard_position_query,
    _is_position_query,
    SYSTEM_PROMPT_COMPACT,
    ACTIVE_TOOLS,
)


class FakeResp:
    def __init__(self, calls=None, content=""):
        self.tool_calls = [{"name": n, "args": a, "id": f"id_{i}"}
                           for i, (n, a) in enumerate(calls or [])]
        self.content = content

    def model_copy(self, update=None):
        r = FakeResp()
        r.tool_calls = (update or {}).get("tool_calls", self.tool_calls)
        r.content = (update or {}).get("content", self.content)
        return r


def names_of(r):
    return [tc["name"] for tc in r.tool_calls]


# ============================================================
# Ⅰ-1  工具名模糊匹配
# ============================================================
section("Ⅰ-1  _match_tool_name —— 模型把名字开头写丢时怎么认")

should_match = [
    ("navigate_to_saved_location", "navigate_to_saved_location"),
    ("avigate_to_saved_location", "navigate_to_saved_location"),   # 缺 n
    ("odge_to_saved_location", "navigate_to_saved_location"),      # 少 4 字符
    ("saved_location", "navigate_to_saved_location"),
    ("return_to_start", "return_to_start"),
    ("list_locations", "list_locations"),
    ("set_butler_mood", "set_butler_mood"),
]
for raw, want in should_match:
    got = _match_tool_name(raw)
    check(f"{raw!r} → {want}", got == want, f"实际 {got!r}")

# 太短的词绝不猜：猜错就是把车开走 / 把地点删掉
should_reject = ["location", "odge", "save_location", "start", "", "xyz"]
for raw in should_reject:
    got = _match_tool_name(raw)
    check(f"{raw!r} 拒绝模糊匹配", got is None, f"实际 {got!r}")

# 破坏性工具只接受精确名字
check("delete_saved_location 精确匹配可用",
      _match_tool_name("delete_saved_location") == "delete_saved_location")


# ============================================================
# Ⅰ-2  整句解析
# ============================================================
section("Ⅰ-2  _parse_textual_call —— 从文本里还原调用")

must_parse = [
    ('odge_to_saved_location("书房")',
     ("navigate_to_saved_location", {"name": "书房"})),
    ("return_to_start()", ("return_to_start", {})),
    ('avigate_to_saved_location {"name": "书房"}',
     ("navigate_to_saved_location", {"name": "书房"})),
    ('navigate_to_saved_location("书房")',
     ("navigate_to_saved_location", {"name": "书房"})),
    ('set_butler_mood({"action": "approve"})',
     ("set_butler_mood", {"action": "approve"})),
    ('save_current_location(name="书房")',
     ("save_current_location", {"name": "书房"})),
    ('<tool_call>navigate_to_saved_location({"name":"厨房"})</tool_call>',
     ("navigate_to_saved_location", {"name": "厨房"})),
    ('```json\n{"name": "return_to_start", "arguments": {}}\n```',
     ("return_to_start", {})),
    ('navigate_to_saved_location("厨房")。',
     ("navigate_to_saved_location", {"name": "厨房"})),
]
for src, want in must_parse:
    got = _parse_textual_call(src)
    check(f"{src[:46]!r}", got == want, f"实际 {got}")

# 正常回复绝不能被误当成调用
must_not_parse = [
    "好的，正在前往书房！",
    "知道了，主人。",
    "主人，书房已经记下了。",
    "电量还有 82%。",
    "我在想，你是不是想让我去书房看看呢？那里好像还有一些书。",
]
for src in must_not_parse:
    got = _parse_textual_call(src)
    check(f"不误判 {src[:26]!r}", got is None, f"实际 {got}")

# 缺必填参数的不恢复（宁可不动，也不能空参数执行）
check("缺必填参数不恢复",
      _parse_textual_call("navigate_to_saved_location()") is None,
      f"实际 {_parse_textual_call('navigate_to_saved_location()')}")


# ============================================================
# Ⅰ-3  _recover_textual_tool_calls 端到端
# ============================================================
section("Ⅰ-3  _recover_textual_tool_calls —— 本轮事故复现")

r = _recover_textual_tool_calls(FakeResp(content='return_to_start()'))
check("【事故复现】return_to_start() 恢复成真调用",
      names_of(r) == ["return_to_start"], f"实际 {names_of(r)}")
check("【事故复现】content 被清空（不污染 checkpoint）",
      not str(r.content).strip(), f"实际 {r.content!r}")

r = _recover_textual_tool_calls(FakeResp(content='odge_to_saved_location("书房")'))
check("【事故复现】odge_to_saved_location 恢复",
      names_of(r) == ["navigate_to_saved_location"], f"实际 {names_of(r)}")
check("【事故复现】参数正确",
      r.tool_calls[0]["args"] == {"name": "书房"}, f"实际 {r.tool_calls[0]['args']}")

# 已有真工具调用时不动
r = _recover_textual_tool_calls(
    FakeResp([("return_to_start", {})], content="好的"))
check("已有 tool_calls 时不干预", names_of(r) == ["return_to_start"])

# 正常文本原样保留
r = _recover_textual_tool_calls(FakeResp(content="好的，正在前往书房！"))
check("正常文本原样返回", str(r.content) == "好的，正在前往书房！")

# 看起来像调用但认不出 → 清空交给重试
r = _recover_textual_tool_calls(FakeResp(content='zzz_qqq_xxx("书房")'))
check("认不出时清空 content 触发重试",
      not str(r.content).strip(), f"实际 {r.content!r}")


# ============================================================
# Ⅱ  规模纪律：提示词 / schema 不许再涨
# ============================================================
section("Ⅱ  规模上限（这是 3B 模型的生死线）")

PROMPT_LIMIT = 900       # 实测 1101 会崩，827 正常，留 70 字符余量
check(f"SYSTEM_PROMPT_COMPACT ≤ {PROMPT_LIMIT} 字符",
      len(SYSTEM_PROMPT_COMPACT) <= PROMPT_LIMIT,
      f"实际 {len(SYSTEM_PROMPT_COMPACT)}")

schema_blob = json.dumps(
    [{"name": t.name,
      "description": t.description,
      "parameters": t.args_schema.model_json_schema()}
     for t in ACTIVE_TOOLS], ensure_ascii=False)
SCHEMA_LIMIT = 5600      # 实测 6056 时「回起点」0/5，5300 时 5/5
check(f"工具 schema ≤ {SCHEMA_LIMIT} 字符",
      len(schema_blob) <= SCHEMA_LIMIT, f"实际 {len(schema_blob)}")

for t in ACTIVE_TOOLS:
    n = len(t.description or "")
    check(f"{t.name} 描述 ≤ 60 字符", n <= 60, f"实际 {n}")


# ============================================================
# Ⅱ-2  与上一轮修复的协作（不许把 save 纠错搞坏）
# ============================================================
section("Ⅱ-2  _guard_save_intent 仍生效")

import autopatrol_robot.tools.loc_tools as _lt                       # noqa: E402,F401
import autopatrol_robot.location_store as _ls                        # noqa: E402
import tempfile                                                      # noqa: E402

_tmp = os.path.join(tempfile.mkdtemp(prefix="fishbot_tc_"), "cfg.yaml")
with open(_tmp, "w") as f:
    f.write("patrol_node:\n  ros__parameters:\n"
            "    target_points: [0.0, 0.0, 0.0]\n"
            "    custom_points: {书房: [1.9, 0.35, 2.56]}\n")
_orig = _ls.LocationStore.__init__


def _patched(self):
    _orig(self)
    self._yaml_path = _tmp


_ls.LocationStore.__init__ = _patched
_ls._store = None

r = _guard_save_intent(FakeResp([("save_current_location", {"name": "书房"})]),
                       "去书房")
check("去书房被误判成 save → 改成导航",
      names_of(r) == ["navigate_to_saved_location"], f"实际 {names_of(r)}")

r = _guard_save_intent(FakeResp([("save_current_location", {"name": "书房"})]),
                       "这里是书房，你记住")
check("这里是书房 → 保留 save",
      names_of(r) == ["save_current_location"], f"实际 {names_of(r)}")


# ============================================================
# Ⅱ-3  位置查询护栏
# ============================================================
section("Ⅱ-3  _guard_position_query —— 问位置却直接编答案")

# 该触发
for q in ["现在在哪", "我在哪", "当前位置", "在哪呢", "你坐标多少"]:
    check(f"{q!r} 认作位置查询", _is_position_query(q), "识别失败")

    resp = _guard_position_query(FakeResp(content="我在主人指定的目的地等待指令。"), q)
    check(f"{q!r} → 强制 get_current_position",
          names_of(resp) == ["get_current_position"], f"实际 {names_of(resp)}")
    check(f"{q!r} → 编造的文本被清空",
          not str(resp.content).strip(), f"实际 {resp.content!r}")

# 不该触发：会误伤导航 / 保存 / 闲聊
for q in ["去书房", "书房在哪", "回起点", "记住这里是书房", "你好", "现在几点"]:
    check(f"{q!r} 不认作位置查询", not _is_position_query(q), "误判")

    resp = _guard_position_query(FakeResp(content="好的"), q)
    check(f"{q!r} → 不干预", names_of(resp) == [], f"实际 {names_of(resp)}")

# 模型已经调了工具（哪怕是别的）→ 不干预
resp = _guard_position_query(
    FakeResp([("get_battery_status", {})], content=""), "我在哪")
check("已调工具时不干预（避免抢掉模型的正确判断）",
      names_of(resp) == ["get_battery_status"], f"实际 {names_of(resp)}")

# 模型自己调对了 → 原样保留
resp = _guard_position_query(
    FakeResp([("get_current_position", {})], content=""), "现在在哪")
check("模型自己调对时不动",
      names_of(resp) == ["get_current_position"], f"实际 {names_of(resp)}")


# ============================================================
# Ⅲ  真实模型（可选）
# ============================================================
if os.environ.get("FISHBOT_TEST_LLM") == "1":
    section("Ⅲ  真实模型工具选择（fishbot-qwen:3b）")
    from langchain_core.messages import SystemMessage, HumanMessage

    EXPECT = [
        ("这里是书房，你记住", "save_current_location"),
        ("记住这是书房", "save_current_location"),
        ("去书房", "navigate_to_saved_location"),
        ("带我去厨房", "navigate_to_saved_location"),
        ("回起点", "return_to_start"),
        ("回家", "return_to_start"),
        ("先去厨房再去客厅", "navigate_sequence"),
        ("跟着我走", "start_lidar_follow"),
        ("停止跟随", "stop_lidar_follow"),
        ("电量多少", "get_battery_status"),
        ("今天几号", "get_current_datetime"),
        ("现在在哪", "get_current_position"),
        ("有哪些地点", "list_locations"),
        ("说句话吧", "butler_say"),
        ("取消导航", "cancel_navigation"),
        ("导航状态", "get_navigation_status"),
    ]
    ROUNDS = int(os.environ.get("FISHBOT_TEST_ROUNDS", "3"))
    good = total = 0
    for q, want in EXPECT:
        hits = []
        for i in range(ROUNDS):
            human = HumanMessage(content=q)
            resp = AG.llm_with_tools.invoke([
                SystemMessage(content=AG._nonce_prompt(SYSTEM_PROMPT_COMPACT)),
                human])
            # ★ 走**生产同一条**路径：文本恢复 → 空回复重试 → 全部守卫。
            #   否则「模型自己选不对但护栏能兜住」的用例会被误判为失败，
            #   反过来「护栏顺序写错」也发现不了。
            resp = _recover_textual_tool_calls(resp)
            resp = AG._retry_if_empty(resp, human)
            resp = _recover_textual_tool_calls(resp)
            resp = AG.postprocess_response(resp, q)
            names = names_of(resp)
            hits.append(names[0] if names else "<无>")
        ok = sum(1 for h in hits if h == want)
        good += ok
        total += ROUNDS
        check(f"{q} → {want} ({ok}/{ROUNDS})", ok == ROUNDS, f"实际 {hits}")
    print(f"\n  模型工具选择：{good} / {total}")


# ============================================================
print("\n" + "=" * 66)
total = PASS + FAIL
if FAIL:
    print(f"通过 {PASS} / {total}   失败 {FAIL}")
    for n in FAILED:
        print(f"   · {n}")
    sys.exit(1)
print(f"通过 {PASS} / {total}")
sys.exit(0)
