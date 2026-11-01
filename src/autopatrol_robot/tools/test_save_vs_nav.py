#!/usr/bin/env python3
"""
tools/test_save_vs_nav.py
「记住这里是X」vs「去X」的工具选择纠错测试（2026-11-01）

事故背景（用户提供的 agent_node 原始日志）：
    [0] Human : 记住这里是书房
    [1] AI    : save_current_location(name='书房')   ✓ 书房 = (1.90, 0.35)
    [4] Human : 回起点
    [5] AI    : return_to_start                      ✓
    [8] Human : 去书房                               ← 纯导航指令
    [9] AI    : save_current_location(name='书房')   ✗ 只吐了这一个工具
    [10]Tool  : ✅ 已更新 书房 的位置                 ← 数据被写坏
    [11]AI    : 请把车开到真正的"书房"…
    书房从 (1.90, 0.348, 2.562) 被改写成 (-0.0013, -0.0441, 0.409)

为什么旧防线没拦住：
    · _guard_nav_target 只盯 return_to_start；
    · _dedupe_exclusive_tool_calls 里「同轮出现导航类就丢 save」的前提
      是**存在**导航类工具，本轮模型只给了 save，`if not nav_calls: return`
      直接放行。

双层修复：
    A. agent_graph._guard_save_intent —— 上游代码层纠错，把 save 改成 nav
    B. loc_tools.save_current_location —— 工具本身拒绝可疑覆盖

本测试全部离线：LocationStore 指向临时文件，不碰真实 patrol_config.yaml，
不发任何 ROS 指令，不启动节点。
"""

import os
import sys
import tempfile
import traceback

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
    print(f"\n{'=' * 62}\n{title}\n{'=' * 62}")


# ============================================================
# 0. 准备：把 LocationStore 指到临时文件
# ============================================================
import autopatrol_robot.location_store as ls_mod

_tmpdir = tempfile.mkdtemp(prefix="fishbot_test_savenav_")
_TMP_YAML = os.path.join(_tmpdir, "patrol_config.yaml")

with open(_TMP_YAML, "w") as f:
    f.write("""
patrol_node:
  ros__parameters:
    target_points: [0.0, 0.0, 0.0, 1.5, 0.5, 0.0, 2.0, 1.0, 0.0, 2.5, 1.5, 0.0]
    custom_points: {}
""")

# 覆盖单例，让所有 store 都指向临时文件
_orig_init = ls_mod.LocationStore.__init__


def _patched_init(self):
    _orig_init(self)
    self._yaml_path = _TMP_YAML


ls_mod.LocationStore.__init__ = _patched_init
ls_mod._store = None

from autopatrol_robot.location_store import get_location_store          # noqa: E402
from autopatrol_robot.agent_graph import (                              # noqa: E402
    _extract_destination,
    _resolve_destination,
    _guard_save_intent,
    _guard_nav_target,
    _dedupe_exclusive_tool_calls,
    _SAVE_INTENT_WORDS,
)

store = get_location_store()


class FakeResp:
    """最小 AIMessage 替身：只需要 tool_calls + content。"""

    def __init__(self, calls, content=""):
        self.tool_calls = [{"name": n, "args": a, "id": f"id_{i}"}
                           for i, (n, a) in enumerate(calls)]
        self.content = content

    def model_copy(self, update=None):
        r = FakeResp([], self.content)
        r.tool_calls = (update or {}).get("tool_calls", self.tool_calls)
        return r


def names_of(resp):
    return [tc["name"] for tc in resp.tool_calls]


# ============================================================
# 1. 目的地抽取
# ============================================================
section("1. _extract_destination —— 从口语句子里抠目的地")

cases = [
    ("去书房", "书房"),
    ("去厕所", "厕所"),
    ("去一下厨房", "厨房"),
    ("带我去书房", "书房"),
    ("导航到阳台", "阳台"),
    ("前往客厅", "客厅"),
    ("去书房吧", "书房"),
    ("去书房。", "书房"),
    ("去书房看看", "书房看看"),      # 尾巴靠 _resolve_destination 兜
    # ---- 下面这些都不该抠出东西 ----
    ("记住这里是书房", None),
    ("这里是书房", None),
    ("把这里记成书房", None),
    ("保存当前位置为书房", None),
    ("回起点", None),
    ("我去了", None),
]
for text, want in cases:
    got = _extract_destination(text)
    check(f"{text!r} → {want!r}", got == want, f"实际 {got!r}")


# ============================================================
# 2. 目的地解析（精确 + 宽松前缀）
# ============================================================
section("2. _resolve_destination —— 已知地点才认")

# 预置：书房在 (1.90, 0.35)，就是被写坏之前的真实坐标
store.save("书房", 1.898397936695498, 0.3471096256575249, 2.5620160619975265)
store.save("厕所", 3.40, -1.20, 0.0)

check("精确：书房", _resolve_destination("书房") == "书房")
check("精确：厕所", _resolve_destination("厕所") == "厕所")
check("宽松：书房看看 → 书房", _resolve_destination("书房看看") == "书房",
      f"实际 {_resolve_destination('书房看看')!r}")
check("宽松：厕所一趟 → 厕所", _resolve_destination("厕所一趟") == "厕所")
check("未知名：外星基地 → None", _resolve_destination("外星基地") is None)
check("回家类：起点 → None", _resolve_destination("起点") is None)
check("回家类：家里 → None", _resolve_destination("家里") is None)
check("空串 → None", _resolve_destination("") is None)


# ============================================================
# 3. _guard_save_intent —— 本次事故的核心修复
# ============================================================
section("3. _guard_save_intent —— 去X 被误判成 save_current_location")

# --- 3.1 复现事故：必须被改成导航 ---
r = FakeResp([("save_current_location", {"name": "书房"})])
out = _guard_save_intent(r, "去书房")
check("【事故复现】去书房 + save → 变成导航",
      names_of(out) == ["navigate_to_saved_location"], f"实际 {names_of(out)}")
check("【事故复现】目的地正确",
      out.tool_calls[0]["args"].get("name") == "书房",
      f"实际 {out.tool_calls[0]['args']}")

# --- 3.2 带尾巴的口语 ---
r = FakeResp([("save_current_location", {"name": "书房"})])
out = _guard_save_intent(r, "去书房看看")
check("去书房看看 + save → 导航", names_of(out) == ["navigate_to_saved_location"])

r = FakeResp([("save_current_location", {"name": "厕所"})])
out = _guard_save_intent(r, "带我去厕所")
check("带我去厕所 + save → 导航", names_of(out) == ["navigate_to_saved_location"])

# --- 3.3 真的在保存：不能干预 ---
save_texts = [
    "记住这里是书房",
    "这里是书房",
    "把这里记成书房",
    "记作书房",
    "保存当前位置为书房",
    "这地方叫书房",
    "设置成书房",
]
for text in save_texts:
    r = FakeResp([("save_current_location", {"name": "书房"})])
    out = _guard_save_intent(r, text)
    check(f"{text!r} 保留 save", names_of(out) == ["save_current_location"],
          f"实际 {names_of(out)}")

# --- 3.4 用户没说保存，也没说去某地（闲聊）：保守放行 ---
r = FakeResp([("save_current_location", {"name": "书房"})])
out = _guard_save_intent(r, "嗯")
check("闲聊时保守放行", names_of(out) == ["save_current_location"])

# --- 3.5 目的地查不到：保守放行 ---
r = FakeResp([("save_current_location", {"name": "外星基地"})])
out = _guard_save_intent(r, "去外星基地")
check("目的地查不到 → 放行", names_of(out) == ["save_current_location"])

# --- 3.6 本轮已有导航工具：交给 dedupe，不在这层重复加 ---
r = FakeResp([("save_current_location", {"name": "书房"}),
              ("navigate_to_saved_location", {"name": "书房"})])
out = _guard_save_intent(r, "去书房")
check("已有导航时不重复处理",
      names_of(out) == ["save_current_location", "navigate_to_saved_location"],
      f"实际 {names_of(out)}")

# --- 3.7 没有 save：完全不动 ---
r = FakeResp([("navigate_to_saved_location", {"name": "书房"})])
out = _guard_save_intent(r, "去书房")
check("模型选对时不干预", names_of(out) == ["navigate_to_saved_location"])

# --- 3.8 空 tool_calls ---
r = FakeResp([], "好的")
check("纯文本回复不干预", names_of(_guard_save_intent(r, "去书房")) == [])


# ============================================================
# 4. 与既有两层的协作顺序
# ============================================================
section("4. 三层防线组合（nav_target → save_intent → dedupe）")


def pipeline(resp, text):
    resp = _guard_nav_target(resp, text)
    resp = _guard_save_intent(resp, text)
    resp = _dedupe_exclusive_tool_calls(resp)
    return resp


# 4.1 事故场景（本轮只有 save）
r = pipeline(FakeResp([("save_current_location", {"name": "书房"})]), "去书房")
check("事故场景 → 只剩导航",
      names_of(r) == ["navigate_to_saved_location"], f"实际 {names_of(r)}")

# 4.2 旧故障场景（save + return_to_start + navigate）
r = pipeline(FakeResp([
    ("save_current_location", {"name": "厕所"}),
    ("return_to_start", {}),
    ("navigate_to_saved_location", {"name": "厕所"}),
]), "去厕所")
check("旧故障场景 → 只剩导航",
      names_of(r) == ["navigate_to_saved_location"], f"实际 {names_of(r)}")

# 4.3 save + return_to_start，用户说去书房
r = pipeline(FakeResp([
    ("save_current_location", {"name": "书房"}),
    ("return_to_start", {}),
]), "去书房")
check("save+回起点 → 只剩导航",
      names_of(r) == ["navigate_to_saved_location"], f"实际 {names_of(r)}")

# 4.4 真保存 + 顺手调表情：表情要保留
r = pipeline(FakeResp([
    ("save_current_location", {"name": "书房"}),
    ("set_butler_mood", {"mood": "approve"}),
]), "记住这里是书房")
check("真保存 + 表情 → 都保留",
      sorted(names_of(r)) == ["save_current_location", "set_butler_mood"],
      f"实际 {names_of(r)}")

# 4.5 纯导航 + 表情
r = pipeline(FakeResp([
    ("navigate_to_saved_location", {"name": "书房"}),
    ("set_butler_mood", {"mood": "approve"}),
]), "去书房")
check("导航 + 表情 → 都保留",
      sorted(names_of(r)) == ["navigate_to_saved_location", "set_butler_mood"],
      f"实际 {names_of(r)}")


# ============================================================
# 5. save_current_location 的覆盖保护（第二道独立防线）
# ============================================================
section("5. save_current_location —— 拒绝可疑覆盖")

import autopatrol_robot.tools.loc_tools as lt                    # noqa: E402
import autopatrol_robot.navigation_service as ns_mod                # noqa: E402


class FakeNavService:
    """只提供 get_current_position，坐标由测试指定。"""

    def __init__(self):
        self.pos = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def get_current_position(self, detailed=False):
        return dict(self.pos), ""


_nav = FakeNavService()
# ★ 注意：loc_tools 里是**函数内部** import，直接打 lt 上的名字无效，
#   必须打在 navigation_service 模块上。
ns_mod.get_global_nav_service = lambda: _nav

save_tool = lt.save_current_location

# 5.1 事故场景：书房已在 (1.90, 0.35)，车停在原点，调 save → 必须拒绝
_nav.pos = {"x": -0.0013248666799859025, "y": -0.04406587346286371, "yaw": 0.4088}
before = store.get_coords("书房")
msg = save_tool.invoke({"name": "书房"})
after = store.get_coords("书房")

check("可疑覆盖被拒绝（消息含 ⛔）", "⛔" in msg, f"实际 {msg[:60]!r}")
check("书房坐标未被改写", after == before,
      f"之前 {before} → 之后 {after}")
check("拒绝消息说明原坐标", f"{before[0]:.2f}" in msg, f"实际 {msg[:120]!r}")

# 5.2 车真的开到书房 → 允许覆盖
_nav.pos = {"x": 1.90, "y": 0.35, "yaw": 2.56}
msg = save_tool.invoke({"name": "书房"})
after2 = store.get_coords("书房")
check("正常覆盖被接受", "✅" in msg, f"实际 {msg[:60]!r}")
check("坐标已更新", after2 is not None and abs(after2[0] - 1.90) < 0.01,
      f"实际 {after2}")

# 5.3 新增地点在原点附近 → 允许新增，但要带警告
_nav.pos = {"x": 0.05, "y": -0.02, "yaw": 0.0}
msg = save_tool.invoke({"name": "阳台"})
check("新增（非覆盖）被接受", "已保存" in msg, f"实际 {msg[:60]!r}")
check("新增带原点警告", "离地图原点" in msg, f"实际 {msg[:80]!r}")

# 5.4 已有地点本来就在原点附近，覆盖到原点附近 → 放行（无损失）
store.save("原位点", 0.05, 0.02, 0.0)
_nav.pos = {"x": -0.03, "y": 0.01, "yaw": 0.0}
msg = save_tool.invoke({"name": "原位点"})
check("原点↔原点覆盖放行", "✅" in msg, f"实际 {msg[:60]!r}")

# 5.5 覆盖到非原点位置 → 正常放行
_nav.pos = {"x": 2.80, "y": -1.50, "yaw": 1.2}
msg = save_tool.invoke({"name": "书房"})
after3 = store.get_coords("书房")
check("远离原点覆盖放行", "✅" in msg, f"实际 {msg[:60]!r}")
check("坐标确实变了", after3 is not None and abs(after3[0] - 2.80) < 0.01,
      f"实际 {after3}")


# ============================================================
# 6. 保存词表自检
# ============================================================
section("6. _SAVE_INTENT_WORDS 覆盖度")

for w in ("记住", "这里是", "记成", "保存", "设为"):
    check(f"词表含 {w!r}", w in _SAVE_INTENT_WORDS)

# 「去」类词绝不能被当成保存意图
for bad in ("去", "带我去", "导航到", "前往", "回起点"):
    hit = any(bad == w or (len(w) > 1 and bad.startswith(w))
              for w in _SAVE_INTENT_WORDS)
    check(f"词表不含导航词 {bad!r}", not hit)


# ============================================================
# 结果
# ============================================================
print("\n" + "=" * 62)
total = PASS + FAIL
if FAIL:
    print(f"通过 {PASS} / {total}   失败 {FAIL}")
    for n in FAILED:
        print(f"   · {n}")
    sys.exit(1)
print(f"通过 {PASS} / {total}")
sys.exit(0)
