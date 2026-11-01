#!/usr/bin/env python3
"""
tools/test_nav2_ready_wait.py
patrol_node 等待 nav2 就绪的逻辑测试（2026-11-01）

事故背景
------------------------------------------------------------
用户说「去书房」，AI 回了「📍 正在前往 书房...」，但车一步不动。
现场证据：

  · patrol_node 进程确实起来了（target_indices:=5）
  · nav2 日志里**一条 Begin navigating 都没有**
  · patrol_node 的日志只有「发布导航节点状态: True」这一行，
    之后再无输出，直到用户手动 Ctrl-C
  · 同一秒 AMCL 报了：
      [amcl.rclcpp]: failed to send response to /amcl/get_state
                     (timeout): client will not receive response

根因在 nav2_simple_commander/robot_navigator.py 的
_waitForNodeToActivate：

    future = state_client.call_async(req)
    rclpy.spin_until_future_complete(self, future)   # ★ 没有 timeout
    if future.result() is not None:
        state = future.result().current_state.label
    time.sleep(2)

这不是重试循环 —— 只有一次 spin，且无超时。AMCL 忙时服务端响应
会被 RMW 丢弃，future 永不完成，于是**永久阻塞**。

关键点：这是**偶发**的。用户后续重试就成功了，所以不能靠"复现"
来验证，只能靠代码结构保证：每次等待都必须有上限。

本测试用一个「永不回应的假服务」精确复现这个场景，验证：
  ① 新实现遇到不响应的服务会在有界时间内放弃，不会挂死；
  ② 真的 active 时能正常通过；
  ③ 服务一直不存在时也不会挂死；
  ④ 不再发布默认初始位姿（否则会把 AMCL 重置到原点）。

运行：/usr/bin/python3 tools/test_nav2_ready_wait.py
需要 ROS 环境（只用 ROS_DOMAIN_ID 隔离，不碰真实 nav2）。
"""

import os
import sys
import time
import threading

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
    print(f"\n{'=' * 68}\n{title}\n{'=' * 68}")


# 隔离域，避免和真实导航栈撞上
os.environ["ROS_DOMAIN_ID"] = "97"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

import rclpy                                                          # noqa: E402
from rclpy.node import Node                                           # noqa: E402
from lifecycle_msgs.srv import GetState                               # noqa: E402


class FakePatrol(Node):
    """最小替身：只提供 create_client / destroy_client / get_logger
    和 wait_nav2_ready 需要的 initial_pose_received 属性。"""

    def __init__(self):
        super().__init__('fake_patrol_probe')
        self.initial_pose_received = False


rclpy.init()
node = FakePatrol()


# ============================================================
# 导入被测函数
# ============================================================
from autopatrol_robot.patrol_node import (                            # noqa: E402
    _get_service_state,
    wait_nav2_ready,
    NAV2_WAIT_PER_TRY_SEC,
    NAV2_WAIT_TOTAL_SEC,
)


# ============================================================
# 一、核心：不发响应的服务，查询必须有界返回
# ============================================================
section("一、_get_service_state 遇到不响应的服务")

# 场景 A：服务根本不存在
t0 = time.time()
state = _get_service_state(node, node.create_client(GetState, 'no_such_node/get_state'),
                           'no_such_node/get_state', per_try=1.0)
elapsed = time.time() - t0
check("服务不存在 → 返回 'unavailable'", state == 'unavailable', f"实际 {state!r}")
check("服务不存在 → 有界返回（<3s）", elapsed < 3.0, f"实际 {elapsed:.2f}s")

# 场景 B：服务存在但永不回应（模拟 amcl 响应被丢弃）
#
# 这是本次事故的精确复现：服务端把请求收下了，但响应没发出去。
# 原生 spin_until_future_complete 会永远等下去。
class SilentServer(Node):
    def __init__(self):
        super().__init__('silent_amcl')
        self.received = 0
        # 只创建服务，回调里什么都不返回 —— 请求收下，响应不发
        self.create_service(GetState, 'silent/get_state', self._cb)

    def _cb(self, request, response):
        self.received += 1
        # 故意不填充 response，也不 return —— 保持 silence
        # rclpy 要求回调返回 response，所以这里返回一个"空"对象后
        # 客户端仍会因为服务端处理超时而被 RMW 丢弃响应。
        time.sleep(30)          # 拖住服务端，模拟 AMCL 忙死
        return response


silent = SilentServer()

t0 = time.time()
cli = node.create_client(GetState, 'silent/get_state')
state = _get_service_state(node, cli, 'silent/get_state', per_try=1.5)
elapsed = time.time() - t0

check("【事故复现】服务不回应 → 'unavailable'（不是永久阻塞）",
      state == 'unavailable', f"实际 {state!r}")
check("【事故复现】有界返回（<6s，实际应≈1.5s）",
      elapsed < 6.0, f"实际 {elapsed:.2f}s")
check("【事故复现】耗时接近 per_try，说明真的在等而不是秒返回",
      elapsed >= 1.0, f"实际 {elapsed:.2f}s")


# ============================================================
# 二、真 active 的服务要能正常通过
# ============================================================
# ★ 为什么用**子进程**而不是同进程另起线程：
#   rclpy 的 SingleThreadedExecutor 在多线程里对同一个 context 调
#   spin_once 会崩在 qos_event.is_ready() 里（IndexError: wait set
#   index too big）。那是 rclpy 自身的限制，跟被测逻辑无关，但会把
#   测试搞成假阴性。子进程干净得多，也更接近真实部署。
# ============================================================
section("二、_get_service_state 对正常服务")

import subprocess                                                      # noqa: E402
import tempfile                                                        # noqa: E402
import textwrap                                                        # noqa: E402

_server_code = textwrap.dedent('''
    import os, sys
    os.environ["ROS_DOMAIN_ID"] = "97"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    from rclpy.node import Node
    from lifecycle_msgs.srv import GetState
    rclpy.init()
    n = Node("live_node")
    def cb(req, res):
        res.current_state.id = 3
        res.current_state.label = "active"
        return res
    n.create_service(GetState, "live/get_state", cb)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
''')
_srv_path = os.path.join(tempfile.mkdtemp(prefix='fishbot_live_srv_'), 'live_srv.py')
with open(_srv_path, 'w') as f:
    f.write(_server_code)

srv_proc = subprocess.Popen([sys.executable, _srv_path],
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
try:
    # 等服务端 advertise 出来
    time.sleep(4.0)
    rclpy.spin_once(node, timeout_sec=0.5)
    cli = node.create_client(GetState, 'live/get_state')
    state = _get_service_state(node, cli, 'live/get_state', per_try=6.0)
    check("active 服务 → 返回 'active'", state == 'active', f"实际 {state!r}")
finally:
    srv_proc.terminate()
    try:
        srv_proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        srv_proc.kill()
    os.unlink(_srv_path)


# ============================================================
# 三、wait_nav2_ready 整体超时保护
# ============================================================
section("三、wait_nav2_ready 总超时（nav2 根本不在的情况）")

# node 上没有任何 amcl/bt_navigator 服务，且 initial_pose_received=False
# → 应该在 total_timeout 附近放弃，而不是永久挂住
t0 = time.time()
ok = wait_nav2_ready(node, total_timeout=6.0)
elapsed = time.time() - t0

check("nav2 不在时返回 False（不是挂死）", ok is False, f"实际 {ok!r}")
check("总耗时受控（<20s）", elapsed < 20.0, f"实际 {elapsed:.2f}s")
check("确实等到了超时点（>=5s，说明不是立刻放弃）",
      elapsed >= 5.0, f"实际 {elapsed:.2f}s")


# ============================================================
# 四、不许发布默认初始位姿
# ============================================================
section("四、不再发布默认初始位姿（防止把 AMCL 重置到原点）")

import inspect                                                        # noqa: E402
import ast                                                            # noqa: E402
import autopatrol_robot.patrol_node as pn                             # noqa: E402


def calls_attr(func, attr_name):
    """函数体里是否**真的调用**了 obj.attr_name(...)。

    ★ 不能用字符串搜 —— 注释和 docstring 里提到这个函数名就误判。
      本次修复的注释里恰好就写了 `patrol.waitUntilNav2Active()`
      作为反例说明。用 AST 只看真的 Call 节点。
    """
    try:
        tree = ast.parse(inspect.getsource(func).strip())
    except (OSError, SyntaxError):
        return None
    for n in ast.walk(tree):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute):
            if n.func.attr == attr_name:
                return True
    return False


check("wait_nav2_ready 不调用 _setInitialPose",
      calls_attr(pn.wait_nav2_ready, '_setInitialPose') is False,
      "真的在调 _setInitialPose，会把 AMCL 重置到原点")
check("wait_nav2_ready 不调用 waitUntilNav2Active",
      calls_attr(pn.wait_nav2_ready, 'waitUntilNav2Active') is False,
      "还在用原生的无超时等待")
check("main 不再调用 waitUntilNav2Active",
      calls_attr(pn.main, 'waitUntilNav2Active') is False,
      "main 里仍在用原生等待")

src_main = inspect.getsource(pn.main)
check("main 使用 wait_nav2_ready", "wait_nav2_ready(patrol)" in src_main,
      "main 没有调用新实现")
check("main 在等待失败时提前返回", "放弃本次导航" in src_main,
      "失败时没有明确放弃")


# ============================================================
# 五、参数自检
# ============================================================
section("五、超时参数合理性")

check("单次超时 > 0", NAV2_WAIT_PER_TRY_SEC > 0, f"实际 {NAV2_WAIT_PER_TRY_SEC}")
check("单次超时 ≤ 10s（太长了用户等不起）",
      NAV2_WAIT_PER_TRY_SEC <= 10.0, f"实际 {NAV2_WAIT_PER_TRY_SEC}")
check("总超时 ≥ 单次超时", NAV2_WAIT_TOTAL_SEC >= NAV2_WAIT_PER_TRY_SEC,
      f"{NAV2_WAIT_TOTAL_SEC} vs {NAV2_WAIT_PER_TRY_SEC}")
check("总超时 ≤ 180s（超过就没意义了，用户会重试）",
      NAV2_WAIT_TOTAL_SEC <= 180.0, f"实际 {NAV2_WAIT_TOTAL_SEC}")


# ============================================================
# 清理
# ============================================================
section("清理")

try:
    silent.destroy_node()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print("  ✅ 探针节点已销毁")
except Exception as e:                                                # noqa: BLE001
    print(f"  ⚠️ 清理异常（不影响结论）: {e}")


print("\n" + "=" * 68)
total = PASS + FAIL
if FAIL:
    print(f"通过 {PASS} / {total}   失败 {FAIL}")
    for n in FAILED:
        print(f"   · {n}")
    sys.exit(1)
print(f"通过 {PASS} / {total}")
sys.exit(0)
