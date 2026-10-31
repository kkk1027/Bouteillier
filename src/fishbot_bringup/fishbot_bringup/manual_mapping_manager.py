#!/usr/bin/env python3
"""手动建图会话管理节点（Cartographer 2D）。

职责边界：
  - 本节点**不发布任何 /cmd_vel**，不驱动底盘；手动建图期间由操作者用
    Web 摇杆 / Qt 手动控制 / 键盘遥控驱动机器人。
  - 只管理"建图收尾 + 存图"：调用 Cartographer /finish_trajectory，
    等待最终 /map，调用 nav2 map_saver_cli 落盘 pgm/yaml，
    再调用 Cartographer /write_state 保存 .pbstream。

服务：
  /manual_mapping/save_map    custom_interfaces/srv/SaveMap
  /manual_mapping/discard_map std_srvs/srv/Trigger
  /manual_mapping/get_status  std_srvs/srv/Trigger（message 为 JSON）

状态话题：
  /manual_mapping/status      std_msgs/String（JSON，1 Hz）

设计参考（仅借鉴流程，不复制其自主探索逻辑）：
  /home/user/wheeltec_ros2/src/stemm_cartographer_exploration/
    stemm_cartographer_exploration/cartographer_manager.py
    stemm_cartographer_exploration/state_saver.py
"""

import json
import os
import re
import shutil
import subprocess
import tempfile
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from cartographer_ros_msgs.srv import FinishTrajectory, WriteState
from custom_interfaces.srv import SaveMap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

_SAFE_NAME = re.compile(r'[^0-9A-Za-z_.-]+')
_NAV_CONFLICT_NODES = {
    'amcl',
    'map_server',
    'controller_server',
    'planner_server',
    'bt_navigator',
    'behavior_server',
    'smoother_server',
    'velocity_smoother',
}


def find_workspace_root() -> Path:
    """定位 fishbot_ws 工作空间根目录。

    优先级：
      1. 环境变量 FISHBOT_WS
      2. AMENT_PREFIX_PATH 中以 install 结尾且同级存在 src 的目录
      3. 从本文件向上查找同时包含 src 与 install 的目录
      4. 常见默认路径 / 当前目录
    """
    env = os.environ.get('FISHBOT_WS', '').strip()
    if env:
        candidate = Path(os.path.expanduser(env)).resolve()
        if (candidate / 'src').is_dir():
            return candidate

    for entry in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if not entry:
            continue
        candidate = Path(entry).resolve()
        if candidate.name == 'install' and (candidate.parent / 'src').is_dir():
            return candidate.parent

    for parent in Path(__file__).resolve().parents:
        if (parent / 'src').is_dir() and (parent / 'install').is_dir():
            return parent

    for candidate in (
        Path.home() / 'bishe_ws' / 'bishe-main' / 'fishbot_ws',
        Path.cwd(),
    ):
        if (candidate / 'src').is_dir():
            return candidate

    return Path.cwd()


def default_map_dir() -> Path:
    """默认地图目录：<workspace>/src/fishbot_navigation2/maps。"""
    return find_workspace_root() / 'src' / 'fishbot_navigation2' / 'maps'


def sanitize_map_name(raw: str) -> str:
    """只保留文件名安全字符，防止路径穿越。"""
    return _SAFE_NAME.sub('_', str(raw).strip()).strip('._')


class ManualMappingManager(Node):
    """手动建图收尾与存图状态机。"""

    STATE_WAITING = 'waiting_for_map'
    STATE_MAPPING = 'mapping'
    STATE_SAVING = 'saving'
    STATE_SAVED = 'saved'
    STATE_DISCARDED = 'discarded'
    STATE_ERROR = 'error'

    def __init__(self) -> None:
        super().__init__('manual_mapping_manager')

        # ---------------- 参数 ----------------
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_dir', '')
        self.declare_parameter('map_name_prefix', 'manual_')
        self.declare_parameter('trajectory_id', 0)

        self.declare_parameter('save_map_service', '/manual_mapping/save_map')
        self.declare_parameter('discard_map_service',
                               '/manual_mapping/discard_map')
        self.declare_parameter('get_status_service',
                               '/manual_mapping/get_status')
        self.declare_parameter('status_topic', '/manual_mapping/status')
        self.declare_parameter('finish_trajectory_service',
                               '/finish_trajectory')
        self.declare_parameter('write_state_service', '/write_state')

        self.declare_parameter('final_map_timeout_sec', 20.0)
        self.declare_parameter('final_map_settle_sec', 3.0)
        self.declare_parameter('finish_service_timeout_sec', 10.0)
        self.declare_parameter('write_state_timeout_sec', 20.0)
        self.declare_parameter('service_wait_timeout_sec', 5.0)
        self.declare_parameter('map_saver_timeout_sec', 30.0)
        self.declare_parameter('save_total_timeout_sec', 90.0)
        self.declare_parameter('map_saver_mode', 'trinary')
        self.declare_parameter('occupied_thresh', 0.65)
        # free_thresh 必须 > (255-205)/255 = 0.19607，否则 map_saver 写出的
        # 未知像素(灰度 205) 会被 map_server 的 trinary 模式判成**空闲**。
        # 踩过的坑：用 0.25 时整张图 68% 的未知区全变成空闲，AMCL 只看到
        # 6.8% 的墙包围 93% 的空地，似然场几乎处处平坦，静止收敛要 78 秒
        # 且容易假收敛。0.196 是 nav2 map_saver/ROS 的官方默认值。
        self.declare_parameter('free_thresh', 0.196)
        self.declare_parameter('save_pbstream', True)
        self.declare_parameter('overwrite', False)
        self.declare_parameter('check_navigation_conflict', True)

        # 启动自检：只读统计，不发布任何运动指令
        self.declare_parameter('preflight_check', True)
        self.declare_parameter('preflight_delay_sec', 20.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu_corrected')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('filtered_odom_topic', '/odometry/filtered')

        self.map_topic = str(self.get_parameter('map_topic').value)
        self.map_dir_param = str(self.get_parameter('map_dir').value)
        self.map_name_prefix = str(
            self.get_parameter('map_name_prefix').value)
        self.trajectory_id = int(self.get_parameter('trajectory_id').value)

        self.final_map_timeout_sec = float(
            self.get_parameter('final_map_timeout_sec').value)
        self.final_map_settle_sec = max(
            0.0, float(self.get_parameter('final_map_settle_sec').value))
        self.finish_service_timeout_sec = float(
            self.get_parameter('finish_service_timeout_sec').value)
        self.write_state_timeout_sec = float(
            self.get_parameter('write_state_timeout_sec').value)
        self.service_wait_timeout_sec = float(
            self.get_parameter('service_wait_timeout_sec').value)
        self.map_saver_timeout_sec = float(
            self.get_parameter('map_saver_timeout_sec').value)
        self.save_total_timeout_sec = float(
            self.get_parameter('save_total_timeout_sec').value)
        self.map_saver_mode = str(
            self.get_parameter('map_saver_mode').value)
        self.occupied_thresh = float(
            self.get_parameter('occupied_thresh').value)
        self.free_thresh = float(self.get_parameter('free_thresh').value)
        self.save_pbstream_default = bool(
            self.get_parameter('save_pbstream').value)
        self.overwrite_default = bool(
            self.get_parameter('overwrite').value)

        # ---------------- 状态 ----------------
        self._lock = threading.RLock()
        self._map_cond = threading.Condition(self._lock)
        self._state = self.STATE_WAITING
        self._last_map = None
        self._map_revision = 0
        self._last_map_monotonic = 0.0
        self._last_error = ''
        self._saved_paths = {}
        self._save_result = {}
        self._save_thread = None
        self._shutting_down = False

        # 所有回调放进可重入组：save_map 服务回调会阻塞等待，期间
        # /map 订阅、服务客户端响应仍需要被执行器处理。
        self.cb_group = ReentrantCallbackGroup()

        # ---------------- /map 订阅 ----------------
        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, map_qos,
            callback_group=self.cb_group)

        # ---------------- 服务 ----------------
        self.create_service(
            SaveMap, str(self.get_parameter('save_map_service').value),
            self._save_map_callback, callback_group=self.cb_group)
        self.create_service(
            Trigger, str(self.get_parameter('discard_map_service').value),
            self._discard_map_callback, callback_group=self.cb_group)
        self.create_service(
            Trigger, str(self.get_parameter('get_status_service').value),
            self._get_status_callback, callback_group=self.cb_group)

        # ---------------- 状态话题 ----------------
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)
        self.create_timer(
            1.0, self._publish_status, callback_group=self.cb_group)

        # ---------------- Cartographer 服务客户端 ----------------
        self._finish_client = self.create_client(
            FinishTrajectory,
            str(self.get_parameter('finish_trajectory_service').value),
            callback_group=self.cb_group)
        self._write_state_client = self.create_client(
            WriteState,
            str(self.get_parameter('write_state_service').value),
            callback_group=self.cb_group)

        self._conflict_timer = None
        if bool(self.get_parameter('check_navigation_conflict').value):
            self._conflict_timer = self.create_timer(
                3.0, self._check_navigation_conflict,
                callback_group=self.cb_group)

        # ---------------- 启动自检（只读） ----------------
        self._sensor_counts = {
            'scan': 0, 'imu': 0, 'odom': 0, 'filtered_odom': 0}
        self._preflight_timer = None
        if bool(self.get_parameter('preflight_check').value):
            self._setup_preflight()

        self.get_logger().info(
            '手动建图管理节点已启动：等待 %s 第一帧地图...' % self.map_topic)
        self.get_logger().info(
            '存图服务: %s；丢弃服务: %s；状态服务: %s' % (
                self.get_parameter('save_map_service').value,
                self.get_parameter('discard_map_service').value,
                self.get_parameter('get_status_service').value))

    # ============================================================
    # /map 与状态
    # ============================================================
    def _map_callback(self, msg: OccupancyGrid) -> None:
        became_mapping = False
        with self._map_cond:
            self._last_map = msg
            self._map_revision += 1
            self._last_map_monotonic = time.monotonic()
            if self._state == self.STATE_WAITING:
                self._state = self.STATE_MAPPING
                became_mapping = True
            self._map_cond.notify_all()
        if became_mapping:
            self.get_logger().info(
                '收到第一帧 /map（%dx%d, %.3f m/px），进入 mapping 状态；'
                '可以开始手动遥控建图。' % (
                    int(msg.info.width), int(msg.info.height),
                    float(msg.info.resolution)))

    def _map_stats(self) -> dict:
        with self._lock:
            msg = self._last_map
        if msg is None:
            return {}
        occupied = free = unknown = 0
        for value in msg.data:
            if value < 0:
                unknown += 1
            elif value >= 65:
                occupied += 1
            elif value <= 25:
                free += 1
        return {
            'width': int(msg.info.width),
            'height': int(msg.info.height),
            'resolution': float(msg.info.resolution),
            'frame_id': msg.header.frame_id,
            'occupied_cells': occupied,
            'free_cells': free,
            'unknown_cells': unknown,
        }

    def _status_dict(self, include_stats: bool = False) -> dict:
        with self._lock:
            state = self._state
            revision = self._map_revision
            has_map = self._last_map is not None
            map_age = (
                round(time.monotonic() - self._last_map_monotonic, 2)
                if self._last_map_monotonic > 0.0 else None)
            error = self._last_error
            saved = dict(self._saved_paths)
        status = {
            'state': state,
            'map_received': has_map,
            'map_revision': revision,
            'last_map_age_sec': map_age,
            'last_error': error,
            'saved': saved,
        }
        if include_stats:
            status.update(self._map_stats())
        return status

    def _publish_status(self) -> None:
        if self._shutting_down:
            return
        self._status_pub.publish(String(
            data=json.dumps(self._status_dict(), ensure_ascii=False)))

    def _check_navigation_conflict(self) -> None:
        """一次性提示：建图与 AMCL/Nav2 同时发布会争抢 map->odom。"""
        if self._conflict_timer is not None:
            self._conflict_timer.cancel()
        try:
            names = {name.split('/')[-1] for name in self.get_node_names()}
        except Exception:  # noqa: BLE001 - 仅诊断用途
            return
        conflicts = sorted(_NAV_CONFLICT_NODES & names)
        if conflicts:
            self.get_logger().error(
                '检测到导航节点仍在运行: %s。Cartographer 与 AMCL 会同时发布 '
                'map->odom，建图前请先关闭导航（或关闭本检查 '
                'check_navigation_conflict:=false）。' % ', '.join(conflicts))

    # ============================================================
    # 启动自检（只读，不发布任何运动指令）
    # ============================================================
    def _setup_preflight(self) -> None:
        sensor_qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            lambda msg: self._count('scan', msg), sensor_qos,
            callback_group=self.cb_group)
        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value),
            lambda msg: self._count('imu', msg), sensor_qos,
            callback_group=self.cb_group)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            lambda msg: self._count('odom', msg), sensor_qos,
            callback_group=self.cb_group)
        self.create_subscription(
            Odometry, str(self.get_parameter('filtered_odom_topic').value),
            lambda msg: self._count('filtered_odom', msg), sensor_qos,
            callback_group=self.cb_group)

        self._tf_buffer = None
        self._tf_listener = None
        try:
            from tf2_ros import Buffer, TransformListener
            try:
                self._tf_buffer = Buffer(node=self)
            except TypeError:
                self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        except Exception as exc:  # noqa: BLE001 - 自检降级
            self.get_logger().warn('TF 自检不可用: %s' % exc)

        delay = max(1.0, float(self.get_parameter('preflight_delay_sec').value))
        self._preflight_timer = self.create_timer(
            delay, self._run_preflight, callback_group=self.cb_group)

    def _count(self, key: str, msg) -> None:
        del msg
        self._sensor_counts[key] = self._sensor_counts.get(key, 0) + 1

    def _tf_available(self, target: str, source: str):
        if self._tf_buffer is None:
            return None
        try:
            self._tf_buffer.lookup_transform(
                target, source, Time(), Duration(seconds=0.2))
            return True
        except Exception:  # noqa: BLE001 - 查不到就是 False
            return False

    def _run_preflight(self) -> None:
        if self._preflight_timer is not None:
            self._preflight_timer.cancel()
            self._preflight_timer = None
        if self._shutting_down:
            return

        counts = dict(self._sensor_counts)
        scan_topic = str(self.get_parameter('scan_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        filtered_topic = str(self.get_parameter('filtered_odom_topic').value)

        self.get_logger().info(
            '建图自检 —— %s: %d 帧 | %s: %d 帧 | %s: %d 帧 | %s: %d 帧' % (
                scan_topic, counts.get('scan', 0),
                imu_topic, counts.get('imu', 0),
                odom_topic, counts.get('odom', 0),
                filtered_topic, counts.get('filtered_odom', 0)))

        if counts.get('scan', 0) == 0:
            self.get_logger().error(
                '%s 无数据：雷达驱动未运行或串口桥接未就绪。排查顺序：'
                '1) ss -tlnp | grep 8889 是否有 tcp_server 监听；'
                '2) ss -tan | grep 8889 雷达转接板是否已连入；'
                '3) timeout 3 cat /tmp/tty_laser | wc -c 是否有字节流；'
                '4) pgrep -af ydlidar_node 进程是否存活（失败会自动退出）'
                % scan_topic)
        if counts.get('odom', 0) == 0:
            self.get_logger().error(
                '%s 无数据：检查 micro_ros_agent 与 ESP32 连接' % odom_topic)
        if counts.get('filtered_odom', 0) == 0:
            self.get_logger().error(
                '%s 无数据：EKF 未运行。若底盘由外部 launch 启动，请确认其中'
                '包含 imu_bias_remover + odom_covariance_injector + ekf_node；'
                '否则用 start_ekf:=true 启动本 launch 的 EKF'
                % filtered_topic)
        if counts.get('imu', 0) == 0:
            self.get_logger().warn(
                '%s 无数据：IMU 融合建图会退化为纯激光匹配。检查 /imu_raw 是否'
                '发布、imu_bias_remover 是否运行' % imu_topic)

        odom_tf = self._tf_available('odom', 'base_footprint')
        imu_tf = self._tf_available('base_link', 'imu_link')
        if odom_tf is False:
            self.get_logger().error(
                'TF odom -> base_footprint 不存在：Cartographer 无法初始化位姿。'
                '检查 odom2tf 是否收到 %s' % filtered_topic)
        if imu_tf is False:
            self.get_logger().error(
                'TF base_link -> imu_link 不存在：本 lua 的 tracking_frame 是 '
                'imu_link，Cartographer 会拒绝所有扫描。可设 publish_imu_tf:=true，'
                '或改用 fishbot_cartographer.lua（无 IMU 版）')
        if odom_tf is None or imu_tf is None:
            self.get_logger().warn('未启用 TF 自检（tf2_ros 不可用）')

        with self._lock:
            has_map = self._last_map is not None
        if not has_map:
            self.get_logger().warn(
                '%s 仍无第一帧地图：若上面的传感器都正常，说明 Cartographer '
                '还在等 TF 或首帧扫描' % self.map_topic)

    # ============================================================
    # 服务回调
    # ============================================================
    def _save_map_callback(self, request, response):
        with self._lock:
            state = self._state
            has_map = self._last_map is not None
            saved_paths = dict(self._saved_paths)

        if state == self.STATE_SAVING:
            response.success = False
            response.message = '已有存图任务在进行，请稍后查询状态'
            return response
        if state == self.STATE_SAVED:
            response.success = True
            response.message = '本次地图已保存，可直接导航使用'
            response.map_yaml = saved_paths.get('map_yaml', '')
            response.map_pgm = saved_paths.get('map_pgm', '')
            response.pbstream = saved_paths.get('pbstream', '')
            return response
        if state == self.STATE_DISCARDED:
            response.success = False
            response.message = '本次建图已被丢弃，请重新启动建图 launch'
            return response
        if state == self.STATE_ERROR:
            response.success = False
            response.message = '上次存图失败（%s），请重新启动建图 launch' % (
                self._last_error or 'unknown error')
            return response
        if not has_map:
            response.success = False
            response.message = ('尚未收到 %s，请确认雷达/Cartographer 已启动并'
                                '完成一次扫描' % self.map_topic)
            return response

        # ---------- 解析文件名与目录 ----------
        raw_name = str(request.name).strip()
        if raw_name:
            name = sanitize_map_name(raw_name)
            if not name:
                response.success = False
                response.message = '非法地图名: %r' % raw_name
                return response
        else:
            name = self.map_name_prefix + datetime.now().strftime(
                '%Y%m%d_%H%M%S')

        raw_dir = str(request.directory).strip()
        if raw_dir:
            map_dir = Path(os.path.expanduser(raw_dir)).resolve()
        elif self.map_dir_param.strip():
            map_dir = Path(
                os.path.expanduser(self.map_dir_param.strip())).resolve()
        else:
            map_dir = default_map_dir().resolve()

        save_pbstream = bool(request.save_pbstream) and \
            self.save_pbstream_default
        overwrite = bool(request.overwrite) or self.overwrite_default

        final_yaml = map_dir / (name + '.yaml')
        final_pgm = map_dir / (name + '.pgm')
        final_pbstream = map_dir / (name + '.pbstream')

        if not overwrite:
            existing = [p for p in (
                final_yaml, final_pgm,
                final_pbstream if save_pbstream else None,
            ) if p is not None and p.exists()]
            if existing:
                response.success = False
                response.message = (
                    '目标文件已存在（%s）；如需覆盖请传 overwrite: true'
                    % ', '.join(str(p) for p in existing))
                return response

        try:
            map_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            response.success = False
            response.message = '无法创建地图目录 %s: %s' % (map_dir, exc)
            return response

        # ---------- 启动存图线程 ----------
        done = threading.Event()
        with self._lock:
            self._state = self.STATE_SAVING
            self._save_result = {}

        self._save_thread = threading.Thread(
            target=self._save_worker,
            args=(name, map_dir, overwrite, save_pbstream, done),
            name='manual-mapping-save',
            daemon=True)
        self._save_thread.start()

        if not done.wait(self.save_total_timeout_sec):
            with self._lock:
                self._state = self.STATE_ERROR
                self._last_error = '存图总超时 (%.1fs)' % (
                    self.save_total_timeout_sec)
            response.success = False
            response.message = self._last_error
            return response

        with self._lock:
            result = dict(self._save_result)
            if result.get('success'):
                self._state = self.STATE_SAVED
                self._saved_paths = {
                    'map_yaml': result.get('map_yaml', ''),
                    'map_pgm': result.get('map_pgm', ''),
                    'pbstream': result.get('pbstream', ''),
                }
                self._last_error = ''
            else:
                self._state = self.STATE_ERROR
                self._last_error = result.get('message', 'unknown error')

        response.success = bool(result.get('success'))
        response.message = str(result.get('message', ''))
        response.map_yaml = str(result.get('map_yaml', ''))
        response.map_pgm = str(result.get('map_pgm', ''))
        response.pbstream = str(result.get('pbstream', ''))
        return response

    def _discard_map_callback(self, request, response):
        del request
        with self._lock:
            state = self._state
            has_map = self._last_map is not None
        if state == self.STATE_SAVING:
            response.success = False
            response.message = '存图进行中，不能丢弃'
            return response
        if state == self.STATE_SAVED:
            response.success = False
            response.message = '地图已保存，不能丢弃；如需新地图请重新启动 launch'
            return response
        if state == self.STATE_DISCARDED:
            response.success = True
            response.message = '本次建图已处于丢弃状态'
            return response
        if not has_map:
            response.success = False
            response.message = '尚未收到 /map，本次建图尚未开始，无需丢弃'
            return response

        # finish_trajectory 可能包含 wait_for_service，放到工作线程里执行，
        # 避免占用执行器线程导致客户端响应无法处理。
        outcome = {}
        done = threading.Event()

        def _worker():
            outcome['ok'], outcome['detail'] = self._call_finish_trajectory()
            done.set()

        threading.Thread(
            target=_worker, name='manual-mapping-discard',
            daemon=True).start()
        done.wait(self.finish_service_timeout_sec + 2.0)
        ok = bool(outcome.get('ok', False))
        detail = outcome.get('detail', 'finish_trajectory 超时')

        with self._lock:
            self._state = self.STATE_DISCARDED
            self._last_error = '' if ok else detail
        if ok:
            response.success = True
            response.message = '已结束 Cartographer 轨迹，未保存任何文件'
        else:
            response.success = True
            response.message = (
                '已标记丢弃，但 finish_trajectory 失败: %s' % detail)
        self.get_logger().warn(response.message)
        return response

    def _get_status_callback(self, request, response):
        del request
        status = self._status_dict(include_stats=True)
        response.success = bool(status.get('map_received'))
        response.message = json.dumps(status, ensure_ascii=False)
        return response

    # ============================================================
    # 存图线程
    # ============================================================
    def _save_worker(self, name: str, map_dir: Path, overwrite: bool,
                     save_pbstream: bool, done: threading.Event) -> None:
        result = {
            'success': False,
            'message': '',
            'map_yaml': '',
            'map_pgm': '',
            'pbstream': '',
        }
        notes = []
        temp_dir = None
        try:
            with self._lock:
                finish_revision = self._map_revision

            finish_ok, finish_detail = self._call_finish_trajectory()
            if finish_ok:
                got_final = self._wait_for_new_map(
                    finish_revision, self.final_map_timeout_sec)
                if got_final:
                    if self.final_map_settle_sec > 0.0:
                        time.sleep(self.final_map_settle_sec)
                    notes.append('已等待最终 /map')
                else:
                    notes.append(
                        'finish 后 %.1fs 内未收到新 /map，使用缓存地图'
                        % self.final_map_timeout_sec)
                    self.get_logger().warn(notes[-1])
            else:
                notes.append(
                    'finish_trajectory 失败（%s），使用缓存 /map'
                    % finish_detail)
                self.get_logger().error(notes[-1])

            temp_dir, temp_stem = self._run_map_saver(name, map_dir)
            notes.append('map_saver_cli 成功')

            temp_pbstream = None
            if save_pbstream:
                try:
                    temp_pbstream = self._write_pbstream(temp_stem)
                    notes.append('write_state 成功')
                except Exception as exc:  # noqa: BLE001 - pbstream 可降级
                    notes.append('write_state 失败（仅保存 pgm/yaml）: %s' % exc)
                    self.get_logger().warn(notes[-1])
            else:
                notes.append('按请求跳过 pbstream')

            # 目标文件在进入 worker 前已检查；这里再做一次覆盖保护，
            # 防止并发/外部进程在此期间创建同名文件。
            final_yaml = map_dir / (name + '.yaml')
            final_pgm = map_dir / (name + '.pgm')
            final_pbstream = map_dir / (name + '.pbstream')
            if not overwrite:
                for path in (final_yaml, final_pgm):
                    if path.exists():
                        raise RuntimeError('目标文件已存在: %s' % path)
                if temp_pbstream is not None and final_pbstream.exists():
                    raise RuntimeError('目标文件已存在: %s' % final_pbstream)

            temp_pgm = Path(str(temp_stem) + '.pgm')
            temp_yaml = Path(str(temp_stem) + '.yaml')

            # 原子替换：yaml 最后写入，作为"地图完整"的提交标记。
            # 中途失败则回滚已落盘的文件，避免留下半套地图。
            moved = []
            try:
                os.replace(temp_pgm, final_pgm)
                moved.append(final_pgm)
                if temp_pbstream is not None:
                    os.replace(temp_pbstream, final_pbstream)
                    moved.append(final_pbstream)
                os.replace(temp_yaml, final_yaml)
            except Exception:
                for path in moved:
                    try:
                        path.unlink()
                    except OSError:
                        pass
                raise

            result.update({
                'success': True,
                'map_yaml': str(final_yaml),
                'map_pgm': str(final_pgm),
                'pbstream': str(final_pbstream) if temp_pbstream else '',
                'message': '；'.join(notes),
            })
            self.get_logger().info(
                '地图保存成功: %s, %s%s' % (
                    final_yaml, final_pgm,
                    ', %s' % final_pbstream if temp_pbstream else ''))
        except Exception as exc:  # noqa: BLE001 - 服务需要把失败原因带回
            result['message'] = '；'.join(notes + ['保存失败: %s' % exc])
            self.get_logger().error(result['message'])
        finally:
            if temp_dir is not None:
                shutil.rmtree(temp_dir, ignore_errors=True)
            with self._lock:
                self._save_result = result
            done.set()

    def _wait_for_new_map(self, baseline_revision: int, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        with self._map_cond:
            while (self._map_revision <= baseline_revision and
                   time.monotonic() < deadline):
                remaining = deadline - time.monotonic()
                self._map_cond.wait(max(0.0, min(0.2, remaining)))
            return self._map_revision > baseline_revision

    def _run_map_saver(self, name: str, map_dir: Path):
        """在目标目录内建临时子目录，调用 map_saver_cli 写出 pgm/yaml。"""
        temp_dir = Path(tempfile.mkdtemp(
            prefix='.%s.' % name, dir=str(map_dir)))
        temp_stem = temp_dir / name
        cmd = [
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-t', self.map_topic,
            '-f', str(temp_stem),
            '--mode', self.map_saver_mode,
            '--occ', str(self.occupied_thresh),
            '--free', str(self.free_thresh),
            # map_saver_cli 的 --ros-args 必须放在最后
            '--ros-args', '-p', 'map_subscribe_transient_local:=true',
        ]
        try:
            proc = subprocess.run(
                cmd, capture_output=True, text=True,
                timeout=self.map_saver_timeout_sec)
        except FileNotFoundError as exc:
            raise RuntimeError('找不到 ros2 命令: %s' % exc)
        except subprocess.TimeoutExpired:
            raise RuntimeError(
                'map_saver_cli 超时 (%.1fs)' % self.map_saver_timeout_sec)

        if proc.returncode != 0:
            stderr = (proc.stderr or '').strip().splitlines()
            tail = ' | '.join(stderr[-4:]) if stderr else '无 stderr'
            raise RuntimeError(
                'map_saver_cli 失败 (rc=%d): %s' % (proc.returncode, tail))

        pgm_path = Path(str(temp_stem) + '.pgm')
        yaml_path = Path(str(temp_stem) + '.yaml')
        if not pgm_path.is_file() or not yaml_path.is_file():
            raise RuntimeError(
                'map_saver_cli 返回成功但缺少输出文件: %s / %s' % (
                    pgm_path, yaml_path))

        # map_saver_cli 会把 yaml 的 image 字段写成调用时传入的路径，
        # 而这里传入的是临时目录，之后会被删除。必须改写成同目录下的
        # 文件名，否则地图加载器会指向不存在的绝对路径。
        self._normalize_yaml_image(yaml_path, pgm_path.name)
        return temp_dir, temp_stem

    @staticmethod
    def _normalize_yaml_image(yaml_path: Path, pgm_name: str) -> None:
        try:
            lines = yaml_path.read_text(encoding='utf-8').splitlines()
        except OSError as exc:
            raise RuntimeError('读取 map_saver 生成的 yaml 失败: %s' % exc)

        replaced = False
        normalized = []
        for line in lines:
            if line.startswith('image:'):
                normalized.append('image: %s' % pgm_name)
                replaced = True
            else:
                normalized.append(line)
        if not replaced:
            raise RuntimeError(
                'map_saver 生成的 yaml 缺少 image 字段: %s' % yaml_path)
        yaml_path.write_text('\n'.join(normalized) + '\n', encoding='utf-8')

    def _write_pbstream(self, temp_stem: Path) -> Path:
        if not self._write_state_client.wait_for_service(
                timeout_sec=self.service_wait_timeout_sec):
            raise RuntimeError(
                'Cartographer %s 不可用，无法保存 pbstream' %
                self._write_state_client.srv_name)
        request = WriteState.Request()
        request.filename = str(temp_stem) + '.pbstream'
        request.include_unfinished_submaps = False
        response, error = self._call_client(
            self._write_state_client, request, self.write_state_timeout_sec)
        if response is None:
            raise RuntimeError(error)
        if int(response.status.code) != 0:
            raise RuntimeError(
                'write_state 返回 code=%d: %s' % (
                    int(response.status.code), response.status.message))
        path = Path(request.filename)
        if not path.is_file() or path.stat().st_size <= 0:
            raise RuntimeError('write_state 未生成有效文件: %s' % path)
        return path

    def _call_finish_trajectory(self):
        if not self._finish_client.wait_for_service(
                timeout_sec=self.service_wait_timeout_sec):
            return False, '服务 %s 不可用' % self._finish_client.srv_name
        request = FinishTrajectory.Request()
        request.trajectory_id = self.trajectory_id
        response, error = self._call_client(
            self._finish_client, request, self.finish_service_timeout_sec)
        if response is None:
            return False, error
        if int(response.status.code) != 0:
            return False, 'code=%d: %s' % (
                int(response.status.code), response.status.message)
        self.get_logger().info(
            'finish_trajectory 已受理 (trajectory_id=%d)' % self.trajectory_id)
        return True, 'ok'

    def _call_client(self, client, request, timeout_sec: float):
        """异步调用 + Event 等待，避免阻塞执行器线程。"""
        event = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(timeout_sec):
            return None, '服务 %s 调用超时 (%.1fs)' % (
                client.srv_name, timeout_sec)
        try:
            return future.result(), ''
        except Exception as exc:  # noqa: BLE001
            return None, '服务 %s 调用异常: %s' % (client.srv_name, exc)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManualMappingManager()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._shutting_down = True  # noqa: SLF001 - 进程退出标记
        save_thread = node._save_thread  # noqa: SLF001
        if save_thread is not None and save_thread.is_alive():
            save_thread.join(timeout=1.0)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
