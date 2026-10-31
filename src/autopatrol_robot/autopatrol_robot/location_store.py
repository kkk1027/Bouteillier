"""
location_store.py
====================================================
patrol_config.yaml 单文件存储管理器。

统一管理预定义航点和自定义地点，patrol_config.yaml 是唯一真相源。

YAML 结构:
  patrol_node:
    ros__parameters:
      initial_point: [0.0, 0.0, 3.14]
      target_points: [x,y,yaw, x,y,yaw, ...]   # 扁平列表
      custom_points:                            # 自定义地点元数据
        书房: [1.5, 3.2, 0.0]

线程安全: threading.Lock + fcntl.flock 双重保护。
"""

import os
import yaml
import fcntl
import threading
from ament_index_python.packages import get_package_share_directory

# 预定义航点数量（前 N 个三元组不会被 clear_all 删除）
NUM_PREDEFINED = 4


class LocationStore:
    """patrol_config.yaml 单文件存储管理器。"""

    def __init__(self):
        self._file_lock = threading.Lock()
        self._yaml_path = os.path.join(
            get_package_share_directory('autopatrol_robot'),
            'config', 'patrol_config.yaml'
        )

    def _read(self) -> dict:
        """读取完整 YAML 配置（共享锁）"""
        with open(self._yaml_path, 'r') as f:
            fcntl.flock(f, fcntl.LOCK_SH)
            data = yaml.safe_load(f)
            fcntl.flock(f, fcntl.LOCK_UN)
        return data

    def _write(self, data: dict):
        """写入完整 YAML 配置（原子：先写临时文件再 rename）"""
        tmp_path = self._yaml_path + '.tmp'
        with open(tmp_path, 'w') as f:
            fcntl.flock(f, fcntl.LOCK_EX)
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
            fcntl.flock(f, fcntl.LOCK_UN)
        os.replace(tmp_path, self._yaml_path)  # 原子操作

    # ----------------------------------------------------------
    # 自定义地点 CRUD
    # ----------------------------------------------------------

    def save(self, name: str, x: float, y: float, yaw: float) -> str:
        """
        保存自定义地点。

        新增: 追加到 target_points 末尾 + 记录到 custom_points。
        更新: 原地替换坐标。
        """
        with self._file_lock:
            data = self._read()
            params = data['patrol_node']['ros__parameters']
            tp = params.setdefault('target_points', [])
            cp = params.setdefault('custom_points', {})

            if name in cp:
                # 更新已有地点 —— 在 target_points 中找到旧坐标并替换
                old = cp[name]
                for i in range(0, len(tp), 3):
                    if (abs(tp[i] - old[0]) < 0.001
                            and abs(tp[i + 1] - old[1]) < 0.001
                            and abs(tp[i + 2] - old[2]) < 0.001):
                        tp[i], tp[i + 1], tp[i + 2] = x, y, yaw
                        break
                cp[name] = [x, y, yaw]
                self._write(data)
                return f"✅ 已更新 {name} 的位置"

            # 新增地点
            tp.extend([x, y, yaw])
            cp[name] = [x, y, yaw]
            self._write(data)
            return f"✅ 已保存 {name}"

    def delete(self, name: str) -> str:
        """删除自定义地点（从 target_points 和 custom_points 中移除）。"""
        with self._file_lock:
            data = self._read()
            params = data['patrol_node']['ros__parameters']
            cp = params.get('custom_points', {})
            tp = params.get('target_points', [])

            if name not in cp:
                return f"❌ 未找到地点 {name}"

            coords = cp[name]
            # 从 target_points 中移除
            idx_to_remove = None
            for i in range(0, len(tp), 3):
                if (abs(tp[i] - coords[0]) < 0.001
                        and abs(tp[i + 1] - coords[1]) < 0.001
                        and abs(tp[i + 2] - coords[2]) < 0.001):
                    idx_to_remove = i
                    break

            if idx_to_remove is not None:
                del tp[idx_to_remove:idx_to_remove + 3]
            del cp[name]
            self._write(data)
            return f"✅ 已删除 {name}"

    def find_index(self, name: str) -> "int | None":
        """
        查找地点在 target_points 中对应的 1-based 索引。

        如果坐标不存在于 target_points（如刚删除后），返回 None。
        """
        data = self._read()
        params = data['patrol_node']['ros__parameters']
        cp = params.get('custom_points', {})
        if name not in cp:
            return None
        coords = cp[name]
        tp = params.get('target_points', [])
        for i in range(0, len(tp), 3):
            if (abs(tp[i] - coords[0]) < 0.001
                    and abs(tp[i + 1] - coords[1]) < 0.001
                    and abs(tp[i + 2] - coords[2]) < 0.001):
                return i // 3 + 1  # 1-based
        return None

    def get_coords(self, name: str) -> "tuple | None":
        """获取自定义地点的坐标 (x, y, yaw)。"""
        data = self._read()
        cp = data['patrol_node']['ros__parameters'].get('custom_points', {})
        if name not in cp:
            return None
        c = cp[name]
        return (c[0], c[1], c[2])

    def get_predefined_coords(self, index_1based: int) -> "tuple | None":
        """
        获取预定义航点的坐标（1-based 索引）。

        index=1 → 第一个预定义航点（起点），index=2 → 第二个，以此类推。
        """
        if index_1based < 1 or index_1based > NUM_PREDEFINED:
            return None
        data = self._read()
        tp = data['patrol_node']['ros__parameters'].get('target_points', [])
        i = (index_1based - 1) * 3
        if i + 2 < len(tp):
            return (tp[i], tp[i + 1], tp[i + 2])
        return None

    def list_all(self) -> dict:
        """列出所有自定义地点 {name: [x, y, yaw], ...}。"""
        data = self._read()
        return data['patrol_node']['ros__parameters'].get('custom_points', {})

    def clear_all(self) -> str:
        """清空所有自定义地点（不删除预定义前 4 个航点）。"""
        with self._file_lock:
            data = self._read()
            params = data['patrol_node']['ros__parameters']
            cp = params.get('custom_points', {})
            tp = params.get('target_points', [])

            count = len(cp)
            # 只保留前 NUM_PREDEFINED 个三元组
            del tp[NUM_PREDEFINED * 3:]
            params['custom_points'] = {}
            self._write(data)
            return f"✅ 已清空 {count} 个自定义地点"


# 全局单例
_store: LocationStore | None = None


def get_location_store() -> LocationStore:
    """获取全局 LocationStore 单例。"""
    global _store
    if _store is None:
        _store = LocationStore()
    return _store
