#!/usr/bin/env python3
"""
FishBot 地图保存工具

保存当前 Cartographer 或 Nav2 发布的 /map 为 .pgm + .yaml 文件。

使用方式：
    # 保存到默认路径
    ros2 run fishbot_bringup save_map

    # 保存到指定文件名
    ros2 run fishbot_bringup save_map --name my_room

    # 保存到指定目录
    ros2 run fishbot_bringup save_map --dir ~/maps

    # 完整参数
    ros2 run fishbot_bringup save_map --name office_2f --dir ~/chapt9_update/fishbot_ws/src/fishbot_navigation2/maps
"""

import argparse
import os
import subprocess
import sys


def get_default_map_dir():
    """获取默认地图保存目录"""
    # 优先使用环境变量
    fishbot_ws = os.environ.get(
        'FISHBOT_WS',
        os.path.expanduser('~/chapt9_update/fishbot_ws')
    )
    return os.path.join(fishbot_ws, 'src', 'fishbot_navigation2', 'maps')


def save_map(map_name: str, map_dir: str = None) -> bool:
    """
    调用 nav2_map_server 的 map_saver_cli 保存当前地图

    Args:
        map_name: 地图文件名（不含扩展名）
        map_dir: 保存目录，默认使用 fishbot_navigation2/maps/

    Returns:
        bool: 是否保存成功
    """
    if map_dir is None:
        map_dir = get_default_map_dir()

    # 确保目录存在
    os.makedirs(map_dir, exist_ok=True)

    # 生成完整路径
    map_file_path = os.path.join(map_dir, map_name)
    map_pgm_path = map_file_path + '.pgm'
    map_yaml_path = map_file_path + '.yaml'

    # 检查是否已有同名地图
    if os.path.exists(map_pgm_path) or os.path.exists(map_yaml_path):
        print(f"⚠️  警告：地图文件已存在：")
        print(f"   {map_pgm_path}")
        print(f"   {map_yaml_path}")
        response = input("  是否覆盖？(y/N): ")
        if response.lower() != 'y':
            print("❌ 已取消保存")
            return False

    print(f"🗺️  正在保存地图...")
    print(f"   名称: {map_name}")
    print(f"   路径: {map_dir}/")

    try:
        # 调用 nav2_map_server 的 map_saver_cli
        result = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
             '-f', map_file_path,
             '--mode', 'trinary'],
            capture_output=True,
            text=True,
            timeout=30
        )

        if result.returncode == 0:
            print(f"✅ 地图保存成功！")
            print(f"   {map_pgm_path}")
            print(f"   {map_yaml_path}")
            print(f"\n💡 使用此地图导航：")
            print(f"   ros2 launch fishbot_navigation2 navigation2.launch.py "
                  f"map:={map_yaml_path}")
            return True
        else:
            print(f"❌ 地图保存失败 (返回码: {result.returncode})")
            if result.stderr:
                print(f"   错误: {result.stderr}")
            return False

    except FileNotFoundError:
        print("❌ 找不到 nav2_map_server，请确认已安装：")
        print("   sudo apt install ros-humble-nav2-map-server")
        return False
    except subprocess.TimeoutExpired:
        print("❌ 地图保存超时（30s）")
        return False
    except Exception as e:
        print(f"❌ 保存地图时出错: {e}")
        return False


def list_maps(map_dir: str = None):
    """列出已保存的地图"""
    if map_dir is None:
        map_dir = get_default_map_dir()

    if not os.path.exists(map_dir):
        print(f"📂 地图目录不存在: {map_dir}")
        return

    yaml_files = sorted([f for f in os.listdir(map_dir) if f.endswith('.yaml')])

    if not yaml_files:
        print(f"📂 地图目录为空: {map_dir}")
        return

    print(f"📂 已保存的地图 (位于 {map_dir}):")
    for yaml_file in yaml_files:
        name = yaml_file.replace('.yaml', '')
        pgm_file = name + '.pgm'
        pgm_path = os.path.join(map_dir, pgm_file)
        yaml_path = os.path.join(map_dir, yaml_file)

        pgm_size = os.path.getsize(pgm_path) if os.path.exists(pgm_path) else 0
        pgm_kb = pgm_size / 1024

        print(f"   🗺️  {name}")
        print(f"       .yaml: {yaml_path}")
        print(f"       .pgm:  {pgm_path} ({pgm_kb:.0f} KB)")


def main():
    parser = argparse.ArgumentParser(
        description='FishBot 地图保存工具 - 保存 Cartographer/Nav2 地图为文件'
    )
    parser.add_argument(
        '--name', '-n',
        default='fishbot_map',
        help='地图文件名（不含扩展名，默认: fishbot_map）'
    )
    parser.add_argument(
        '--dir', '-d',
        default=None,
        help='地图保存目录（默认: fishbot_navigation2/maps/）'
    )
    parser.add_argument(
        '--list', '-l',
        action='store_true',
        help='列出已保存的地图'
    )

    args = parser.parse_args()

    if args.list:
        list_maps(args.dir)
        return

    save_map(args.name, args.dir)


if __name__ == '__main__':
    main()
