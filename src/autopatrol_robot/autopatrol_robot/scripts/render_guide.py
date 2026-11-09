#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线重出手绘地图 PNG（不启动 web 服务、不碰 ROS、不会让车动）。

为什么需要这个脚本
------------------
`web_controller.py` 是长驻进程，`guide_map` 模块一旦 import 就固定在内存里。
改了 `guide_map.py` 里的配色（`VECTOR_BG` 等）之后：

  * 网页上看到的**已有 PNG** 不会变（文件没动）
  * 但在网页点「导出」时，是**旧进程里的旧代码**在画图 → 配色退回旧值

而已连着实车时不能重启 `web_controller`（bringup 会变孤儿，
`micro_ros_agent` 还活着 → ESP32 收到 /cmd_vel 就跑）。

所以这个脚本直接以当前源码为准，把三张图重新生成一遍：

    python3 scripts/render_guide.py                 # 默认 room1
    python3 scripts/render_guide.py --map room1
    python3 scripts/render_guide.py --scale 6       # 换超采样倍数
    python3 scripts/render_guide.py --kind vector   # 只重出线稿

安全性
------
* 只**读** maps/<map>.pgm / .yaml / <map>_regions.json
* 只**写** maps/guide_render/<map>_render*.png
* 不 import rclpy，不建 ROS 节点，不发任何话题
"""
import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
if _PKG not in sys.path:
    sys.path.insert(0, _PKG)

import guide_map as gm                      # noqa: E402


def _maps_dir(explicit):
    if explicit:
        return os.path.abspath(os.path.expanduser(explicit))
    # 本脚本位于 <ws>/src/autopatrol_robot/autopatrol_robot/scripts/
    ws = os.path.abspath(os.path.join(_HERE, '..', '..', '..', '..'))
    return os.path.join(ws, 'src', 'fishbot_navigation2', 'maps')


def main():
    ap = argparse.ArgumentParser(
        description='离线重出手绘地图 PNG（从工作台着色，不重启 web 服务）')
    ap.add_argument('--map', default='room1', help='地图名（默认 room1）')
    ap.add_argument('--scale', type=int, default=4, help='超采样倍数（默认 4）')
    ap.add_argument('--maps-dir', default='', help='maps 目录（默认自动定位）')
    ap.add_argument('--kind', default='all',
                    choices=['all', 'vector', 'annotated', 'clean'],
                    help='只重出某一份（默认全部）')
    args = ap.parse_args()

    mdir = _maps_dir(args.maps_dir)
    print('maps 目录 : %s' % mdir)

    yaml_p = os.path.join(mdir, args.map + '.yaml')
    pgm_p = os.path.join(mdir, args.map + '.pgm')
    has_yaml = os.path.isfile(yaml_p)

    if not os.path.isfile(pgm_p):
        print('✗ 找不到地图图片: %s' % pgm_p, file=sys.stderr)
        return 2

    arr, err = gm.read_pgm(pgm_p)
    if arr is None:
        print('✗ 读取地图失败: %s' % err, file=sys.stderr)
        return 2
    map_size = (int(arr.shape[1]), int(arr.shape[0]))

    meta = (gm.read_yaml_meta(yaml_p)[0] if has_yaml
            else {'occupied_thresh': 0.65, 'free_thresh': 0.196,
                  'negate': 0})

    regions_data, _e = gm.load_regions(mdir, args.map)
    regions = regions_data.get('regions', [])
    strokes = regions_data.get('strokes', [])
    auto = sum(1 for s in strokes if s.get('auto'))

    print('地图尺寸 : %dx%d' % map_size)
    print('标注数据 : regions=%d  strokes=%d（自动描边 %d / 手画 %d）'
          % (len(regions), len(strokes), auto, len(strokes) - auto))
    print('配色     : VECTOR_BG=#%02x%02x%02x' % gm.VECTOR_BG)
    print('')

    want = args.kind
    rc = 0

    # 三态分类只做一次，线稿/带标注/纯底图都用它
    cls = gm.classify(arr, meta.get('occupied_thresh', 0.65),
                      meta.get('free_thresh', 0.196),
                      meta.get('negate', 0))

    if want in ('all', 'vector'):
        out = gm.render_png_path(mdir, args.map, 'vector')
        # 需要 cls 才能把封闭空闲区域填白（房间内部白 / 墙外暖沙色）
        ok, msg = gm.export_vector_png(regions, strokes, out, map_size,
                                       scale=args.scale, cls=cls)
        print('%s 线稿      %s  %s' % ('✓' if ok else '✗', out, msg))
        rc |= 0 if ok else 1

    if want in ('all', 'annotated'):
        out = gm.render_png_path(mdir, args.map, 'annotated')
        ok, msg = gm.export_render_png(cls, regions, strokes, out,
                                       scale=args.scale)
        print('%s 带标注    %s  %s' % ('✓' if ok else '✗', out, msg))
        rc |= 0 if ok else 1

    if want in ('all', 'clean'):
        out = gm.render_png_path(mdir, args.map, 'clean')
        ok, msg = gm.export_clean_png(cls, out, scale=1)
        print('%s 纯底图    %s  %s' % ('✓' if ok else '✗', out, msg))
        rc |= 0 if ok else 1

    print('')
    print('完成。网页刷新后即可看到新配色（/guide/render 只读文件，不需重启服务）。')
    return rc


if __name__ == '__main__':
    sys.exit(main())
