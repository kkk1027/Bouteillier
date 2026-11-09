#!/usr/bin/env python3
"""guide_map.py — 初次建图引导：地图渲染、自动描边、区域标签存取。

对应网页端「引导页 → 手动渲染页」的后端支撑。**完全只读原地图**：
`room1.pgm` / `room1.yaml` 一个字节都不会被改写，所有新产物都是
新增文件（`room1_render.png`、`room1_regions.json`）。

--------------------------------------------------------------------------
三态配色（2026-11-01 用户确认）
--------------------------------------------------------------------------
实测 room1.pgm 的数值分布：

    value   0    →  6.8%   占据（墙）
    value 205    → 67.9%   未知（没扫到）
    value 254    → 25.3%   自由（空地）

未知区占了三分之二。如果把未知区和自由区都涂成纯白，图上就分不出
「确定是空地」和「根本没探测过」——用户在第二页框区域时很容易把
标签框进一个激光从没扫到的空洞里。

所以用三态：

    占据  #E07A3C   暖橙（强调「这里有东西」）
    自由  #FFFFFF   纯白（用户要求的空白区）
    未知  #F7ECE0   浅米色（和页面底色同族，不抢眼但看得出差别）

--------------------------------------------------------------------------
分类算法：照搬 ROS map_server 的定义，不自己编阈值
--------------------------------------------------------------------------
    occ = (255 - value) / 255.0          # negate=0 时
    occ > occupied_thresh  → 占据
    occ < free_thresh      → 自由
    其余                    → 未知

room1.yaml 里是 occupied_thresh=0.65 / free_thresh=0.196：

    value 0   → occ=1.0000 → 占据
    value 205 → occ=0.1961 → 未知（0.1961 不 < 0.196，差一点点，正好落进 else）
    value 254 → occ=0.0039 → 自由

这个「差一点点」不是巧合：205 就是 map_saver 用来写未知区的值，
free_thresh 0.196 就是从它反推出来的。所以必须用标准公式，
不能用 `value == 205` 这种硬编码（换台机器建图可能是 0xCD 之外的值）。
"""

import base64
import io
import json
import os
import re

import numpy as np
from PIL import Image

# ★ 在模块顶层把 scipy 导入完，而不是用到时再 import。
#   首次 import scipy 实测要 ~2.4 秒（加载大量 .so），如果放在
#   auto_trace 里懒加载，用户第一次点「自动描边」会卡住两秒多，
#   看起来就像按了没反应。放到顶层后这笔开销落在 web_controller
#   启动阶段，请求时是 0。
#   scipy 只用于连通域与形态学，缺了就降级成「不筛选」，不影响主流程。
try:
    from scipy import ndimage as _ndi
except ImportError:                                     # pragma: no cover
    _ndi = None

# ============================================================
# 三态常量与配色
# ============================================================
OCC, FREE, UNK = 0, 1, 2

# 配色与 control.html 的暖色主题同族：
#   --w-brand #f08c3c、--w-bg #fdf4e8
DEFAULT_PALETTE = {
    OCC: (0xE0, 0x7A, 0x3C),    # 暖橙：墙 / 障碍
    FREE: (0xFF, 0xFF, 0xFF),   # 纯白：空地
    UNK: (0xF7, 0xEC, 0xE0),    # 浅米：未知（没探测过）
}

# 自动描边用色：比占据色更深一点，压在图上能看清
STROKE_COLOR = '#b8562a'


# ============================================================
# 地图读取
# ============================================================

def read_pgm(path):
    """读 PGM（P5）成 numpy uint8 数组。返回 (array, error)。"""
    try:
        img = Image.open(path)
    except Exception as exc:                      # noqa: BLE001
        return None, '读取地图失败: %s' % exc
    if img.mode not in ('L', 'I', 'F'):
        # P2/P5 都是单通道；万一被转成 RGB 说明文件不对
        img = img.convert('L')
    return np.array(img, dtype=np.uint8), ''


def read_yaml_meta(path):
    """解析 map yaml 的少量字段。

    刻意不依赖 pyyaml：需要的信息只有 4 个标量 + 一个三元组，
    手写解析比引入依赖更可控（而且 yaml 文件缺字段时要能给出默认值，
    pyyaml 的异常处理反而更啰嗦）。
    """
    meta = {
        'resolution': 0.05,
        'origin': [-0.0, -0.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'image': '',
    }
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            text = f.read()
    except OSError as exc:
        return meta, '读取地图配置失败: %s' % exc

    def _scalar(key, cast):
        m = re.search(r'^\s*%s\s*:\s*([^\s#]+)' % re.escape(key),
                      text, re.MULTILINE)
        if not m:
            return
        try:
            meta[key] = cast(m.group(1))
        except (TypeError, ValueError):
            pass

    _scalar('resolution', float)
    _scalar('negate', lambda v: int(float(v)))
    _scalar('occupied_thresh', float)
    _scalar('free_thresh', float)

    m = re.search(r'^\s*image\s*:\s*([^\s#]+)', text, re.MULTILINE)
    if m:
        meta['image'] = m.group(1).strip('"\'')

    m = re.search(r'^\s*origin\s*:\s*\[([^\]]*)\]', text, re.MULTILINE)
    if m:
        parts = [p.strip() for p in m.group(1).split(',') if p.strip()]
        if len(parts) >= 2:
            try:
                meta['origin'] = [float(parts[0]), float(parts[1]),
                                  float(parts[2]) if len(parts) > 2 else 0.0]
            except ValueError:
                pass
    return meta, ''


def classify(arr, occupied_thresh=0.65, free_thresh=0.196, negate=0):
    """按 ROS map_server 的定义把灰度值分成三态。"""
    a = arr.astype(np.float32)
    if negate:
        # negate=1 表示语义相反（黑=自由）
        occ = a / 255.0
    else:
        occ = (255.0 - a) / 255.0

    out = np.full(arr.shape, UNK, dtype=np.uint8)
    # 先定自由，再覆盖占据 —— 占据优先级更高
    out[occ < free_thresh] = FREE
    out[occ > occupied_thresh] = OCC
    return out


# ============================================================
# 渲染
# ============================================================

def render_rgb(cls, palette=None):
    """三态索引图 → RGB 数组。"""
    pal = palette or DEFAULT_PALETTE
    h, w = cls.shape
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    for state, color in pal.items():
        rgb[cls == state] = color
    return rgb


def render_png_base64(cls, palette=None, scale=1):
    """三态索引图 → base64 PNG（给前端直接塞进 <img>）。

    scale>1 时用**最近邻**放大：地图是像素级的，双线性会把墙
    糊成渐变，二值化的边界就看不清了。
    """
    img = Image.fromarray(render_rgb(cls, palette), 'RGB')
    if scale and scale > 1:
        img = img.resize((img.width * scale, img.height * scale),
                         Image.NEAREST)
    buf = io.BytesIO()
    img.save(buf, format='PNG', optimize=True)
    return base64.b64encode(buf.getvalue()).decode('ascii')


# ============================================================
# 自动描边
# ============================================================

def _runs(bool_row):
    """把一维布尔数组里连续 True 的区间转成 [(start, end_exclusive)]。"""
    if bool_row.size == 0:
        return []
    # 差分找边界：上升沿是起点，下降沿是终点
    padded = np.concatenate(([False], bool_row, [False]))
    diff = np.diff(padded.astype(np.int8))
    starts = np.flatnonzero(diff == 1)
    ends = np.flatnonzero(diff == -1)
    return list(zip(starts.tolist(), ends.tolist()))


def ortho_boundary(mask):
    """提取二值区域的**正交像素边界**，返回 [(x0, y0, x1, y1), ...]。

    为什么用正交边界而不是画斜线/样条：
      SLAM 出来的地图像素本身就是格子状的，画斜线会引入「描得比实际准」
      的假象。正交线（横平竖直）反而和像素边界严丝合缝，看起来像建筑
      图纸，而且用户后续想改成斜线时是可以自己拖的。

    坐标系：像素格点坐标，x 向右、y 向下（与 control.html 里
    worldToPixel 的约定一致，**不做 Y 翻转**）。
    """
    if mask.ndim != 2 or mask.size == 0:
        return []
    h, w = mask.shape
    segs = []

    # ---- 水平边：相邻两行在某段 x 上不同 ----
    for y in range(1, h):
        diff = mask[y] != mask[y - 1]
        for x0, x1 in _runs(diff):
            segs.append((int(x0), int(y), int(x1), int(y)))
    # 图像上边界 / 下边界处的边框
    for x0, x1 in _runs(mask[0]):
        segs.append((int(x0), 0, int(x1), 0))
    for x0, x1 in _runs(mask[h - 1]):
        segs.append((int(x0), int(h), int(x1), int(h)))

    # ---- 垂直边：相邻两列在某段 y 上不同 ----
    for x in range(1, w):
        diff = mask[:, x] != mask[:, x - 1]
        for y0, y1 in _runs(diff):
            segs.append((int(x), int(y0), int(x), int(y1)))
    for y0, y1 in _runs(mask[:, 0]):
        segs.append((0, int(y0), 0, int(y1)))
    for y0, y1 in _runs(mask[:, w - 1]):
        segs.append((int(w), int(y0), int(w), int(y1)))

    return segs


def _largest_component(mask):
    """返回 mask 中最大的连通域（4 邻域），失败时原样返回。

    目的：SLAM 图里的自由区常常碎成好几块（走廊被门挡住、扫描缝隙）。
    直接描所有碎块的边界会得到一堆雪花点，只保留最大那块才是
    「用户心里的那张房间图」。
    """
    if _ndi is None or not mask.any():
        return mask

    labels, n = _ndi.label(mask)
    if n <= 1:
        return mask
    sizes = _ndi.sum(mask, labels, range(1, n + 1))
    biggest = int(np.argmax(sizes)) + 1
    return labels == biggest


def _smooth_mask(mask, iterations=1):
    """形态学闭运算填掉 1 像素的小孔，避免边界出现毛刺。"""
    if _ndi is None or iterations <= 0:
        return mask
    struct = np.ones((3, 3), dtype=bool)
    out = mask
    for _ in range(iterations):
        out = _ndi.binary_closing(out, structure=struct)
    return out


def _block_downsample(mask, block):
    """按 block×block 块降采样：块内过半为前景则整体算前景。

    ★ 为什么需要这一步（实测踩到的）：
      room1 的自由区边界是**斜的**，实测前几行的宽度是
      5 / 7 / 19 / 27 / 34 / 42 / 52 / 55 px —— 每行只宽一点点。
      对二进制这样直接提正交边界，每个「台阶」只有 1~2px 高，
      整张图会得到 338 条碎段，最长 14px、208 条长度只有 1，
      完全看不出这是房间轮廓。
      块降采样相当于把锯齿磨成大方块，台阶变高、段数变少，
      轮廓就能读出来了。剩下的方块台阶用户用直线工具拉直即可。
    """
    if block <= 1:
        return mask, 1
    h, w = mask.shape
    h2 = (h // block) * block
    w2 = (w // block) * block
    if h2 == 0 or w2 == 0:
        return mask, 1
    m = mask[:h2, :w2]
    blocks = m.reshape(h2 // block, block, w2 // block, block)
    return (blocks.mean(axis=(1, 3)) >= 0.5), block


def auto_trace(cls, min_segment=3, max_segments=400, block=3):
    """从三态图自动生成初始轮廓线。

    返回 dict：
        {
          'segments': [[x0,y0,x1,y1], ...],   # 像素坐标
          'truncated': bool,                  # 是否因为太多被截断
          'count': int,
          'block': int,                       # 实际使用的降采样倍数
        }

    `min_segment`：短于这个长度的段直接丢掉（去掉锯齿毛刺）
    `max_segments`：上限保护。段太多前端 canvas 会卡，而且人也看不清
    `block`：块降采样倍数。调大会得到更粗但更整齐的台阶状轮廓，
             调小（=1）则完全不平滑，几乎不可用 —— 见 _block_downsample 注释
    """
    free_mask = (cls == FREE)
    # 先闭运算再取最大连通域：先补孔再判连通，否则大房间会因为
    # 一个小空洞被切成两块
    free_mask = _smooth_mask(free_mask, iterations=1)
    free_mask = _largest_component(free_mask)

    # 降采样 → 提边界 → 坐标乘回原尺度
    small, factor = _block_downsample(free_mask, block)
    segs = ortho_boundary(small)

    out = []
    for x0, y0, x1, y1 in segs:
        length = abs(x1 - x0) + abs(y1 - y0)
        if length < min_segment:
            continue
        out.append([x0 * factor, y0 * factor, x1 * factor, y1 * factor])

    truncated = False
    if len(out) > max_segments:
        # 按长度从长到短保留 —— 长的多半是房间的主轮廓
        out.sort(key=lambda s: -(abs(s[2] - s[0]) + abs(s[3] - s[1])))
        out = out[:max_segments]
        truncated = True

    return {'segments': out, 'count': len(out),
            'truncated': truncated, 'block': factor}


# ============================================================
# 区域标签存取
# ============================================================

def regions_path(maps_dir, map_name):
    return os.path.join(maps_dir, '%s_regions.json' % map_name)


# ============================================================
# 导出目录
# ============================================================
def render_dir(maps_dir):
    """手绘渲染图的独立存放目录：maps/guide_render/。

    ★ 为什么不直接放在 maps/ 下：
      map_server / list_available_maps 会扫 maps/*.yaml，虽然 .png 不会
      被当成地图，但混在一起让人分不清哪个是"真地图"、哪个是"手绘稿"。
      单独一个子目录既干净，也方便日后整个删掉重来。
    """
    return os.path.join(maps_dir, 'guide_render')


def render_png_path(maps_dir, map_name, kind='vector'):
    """导出文件的路径。

    kind 三种产物：
      'vector'     纯线稿。只有线条 + 区域框 + 名字，**没有源地图栅格**。
                   这是用户要的"手绘地图"成品，也是地图面板显示的版本。
      'annotated'  三态栅格底图 + 标注。给答辩 / 文档用（看得到原始扫描）。
      'clean'      纯三态栅格底图，无标注。留着备查，面板不用它。

    默认返回 vector 而不是 annotated —— 之前默认给的是 clean（栅格图），
    用户看到的就是一张粗糙的扫描图，而不是自己描的线。
    """
    suffix = {'annotated': '', 'clean': '_clean', 'vector': '_vector'}
    return os.path.join(render_dir(maps_dir),
                        '%s_render%s.png' % (map_name, suffix.get(kind, '')))


def empty_regions(map_name):
    return {
        'map': map_name,
        'resolution': 0.05,
        'origin': [0.0, 0.0, 0.0],
        'regions': [],
        'strokes': [],
    }


def load_regions(maps_dir, map_name):
    """读区域标签。文件不存在时返回空结构（**不是错误**）。"""
    path = regions_path(maps_dir, map_name)
    if not os.path.isfile(path):
        return empty_regions(map_name), ''
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except (OSError, ValueError) as exc:
        return empty_regions(map_name), '读取区域标注失败: %s' % exc

    if not isinstance(data, dict):
        return empty_regions(map_name), '区域标注格式不对（不是对象）'
    data.setdefault('map', map_name)
    data.setdefault('regions', [])
    data.setdefault('strokes', [])
    data.setdefault('resolution', 0.05)
    data.setdefault('origin', [0.0, 0.0, 0.0])
    data['regions'] = [r for r in data['regions'] if isinstance(r, dict)]
    data['strokes'] = [s for s in data['strokes'] if isinstance(s, dict)]
    return data, ''


def save_regions(maps_dir, map_name, data):
    """写区域标签（原子写：先写临时文件再 rename）。

    为什么要原子写：手机端网络不稳，如果直接覆盖写的过程中断，
    room1_regions.json 会变成半个 JSON，下次读取直接报「格式不对」，
    用户辛苦标的区域就全丢了。
    """
    path = regions_path(maps_dir, map_name)
    tmp = path + '.tmp'
    try:
        os.makedirs(maps_dir, exist_ok=True)
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        os.replace(tmp, path)
    except OSError as exc:
        try:
            if os.path.isfile(tmp):
                os.remove(tmp)
        except OSError:
            pass
        return False, '保存区域标注失败: %s' % exc
    return True, path


def pixel_to_world(px, py, origin, resolution):
    """像素格点 → 世界坐标（米）。与 control.html 的 worldToPixel 互逆。"""
    return (origin[0] + px * resolution,
            origin[1] + py * resolution)


def world_to_pixel(wx, wy, origin, resolution):
    """世界坐标 → 像素格点。"""
    return ((wx - origin[0]) / resolution,
            (wy - origin[1]) / resolution)


def normalize_regions(regions, origin, resolution, map_size):
    """校验并补全前端传来的区域列表。

    ★ 世界坐标在**后端**算，不信前端传的：
      前端可能算错（漏了原点偏移、用错分辨率），而 AI 导航直接吃
      这个坐标，错一点就导到墙里。以后端为准最省事也最安全。
    """
    w, h = map_size
    out = []
    warnings = []
    seen = {}

    for item in regions or []:
        if not isinstance(item, dict):
            continue
        name = str(item.get('name', '')).strip()
        if not name:
            warnings.append('跳过了一个没有名字的区域')
            continue

        bbox = item.get('bbox_px')
        if not (isinstance(bbox, (list, tuple)) and len(bbox) == 4):
            warnings.append('区域「%s」缺少 bbox_px，已跳过' % name)
            continue
        try:
            x0, y0, x1, y1 = [float(v) for v in bbox]
        except (TypeError, ValueError):
            warnings.append('区域「%s」的 bbox_px 不是数字' % name)
            continue

        # 归一化成从小到大，再夹到地图范围内
        x0, x1 = sorted((x0, x1))
        y0, y1 = sorted((y0, y1))
        x0 = max(0.0, min(x0, float(w)))
        x1 = max(0.0, min(x1, float(w)))
        y0 = max(0.0, min(y0, float(h)))
        y1 = max(0.0, min(y1, float(h)))
        if x1 - x0 < 1 or y1 - y0 < 1:
            warnings.append('区域「%s」太小（小于 1 像素），已跳过' % name)
            continue

        cx = (x0 + x1) / 2.0
        cy = (y0 + y1) / 2.0
        wx, wy = pixel_to_world(cx, cy, origin, resolution)

        if name in seen:
            warnings.append('区域名「%s」重复了，AI 只会用第一个' % name)
        seen[name] = True

        out.append({
            'name': name,
            'bbox_px': [round(x0, 1), round(y0, 1),
                        round(x1, 1), round(y1, 1)],
            'center_px': [round(cx, 1), round(cy, 1)],
            'world': {'x': round(wx, 3), 'y': round(wy, 3)},
            'color': item.get('color') or '#f0a75c',
        })
    return out, warnings


def normalize_strokes(strokes, map_size):
    """校验并补全手绘线条。只保留认识的基本图形。"""
    w, h = map_size
    out = []
    for item in strokes or []:
        if not isinstance(item, dict):
            continue
        kind = item.get('type')
        pts = item.get('pts')
        if kind not in ('line', 'rect', 'circle', 'poly'):
            continue
        if not (isinstance(pts, (list, tuple)) and len(pts) >= 2):
            continue
        clean = []
        for p in pts[:64]:
            try:
                px = float(p[0])
                py = float(p[1])
            except (TypeError, ValueError, IndexError):
                continue
            clean.append([round(max(0.0, min(px, float(w))), 1),
                          round(max(0.0, min(py, float(h))), 1)])
        if len(clean) < 2:
            continue
        try:
            width = float(item.get('w', 2))
        except (TypeError, ValueError):
            width = 2.0
        out.append({
            'type': kind,
            'pts': clean,
            'w': round(max(0.5, min(width, 12.0)), 2),
            'color': item.get('color') or STROKE_COLOR,
            # ★ 保留 auto 标记：区分「自动描边生成的线」和「用户手画的线」。
            #   两者都要导出到线稿，但要能分辨（日后想「只看手画的」也能过滤）。
            'auto': bool(item.get('auto')),
        })
    return out


# ============================================================
# 导出渲染图
# ============================================================

# 中文字体候选：按优先级探测。都没有就跳过文字，只画框。
_FONT_CANDIDATES = [
    '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',
    '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/arphic/uming.ttc',
    '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',
]


def _load_cjk_font(size):
    """找一个能显示中文的字体。找不到返回 None（调用方需降级）。"""
    from PIL import ImageFont
    for path in _FONT_CANDIDATES:
        if os.path.isfile(path):
            try:
                return ImageFont.truetype(path, size)
            except OSError:
                continue
    return None


def export_render_png(cls, regions, strokes, out_path, scale=4):
    """把三态图 + 手绘线 + 区域框与名字合成一张 PNG。

    返回 (ok, message)。message 在成功时是输出路径，失败时是原因。

    ★ 文字是「尽力而为」：没有中文字体就**只画框不写字**。
      宁可导出图里缺名字，也不能因为字体缺失让整个导出接口 500 ——
      用户做完一整套引导流程后点导出失败，会非常难受。
    """
    return _export_png(cls, regions, strokes, out_path, scale,
                       with_annotations=True)


def export_clean_png(cls, out_path, scale=1):
    """只导出三态底图，不画任何标注。

    用途：备查「原始扫描长什么样」。
    **不再给地图面板用** —— 面板要的是纯线稿，见 export_vector_png。
    """
    return _export_png(cls, [], [], out_path, scale,
                       with_annotations=False)


# ============================================================
# 纯线稿（用户要的「手绘地图」成品）
# ============================================================

# 线稿配色：与 control.html 的暖色主题同族
# 线稿底色：偏暖的沙色。
#
# ★ 原来是 (255,253,248)，几乎就是白色 —— 用户实际看起来「背景还是白的」。
#   现在改成明确偏暖的沙色，并且与网页地图面板的 MAP_BG **完全同一个值**。
#   两边颜色一致，把图贴到面板上就不会出现接缝/分割线。
#   比单纯加饱和更重要的约束：不能压过棕色线条（184,86,42 / 201,134,60），
#   亮度要够高，否则线条会看不清。
VECTOR_BG = (0xF5, 0xDD, 0xB9)       # #f5ddb9 沙色底（= 面板 MAP_BG）
VECTOR_FREE = (0xFF, 0xFF, 0xFF)     # 封闭空闲区域：纯白（用户要求）
VECTOR_GRID = (0xE2, 0xC8, 0xA2)     # 比底色略深的参考网格（可选，见参数）
VECTOR_LINE_AUTO = (0xC9, 0x86, 0x3C)   # 自动描边线：浅棕
VECTOR_LINE_MANUAL = (0xB8, 0x56, 0x2A)  # 人工描的线：深棕（更显眼）
VECTOR_REGION = (0xF0, 0xA7, 0x5C)   # 区域框（同 REGION_COLOR）


def _rasterize_strokes(strokes, map_size, pad=0):
    """把绘制线烧成布尔遮罩（地图像素分辨率）。

    用于「沿描边线分割白区与背景」：把线本身当作墙，从空闲区里挖掉，
    白色就正好停在线上，不会多出 1~3px 的毛边。

    ★ 只用 DDA 整数步进，不做抗锯齿 —— 遮罩是硬边的，
      与后面 NEAREST 放大一致，不会出现灰边。

    参数：
      strokes  笔画列表（type: line/poly 参与；rect/circle 跳过）
      map_size (w, h) 地图像素尺寸
      pad      线条加粗像素数（0 = 只占 1px）
    """
    m = np.zeros((int(map_size[1]), int(map_size[0])), dtype=bool)

    def put(x, y):
        for dy in range(-pad, pad + 1):
            for dx in range(-pad, pad + 1):
                xx, yy = x + dx, y + dy
                if 0 <= yy < m.shape[0] and 0 <= xx < m.shape[1]:
                    m[yy, xx] = True

    for st in strokes or []:
        pts = st.get('pts') or []
        if len(pts) < 2:
            continue
        kind = st.get('type')
        if kind in ('line', 'poly'):
            seq = pts if kind == 'poly' else [pts[0], pts[-1]]
        else:
            # 矩形/圆不是墙线，不当边界用
            continue

        for i in range(len(seq) - 1):
            x0, y0 = int(round(seq[i][0])), int(round(seq[i][1]))
            x1, y1 = int(round(seq[i + 1][0])), int(round(seq[i + 1][1]))
            steps = max(abs(x1 - x0), abs(y1 - y0))
            if steps == 0:
                put(x0, y0)
                continue
            for s in range(steps + 1):
                t = s / float(steps)
                put(int(round(x0 + (x1 - x0) * t)),
                    int(round(y0 + (y1 - y0) * t)))
    return m


def free_mask(cls, block=1, smooth=1, barriers=None):
    """从三态图提取「封闭空闲区域」的布尔遮罩。

    ★ 用户要的效果是：**背景（墙外/未扫到的未知区）暖色，房间内部纯白**。
      所以不能直接拿 cls==FREE 当填充区，必须先收拾干净：

      1. `_largest_component` —— room1 的自由区实测是 6607 像素一大块
         + 1 个孤立噪点。只留最大块，噪点不会变成突兀的白点。
      2. `_smooth_mask`     —— 闭运算填掉 1px 小孔，边界不出毛刺。
      3. `binary_fill_holes` —— 把房间内部的小块未知/占据（家具、扫描缝）
         一起填成白色。不然房间里会东一块西一块的暖色空洞，很难看。
      4. `_block_downsample` —— 可选降采样。
      5. `barriers`          —— ★ 用**绘制线**当墙，从白区里挖掉。
         SLAM 原始自由区是斜的、毛的；而用户描的线（尤其手画直线）
         才是他们心里的墙。挖掉后白色边缘正好停在线上，
         实现「沿绘制的边线分割」。

    ★ 默认 block=**1**（全分辨率，不降采样）。
      2026-11-04 用户反馈「颗粒度太大了」，原本默认 3，在 4 倍图上
      就是 12px 的大台阶，白区边缘像乐高积木。改成 1 之后边界直接
      贴着真实的空闲区轮廓，实测平均跳变 0.86/1.39 px（左右边界），
      全分辨率 6703 像素 —— 边界细腻得多。

      ★ 注意：这个 block **不跟着** `auto_trace` 的 block 走。
        描边线是（block=3）粗粒度的，而填充要细；两者本来就不是一回事。
        线条画在填充**之上**，所以细填充不会被粗线埋没。

    没有 scipy 时（_ndi is None）各步自动降级，仍能返回可用的遮罩。

    参数：
      cls      classify() 的三态数组
      block    降采样块边长，1 = 全分辨率（推荐）
      smooth   闭运算次数，0 关闭
      barriers 笔画列表，当作墙从白区挖掉（可选）

    返回：
      (mask, block)  mask 是布尔数组，实际块边长由 block 给出。
      调用方放大时要乘 block，不是乘 1。
    """
    if cls is None:
        return None, 1

    free = (np.asarray(cls) == FREE)
    if not free.any():
        return None, 1

    m_h, m_w = free.shape

    free = _largest_component(free)
    free = _smooth_mask(free, smooth)
    if _ndi is not None:
        # 填房间内部的空洞（家具、扫描缝），否则房间里会有暖色斑点
        free = _ndi.binary_fill_holes(free)

    # ★ 用绘制线当墙，把白区挖回去，白边就正好停在线上。
    #
    #   pad=1（线加粗 1px）是实测出来的平衡点：
    #     pad=0 → 白区边界只有 83% 落在线上，剩下的部分是 block=3 描边线
    #             没盖住的细台阶，看起来白区从线外侧「鼓」了出来
    #     pad=1 → 98%，白区只缩 11%（731/6703 px），几乎看不出变小
    #     pad=2 → 100%，但要削掉1246 px（18.6%），房间明显变瘦
    #   pad=2 的 100% 不值得 —— 它靠吞噬白区换覆盖率。
    if barriers:
        free = free & ~_rasterize_strokes(barriers, (m_w, m_h), pad=1)

    return _block_downsample(free, block)


def _paste_free(img, mask):
    """把空闲区遮罩贴成白色。

    mask 是 `(h/block, w/block)` 的小图，直接 NEAREST 放大到画布尺寸 ——
    NEAREST 不插值，色块边界保持硬直，而且天然落在 block*S 的整数倍上，
    与自动描边线的坐标一致。
    """
    from PIL import Image

    m = Image.fromarray((np.asarray(mask).astype(np.uint8) * 255), 'L')
    if m.size != img.size:
        m = m.resize(img.size, Image.NEAREST)

    white = Image.new('RGB', img.size, VECTOR_FREE)
    img.paste(white, (0, 0), m)


def export_vector_png(regions, strokes, out_path, map_size,
                      scale=4, grid=False, cls=None, block=1):
    """导出**纯线稿**：没有源地图栅格，只有抽象的线和标签。

    ★ 这是用户明确要求的形态：
        ①手动设置的标签 ②人工描的线段 ③预描的已有线段
        不要粗糙的源地图栅格。

    ★ 底色分层（用户 2026-11-04 追加要求）：
        最底层  暖沙色 VECTOR_BG  —— 墙外 / 未扫描的未知区
        中间层  纯白 VECTOR_FREE   —— **封闭空闲区域**（房间内部）
        再往上  区域半透明底色 → 线条 → 文字
      所以传了 cls 才会填白；不传 cls 就是一张纯色底的线稿。

    ★ 自动描边线和手画线分开配色：
        自动描边 = 浅棕（底稿，可以粗略）
        人工描的 = 深棕（用户用心画的，要突出）
      另传的 color 字段依然优先生效（将来想自定义颜色不用改这里）。

    参数：
      map_size  (w, h) 地图像素尺寸。线稿坐标系与地图完全一致，
                这样叠加机器人位姿时不需要任何额外换算。
      scale     超采样倍数。默认 4：地图只有 126x207，放大 4 倍后
                线条边缘才有足够像素可以抗锯齿，缩小回显示尺寸才清晰。
      grid      是否画极淡的参考网格（建筑图纸感，默认关）
      cls       classify() 的三态数组。传了才把空闲区填白。
      block     空闲区遮罩降采样块边长。**1 = 全分辨率**（默认），
                边界最细腻；调大可把边界磨成大方块（一般不推荐）。
    """
    try:
        from PIL import Image, ImageDraw
    except ImportError:                          # pragma: no cover
        return False, 'PIL 不可用'

    try:
        w, h = int(map_size[0]), int(map_size[1])
        if w <= 0 or h <= 0:
            return False, '地图尺寸无效: %r' % (map_size,)

        S = max(1, int(scale))
        img = Image.new('RGB', (w * S, h * S), VECTOR_BG)

        # ① 封闭空闲区域填白（房间内部）
        #
        # ★ 把 strokes 当 barriers 传进去：白区边缘会正好停在描边线上，
        #   而不是跟着 SLAM 原始自由区的毛边跑。这是用户要的
        #   「沿绘制的边线分割」。
        mask = (free_mask(cls, block, barriers=strokes)[0]
                if cls is not None else None)
        if mask is not None:
            _paste_free(img, mask)

        draw = ImageDraw.Draw(img, 'RGBA')

        if grid:
            _draw_grid(draw, w, h, S)

        # 绘制顺序（从下往上）：区域底色 → 线条 → 区域名字
        #
        # ★ 线条必须在区域底色**之上**。
        #   底色是半透明的，先画线再盖底色会把线染成混色 ——
        #   实测深棕 (184,86,42) 被盖成了 (194,101,51)，颜色都不对了。
        # ★ 名字又在最顶层：标签是用户手动设的语义信息，不能被线划穿。
        # ★ 线稿的填充比三态图更淡（28 vs 46）：线稿要“干净”，
        #   底色只是提示范围，抢眼了就不像线稿了。
        _draw_regions(draw, regions, S, part='box', fill_alpha=28)
        _draw_strokes(draw, strokes, S, palette='vector')
        note = _draw_regions(draw, regions, S, part='text', halo=VECTOR_BG)

        os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
        img.save(out_path, 'PNG', optimize=True)
        if not regions:
            note = '已导出纯线稿（无区域标签）'
        return True, note
    except Exception as exc:                     # noqa: BLE001
        return False, '导出线稿失败: %s' % exc


def _draw_grid(draw, w, h, S, step=10):
    """极淡的 10 像素参考网格，让线稿看起来像图纸。"""
    for x in range(0, w + 1, step):
        draw.line([(x * S, 0), (x * S, h * S)],
                  fill=VECTOR_GRID + (255,), width=1)
    for y in range(0, h + 1, step):
        draw.line([(0, y * S), (w * S, y * S)],
                  fill=VECTOR_GRID + (255,), width=1)


def _export_png(cls, regions, strokes, out_path, scale, with_annotations):
    """导出 PNG 的实际实现。"""
    try:
        from PIL import ImageDraw
    except ImportError:                          # pragma: no cover
        return False, 'PIL 不可用'

    try:
        S = max(1, int(scale))
        img = Image.fromarray(render_rgb(cls), 'RGB')
        if S > 1:
            # 地图是像素图：用最近邻放大保持硬边，双三次会把墙糊掉
            img = img.resize((img.width * S, img.height * S), Image.NEAREST)

        if with_annotations:
            draw = ImageDraw.Draw(img, 'RGBA')
            # 同线稿一样分层：底色 → 线条 → 名字，避免半透明底色给线染色
            _draw_regions(draw, regions, S, part='box')
            _draw_strokes(draw, strokes, S)
            note = _draw_regions(draw, regions, S, part='text')
        else:
            note = '已导出（纯底图）'

        os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
        img.save(out_path, 'PNG', optimize=True)
        return True, note
    except Exception as exc:                     # noqa: BLE001
        return False, '导出图片失败: %s' % exc


def _draw_strokes(draw, strokes, S, palette='flat'):
    """把标注线烧到图上。

    palette:
      'flat'   所有线用同一个 STROKE_COLOR（三态底图上用，保证对比度）
      'vector' 自动描边线用浅棕、手画线用深棕（线稿用，主次分明）

    ★ 线宽处理：stroke 里的 w 是**地图像素**单位（1.5~1.8），
      乘 S 之后在 4 倍图上就是 6~7px，缩小回去约 1.5px，正好。
      下限保 2px 是为了防止 1 倍导出时线细到看不见。
    """
    for st in strokes or []:
        pts = [(p[0] * S, p[1] * S) for p in st.get('pts', [])]
        if len(pts) < 2:
            continue

        # 颜色优先级：显式 color > palette 给的默认色
        explicit = st.get('color')
        if explicit and explicit != STROKE_COLOR:
            color = _hex_to_rgb(explicit)
        elif palette == 'vector':
            color = (VECTOR_LINE_AUTO if st.get('auto')
                     else VECTOR_LINE_MANUAL)
        else:
            color = _hex_to_rgb(STROKE_COLOR)

        width = max(2, int(round(float(st.get('w', 2)) * S)))
        kind = st.get('type')
        if kind == 'rect' and len(pts) >= 2:
            x0, y0 = pts[0]
            x1, y1 = pts[1]
            draw.rectangle([min(x0, x1), min(y0, y1),
                            max(x0, x1), max(y0, y1)],
                           outline=color, width=width)
        elif kind == 'circle' and len(pts) >= 2:
            (cx, cy), (ex, ey) = pts[0], pts[1]
            r = ((ex - cx) ** 2 + (ey - cy) ** 2) ** 0.5
            draw.ellipse([cx - r, cy - r, cx + r, cy + r],
                         outline=color, width=width)
        elif kind == 'poly':
            draw.line(pts, fill=color, width=width, joint='curve')
        else:
            draw.line([pts[0], pts[-1]], fill=color, width=width)


def _draw_regions(draw, regions, S, part='all', fill_alpha=46,
                  halo=(255, 255, 255)):
    """把区域框 + 名字烧到图上。返回提示文字。

    part:
      'box'   只画底色块 + 边框
      'text'  只写名字
      'all'   两个都画（默认，供三态图用）

    halo: 名字的描边色。
      ★ 白色描边在沙色底上会变成一圈刺眼的白点（实测 132 个纯白像素），
        所以线稿里直接传底色 VECTOR_BG —— 描边相当于“把字后面的线抹掉”，
        既保证可读，又与背景完全融合。

    ★ 为什么要拆开：线稿里区域半透明底色会盖住线条（混色后线就变色了），
      所以线稿的绘制顺序是 box → strokes → text，最后写字保证名字不被线压住。
    """
    want_text = part in ('text', 'all')
    want_box = part in ('box', 'all')
    font = _load_cjk_font(max(10, 5 * S)) if want_text else None
    note = '已导出'
    if want_text and regions and font is None:
        note = '已导出（未找到中文字体，区域名未绘制）'

    halo_rgba = tuple(halo[:3]) + (215,)

    for rg in regions or []:
        bbox = rg.get('bbox_px')
        if not (isinstance(bbox, (list, tuple)) and len(bbox) == 4):
            continue
        x0, y0, x1, y1 = [float(v) * S for v in bbox]
        color = _hex_to_rgb(rg.get('color') or '#f0a75c')

        if want_box:
            # 半透明填充，让区域块能一眼看出范围
            draw.rectangle([x0, y0, x1, y1],
                           fill=color + (int(fill_alpha),))
            draw.rectangle([x0, y0, x1, y1],
                           outline=color + (255,), width=max(1, S // 2))

        name = str(rg.get('name', ''))
        if want_text and font is not None and name:
            tx, ty = x0 + 2 * S, y0 + 2 * S
            # 先描一圈描边色，保证字压在线上也能看清
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                draw.text((tx + dx, ty + dy), name, font=font,
                          fill=halo_rgba)
            draw.text((tx, ty), name, font=font, fill=(70, 40, 20))
    return note


def _hex_to_rgb(text):
    """'#rrggbb' → (r, g, b)。解析失败时返回描边默认色。"""
    s = str(text or '').strip().lstrip('#')
    if len(s) == 3:
        s = ''.join(c * 2 for c in s)
    if len(s) != 6:
        return (0xB8, 0x56, 0x2A)
    try:
        return (int(s[0:2], 16), int(s[2:4], 16), int(s[4:6], 16))
    except ValueError:
        return (0xB8, 0x56, 0x2A)
