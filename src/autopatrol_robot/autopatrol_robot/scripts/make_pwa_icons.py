#!/usr/bin/env python3
"""生成 PWA 图标 —— 图案是网页里那个管家机器人的**头部**。

只在需要重新生成图标时手动跑一次：

    python3 scripts/make_pwa_icons.py

产出写入 static/icons/：
    icon-192.png              Android / manifest
    icon-512.png              manifest
    icon-maskable-512.png     Android 自适应图标（图案收在中心安全圆内）
    apple-touch-icon-180.png  iOS 主屏图标
    icon-1024.png             备用大图

----------------------------------------------------------------------
造型来源：templates/control.html 里 .butler-unit 的 CSS
----------------------------------------------------------------------
下面所有坐标都是**照着 control.html 里那些 CSS 数值一比一搬过来的**，
不是凭感觉画的。头部各部件在 CSS 里的原始尺寸（单位 px）：

    .bt-head      68 x 60   radius 30   #fffdf8 → #f7ead9
    .bt-visor     50 x 34   radius 15   #54422f → #3a2c20
    .bt-eye       10 x 10   圆          #ffd97a（+ 白色高光 3x3）
    .bt-mouth     11 x 3    下圆角      #ffd97a
    .bt-antenna    3 x 13   圆          #d9b48f
    .bt-antenna::after 10 x 10 圆       var(--w-brand) = #f08c3c

换算成「以头部左上角为原点」的坐标系后，各部件位置：

    .bt-head      top 14   → 头部原点在 unit 坐标系的 (26, 14)
    .bt-antenna   top  4   → 相对头部顶端 **高 10px**（不是从 0 开始！）

    头部       (0,    0) - (68,  60)
    面罩       (9,   16) - (59,  50)      ← 相对头部 left 50% -25px, top 16px
    左眼       (18,  26) - (28,  36)
    右眼       (40,  26) - (50,  36)
    嘴         (28.5, 42) - (39.5, 45)
    天线杆     (32.5,-10) - (35.5,  3)     ← unit 坐标 4..17 减掉头部的 14
    天线球     (29,  -18) - (39,   -8)     ← 整个悬在头顶之上

因此整体包围盒是 **68 宽 x 78 高**（y 从 -18 到 60）。
注意高度大于宽度，所以缩放要按高度算，否则天线球会被裁掉。

----------------------------------------------------------------------
两处**刻意的偏离**（都是有原因的，不是画错）
----------------------------------------------------------------------
1. 天线球的颜色从 #f08c3c（品牌橙）改成暖金 #ffd97a。
   原因：品牌橙和暖色背景几乎同色，球会**完全看不见**。
   改成和眼睛同款的暖金色，既看得清，又和眼睛形成呼应。

2. 眼睛保留了一圈柔光（CSS 里本来就有 box-shadow 0 0 6px）。
   在小尺寸图标上这圈光让机器人「有神」，属于原设计的意图。

----------------------------------------------------------------------
兼容性要点
----------------------------------------------------------------------
* 背景满幅铺满、**不自己切圆角** —— iOS 和 Android 都会套自己的遮罩，
  自己再切一次会露出白边。
* maskable 版本把图案缩到 52%，保证落在 Android 的安全圆内
  （半径 0.4 × 边长），否则圆形遮罩会把天线裁掉。
* 4 倍超采样再 LANCZOS 缩放，避免圆角锯齿。
"""

import os
import sys

from PIL import Image, ImageDraw, ImageFilter

SS = 4  # 超采样倍率

# ---------------------------------------------------------------------------
# 配色：全部取自 control.html，改动时请同步那边的 CSS
# ---------------------------------------------------------------------------
BG_TOP = (0xFF, 0xB8, 0x77)     # 暖橙，比 --w-brand 略亮，做背景更通透
BG_BOT = (0xF0, 0x87, 0x3A)     # --w-brand #f08c3c

HEAD_TOP = (0xFF, 0xFD, 0xF8)   # .bt-head 渐变上端 #fffdf8
HEAD_BOT = (0xF7, 0xEA, 0xD9)   # .bt-head 渐变下端 #f7ead9

VISOR_TOP = (0x54, 0x42, 0x2F)  # .bt-visor 渐变上端 #54422f
VISOR_BOT = (0x3A, 0x2C, 0x20)  # .bt-visor 渐变下端 #3a2c20

EYE = (0xFF, 0xD9, 0x7A)        # .bt-eye #ffd97a
ANT_STICK = (0xD9, 0xB4, 0x8F)  # .bt-antenna #d9b48f

SHADOW = (150, 105, 62)         # .bt-head 的 box-shadow 色

# ---------------------------------------------------------------------------
# 几何：头部局部坐标系（单位与 CSS 的 px 一致）
# ---------------------------------------------------------------------------
HEAD = (0.0, 0.0, 68.0, 60.0)
VISOR = (9.0, 16.0, 59.0, 50.0)
EYE_L = (18.0, 26.0, 28.0, 36.0)
EYE_R = (40.0, 26.0, 50.0, 36.0)
MOUTH = (28.5, 42.0, 39.5, 45.0)
# ★ 天线在 CSS 里是 top:4px，而头部是 top:14px，两者共用 .butler-unit
#   的坐标系。换算到「头部原点」坐标系要减 14，所以天线杆从 -10 开始，
#   天线球（ant::after top:-8px）整个悬在头顶之上。
ANT = (32.5, -10.0, 35.5, 3.0)
ANT_BALL = (29.0, -18.0, 39.0, -8.0)

# 整体包围盒（含天线球伸出头部的部分）
DESIGN_W = 68.0
DESIGN_H = 78.0                 # -18 .. 60
BBOX_CX = 34.0                  # 0 .. 68 的中点
BBOX_CY = 21.0                  # (-18 + 60) / 2


def _lerp(a, b, t):
    return tuple(int(round(a[i] + (b[i] - a[i]) * t)) for i in range(3))


def _vgradient(size, top, bottom):
    """竖向线性渐变图。"""
    img = Image.new('RGB', (size, size))
    d = ImageDraw.Draw(img)
    for y in range(size):
        d.line([(0, y), (size, y)], fill=_lerp(top, bottom, y / (size - 1)))
    return img


def _shape_mask(size, box, mapper, radius=None, unit=1.0):
    """把设计坐标里的形状画成 L 模式蒙版。

    radius=None  → 画圆/椭圆（对应 CSS 的 border-radius: 50%）
    radius=数值  → 画圆角矩形，radius 按 unit 换算成像素
    """
    mask = Image.new('L', (size, size), 0)
    d = ImageDraw.Draw(mask)
    x0, y0 = mapper(box[0], box[1])
    x1, y1 = mapper(box[2], box[3])

    if radius is None:
        d.ellipse([x0, y0, x1, y1], fill=255)
    else:
        d.rounded_rectangle([x0, y0, x1, y1], radius=radius * unit, fill=255)
    return mask


def _glow(size, box, mapper, color, blur, strength=1.0):
    """生成一圈柔光层（RGBA），对应 CSS 里的 box-shadow: 0 0 Npx。"""
    mask = _shape_mask(size, box, mapper, radius=None)
    mask = mask.filter(ImageFilter.GaussianBlur(blur))
    if strength != 1.0:
        mask = mask.point(lambda v: int(v * strength))
    layer = Image.new('RGB', (size, size), color).convert('RGBA')
    layer.putalpha(mask)
    return layer


def make_icon(size, maskable=False):
    """画「管家机器人头部」图标。"""
    S = size * SS

    # maskable 要躲开 Android 的圆形安全区（半径 0.4 x 边长），图案必须更小。
    # 注意包围盒是 68x78（高 > 宽），所以按**高度**算缩放比例，
    # 否则天线球会超出画布被裁掉。
    k = 0.54 if maskable else 0.84
    unit = S * k / DESIGN_H     # 1 个设计单位 = 多少像素

    def T(x, y):
        """设计坐标 → 像素坐标（把包围盒中心对齐到画布中心）。"""
        return (S / 2 + (x - BBOX_CX) * unit,
                S / 2 + (y - BBOX_CY) * unit)

    # ---------------- 1. 背景：暖橙竖向渐变 ----------------
    img = _vgradient(S, BG_TOP, BG_BOT).convert('RGBA')

    # 顶部一片柔和的暖光，让背景不那么平
    halo = Image.new('L', (S, S), 0)
    ImageDraw.Draw(halo).ellipse(
        [-S * 0.25, -S * 0.62, S * 1.25, S * 0.45], fill=70)
    halo = halo.filter(ImageFilter.GaussianBlur(S * 0.10))
    img.paste(Image.new('RGB', (S, S), (0xFF, 0xE0, 0xC0)), (0, 0), halo)

    # ---------------- 2. 头部投影（对应 box-shadow 0 3px 10px）----------------
    head_sh = (HEAD[0], HEAD[1] + 3, HEAD[2], HEAD[3] + 3)
    sh = _shape_mask(S, head_sh, T, radius=30, unit=unit)
    sh = sh.filter(ImageFilter.GaussianBlur(10 * unit))
    sh = sh.point(lambda v: int(v * 0.30))
    sh_img = Image.new('RGB', (S, S), SHADOW).convert('RGBA')
    sh_img.putalpha(sh)
    img = Image.alpha_composite(img, sh_img)

    # ---------------- 3. 头部本体（奶白渐变 + 30 圆角）----------------
    img.paste(_vgradient(S, HEAD_TOP, HEAD_BOT), (0, 0),
              _shape_mask(S, HEAD, T, radius=30, unit=unit))

    # ---------------- 4. 面罩（深咖渐变 + 15 圆角）----------------
    img.paste(_vgradient(S, VISOR_TOP, VISOR_BOT), (0, 0),
              _shape_mask(S, VISOR, T, radius=15, unit=unit))

    # ---------------- 5. 眼睛：金色发光 + 白色高光 ----------------
    for eye in (EYE_L, EYE_R):
        img = Image.alpha_composite(
            img, _glow(S, eye, T, EYE, blur=6 * unit, strength=0.95))
        img.paste(Image.new('RGB', (S, S), EYE), (0, 0),
                  _shape_mask(S, eye, T, radius=None))

    # 眼睛高光：CSS 里是 left/top 2.5px、3x3 的白色小圆
    for eye in (EYE_L, EYE_R):
        hx, hy = eye[0] + 2.5, eye[1] + 2.5
        hl = _shape_mask(S, (hx, hy, hx + 3, hy + 3), T, radius=None)
        hl = hl.point(lambda v: int(v * 0.92))
        img.paste(Image.new('RGB', (S, S), (255, 255, 255)), (0, 0), hl)

    # ---------------- 6. 嘴：暖金，下圆角（opacity .85）----------------
    mouth = _shape_mask(S, MOUTH, T, radius=1.5, unit=unit)
    mouth = mouth.point(lambda v: int(v * 0.88))
    img.paste(Image.new('RGB', (S, S), EYE), (0, 0), mouth)

    # ---------------- 7. 天线杆（CSS z-index 6，压在头上面）----------------
    img.paste(Image.new('RGB', (S, S), ANT_STICK), (0, 0),
              _shape_mask(S, ANT, T, radius=1.5, unit=unit))

    # ---------------- 8. 天线球：暖金光晕 ----------------
    img = Image.alpha_composite(
        img, _glow(S, ANT_BALL, T, EYE, blur=8 * unit, strength=0.85))
    img.paste(Image.new('RGB', (S, S), EYE), (0, 0),
              _shape_mask(S, ANT_BALL, T, radius=None))

    return img.convert('RGB').resize((size, size), Image.LANCZOS)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(here, '..', 'static', 'icons')
    out_dir = os.path.normpath(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    targets = [
        ('icon-192.png', 192, False),
        ('icon-512.png', 512, False),
        ('icon-maskable-512.png', 512, True),
        ('apple-touch-icon-180.png', 180, False),
        # iOS 启动画面用不到自定义图，但留一张 1024 备用（App Store/文档）
        ('icon-1024.png', 1024, False),
    ]

    for name, size, maskable in targets:
        path = os.path.join(out_dir, name)
        make_icon(size, maskable).save(path, 'PNG', optimize=True)
        print(f'  ✓ {name:28s} {size}x{size}'
              f'{"  (maskable)" if maskable else ""}  '
              f'{os.path.getsize(path):,} bytes')

    print(f'\n图标输出目录: {out_dir}')


if __name__ == '__main__':
    sys.exit(main())
