#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""页面 2/3 布局验证脚本（只读 UI，不碰 ROS、不发运动指令）。

用法：
    python3 scripts/check_layout.py --page 2 --w 1280 --h 620
    python3 scripts/check_layout.py --page 3 --w 390 --h 844
    python3 scripts/check_layout.py --page 2 --shot /tmp/p2.png
"""
import argparse
import sys

from playwright.sync_api import sync_playwright

BASE = 'http://127.0.0.1:5051'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--page', default='2', choices=['1', '2', '3'])
    ap.add_argument('--w', type=int, default=1280)
    ap.add_argument('--h', type=int, default=620)
    ap.add_argument('--base', default=BASE)
    ap.add_argument('--shot', default='')
    ap.add_argument('--wait', type=int, default=4600)
    args = ap.parse_args()

    with sync_playwright() as pw:
        b = pw.chromium.launch()
        pg = b.new_page(viewport={'width': args.w, 'height': args.h})
        pg.goto('%s/?guide=%s' % (args.base, args.page),
                wait_until='domcontentloaded')
        pg.wait_for_timeout(900)

        # 引导/工作台都必须先「连接」。这一步只在浏览器里点按钮，
        # 不会碰 ROS（5051 是只读联调实例，car_controller is None）。
        clicked = pg.evaluate("""() => {
            const b = [...document.querySelectorAll('button')]
                .find(x => x.textContent.includes('连接以开始'));
            if (!b) return false;
            b.click();
            return true;
        }""")
        print('点击「连接以开始」：%s' % clicked)

        pg.wait_for_timeout(args.wait)

        info = pg.evaluate("""() => {
            const $ = id => document.getElementById(id);
            const r = el => { if (!el) return null;
                const b = el.getBoundingClientRect();
                return { x: Math.round(b.x), y: Math.round(b.y),
                         w: Math.round(b.width), h: Math.round(b.height) }; };
            const layer = $('widget-layer');
            const jw = $('widget-joystick');
            const mw = $('widget-map');
            const bt = document.querySelector('.view-home .butler');
            const gs = $('guideBotStage');
            const out = {
                vw: innerWidth, vh: innerHeight,
                layerClass: layer ? layer.className : null,
                layerDisplay: layer ? getComputedStyle(layer).display : null,
                layerDir: layer ? getComputedStyle(layer).flexDirection : null,
                layerJustify: layer ? getComputedStyle(layer).justifyContent : null,
                layerAlign: layer ? getComputedStyle(layer).alignItems : null,
                joystick: r(jw), map: r(mw),
                bothOpen: !!(jw && mw && jw.classList.contains('open')
                             && mw.classList.contains('open')),
                butler: bt ? { cls: bt.className,
                               display: getComputedStyle(bt).display,
                               visibility: getComputedStyle(bt).visibility,
                               opacity: getComputedStyle(bt).opacity,
                               rect: r(bt) } : null,
                botStage: gs ? { cls: gs.className,
                                 display: getComputedStyle(gs).display,
                                 visibility: getComputedStyle(gs).visibility,
                                 opacity: getComputedStyle(gs).opacity,
                                 rect: r(gs) } : null,
            };
            if (jw && mw) {
                const a = jw.getBoundingClientRect(), c = mw.getBoundingClientRect();
                out.gap = Math.round(c.left - a.right);
                out.centerX = Math.round((a.left + c.right) / 2);
                out.centerY = Math.round((a.top + c.bottom) / 2);
                out.overlap = !(a.right <= c.left || c.right <= a.left
                                || a.bottom <= c.top || c.bottom <= a.top);
            }
            return out;
        }""")

        print('视口：%dx%d' % (args.w, args.h))
        for k, v in info.items():
            print('  %-14s %s' % (k, v))

        if args.shot:
            pg.screenshot(path=args.shot)
            print('\n截图已存: %s' % args.shot)

        b.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
