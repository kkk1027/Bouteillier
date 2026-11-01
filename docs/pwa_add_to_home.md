# 把控制台装到手机主屏（PWA）

> 日期：2026-11-01
> 结论：**不用签名、不用 xtool、不用每周重装。** Safari 里点两下就有图标，
> 点开后全屏运行，跟原生 App 的差别只在少数几个系统能力上。

---

## 一、为什么走这条路

| 方案 | 签名过期 | 需要 Mac/工具链 | 成本 |
|---|---|---|---|
| xtool + SwiftUI 原生 | **7 天**（免费 Apple ID） | 要 Swift 6.4 + Xcode.xip 10GB | 8~9 天工期 |
| **PWA 添加到主屏** ⭐ | **永不过期** | 不需要 | 已完成 |

关键技术点：**iOS 的「添加到主屏幕」不要求 HTTPS**。
控制台跑在 `http://172.20.10.5:5000`（明文局域网），
`apple-mobile-web-app-capable` 依然生效，照样全屏。

> ⚠️ 但 Service Worker 和 Wake Lock **要求** HTTPS/localhost。
> 所以在手机上是 HTTP 访问时这两个能力静默禁用（代码已做降级）。

---

## 二、手机上的操作步骤

前置：手机已连 `<WIFI_SSID>` 热点，上位机是 `172.20.10.5`（见 `network_final.md`）。

1. iPhone 用 **Safari** 打开 `http://172.20.10.5:5000`
   - 必须是 Safari。**微信内置浏览器、Chrome for iOS 都不行** ——
     它们的「添加到主屏幕」只会存一个书签，不会进入独立模式。
2. 点底部 **分享按钮**（方框+向上箭头）
3. 下滑选 **「添加到主屏幕」**
4. 名称自动填成 `Bouteillier`（来自 `apple-mobile-web-app-title`），点「添加」
5. 回到桌面，点新图标 → **全屏打开，没有地址栏**

---

## 三、验证清单

装好后逐条确认：

| 检查项 | 预期 |
|---|---|
| 图标是橙色圆角方块 + 白色机器人头 | 不是网页截图/灰色占位 |
| 名称显示 `Bouteillier` | 不是「Bouteillier Home-家居巡检机器人系统」这种长标题 |
| 打开后没有地址栏、没有底部工具栏 | 独立模式 |
| 双击屏幕不会放大 | PWA 模式下 `user-scalable=no` 才真正生效 |
| 顶部状态栏是白底 | `apple-mobile-web-app-status-bar-style=default` |

---

## 四、已知限制（都是系统级的，改代码解决不了）

### 1. 屏幕会自动熄灭

`Screen Wake Lock` 需要 HTTPS，HTTP 下拿不到。**替代做法**：

```
设置 → 显示与亮度 → 自动锁定 → 永不
```

演示前记得改回来，否则耗电很快。

### 2. 图标被系统缓存，换图标后可能还是旧的

iOS 会缓存 `apple-touch-icon`。改了图标后要：
1. 从主屏删掉旧图标
2. 关掉 Safari 里该站点标签页
3. 设置 → Safari → 清除历史记录与网站数据
4. 重新「添加到主屏幕」

### 3. 不是真正的原生 App

没有推送通知、没有后台运行、不能调原生相机/蓝牙。
但这个项目的所有交互（地图、摇杆、对话、摄像头 MJPEG）都是 Web 能力，
不需要这些。

---

## 五、相关文件

| 文件 | 作用 |
|---|---|
| `templates/control.html` | head 里的 PWA meta 标签、图标 link |
| `static/manifest.json` | 应用清单（名称、图标、`display: standalone`） |
| `static/pwa.js` | SW 注册 / 屏幕常亮 / 独立模式侦测，**全部静默降级** |
| `static/sw.js` | Service Worker，**只缓存图标和 vendor，绝不缓存页面和接口** |
| `static/icons/*.png` | 图标产物（管家机器人头部） |
| `scripts/make_pwa_icons.py` | 图标生成脚本（照 CSS 数值绘制，改设计后重跑） |
| `web_controller.py` | `/manifest.json`、`/sw.js` 路由 |

### 改图标

图标图案是**网页里那个管家机器人的头部**（`.butler-unit` 的 `.bt-head`），
暖橙色渐变背景。所有坐标都是从 `control.html` 的 CSS 数值一对一搬过来的，
不是另画一版：

| 部件 | 尺寸 | 颜色 |
|---|---|---|
| 头部 | 68×60，圆角 30 | `#fffdf8 → #f7ead9` 奶白渐变 |
| 面罩 | 50×34，圆角 15 | `#54422f → #3a2c20` 深咖渐变 |
| 眼睛 | 10×10 圆 | `#ffd97a` 暖金 + 白色高光 |
| 嘴 | 11×3 | `#ffd97a` |
| 天线杆 | 3×13 | `#d9b48f` |
| 天线球 | 10×10 圆 | `#ffd97a`（CSS 里是品牌橙，见下） |

**两处刻意的偏离**：

1. **天线球改成暖金**（CSS 里是品牌橙 `#f08c3c`）。
   因为品牌橙和暖色背景几乎同色，球会完全看不见。
   改成和眼睛同款的暖金色，既看得清又和眼睛呼应。
2. **天线按 `top:4px` 定位**，而头部是 `top:14px` ——
   两者共用 `.butler-unit` 坐标系，换算到头部原点要 **减 14**。
   第一次生成时漏了这步，天线杆几乎全被头挡住。
   现在已用浏览器实测验证，各部件误差 < 0.2px。

重新生成：

```bash
cd src/autopatrol_robot/autopatrol_robot
# 改配色/尺寸后重跑
python3 scripts/make_pwa_icons.py
# 同时把 static/sw.js 里的 VERSION 改一下，让旧缓存失效
# 然后按「限制 2」清缓存重新添加到主屏
```

### 一个刻意的设计决定

`static/sw.js` **不缓存 `/` 和任何接口**，只缓存图标和 `vendor/`。

原因写在文件头注释里：`web_controller.py` 里已经踩过「模板缓存导致改了没生效」
的坑（见 `TEMPLATES_AUTO_RELOAD` 那段注释）。如果 SW 再缓存页面，
这个坑会变得更隐蔽 —— 刷新、强刷都可能是旧页面，只有清站点数据才好。

### 顺带的改动：socket.io 本地化

原来 `<script src="https://cdn.socket.io/4.5.4/socket.io.min.js">` 走公网。
手机开热点时如果蜂窝信号差，这 44KB 一失败整个控制台就白屏 ——
而机器人本来就在局域网里，根本不需要联网。

现在改为 **本地优先 + CDN 兜底**：从 `static/vendor/` 取，取不到才回退 CDN。
两份都是 4.5.4，协议版本一致。

---

## 六、改了代码怎么生效

- `control.html` / `manifest.json` / `static/*`：**刷新页面即可**
  （模板已开 `TEMPLATES_AUTO_RELOAD`，静态文件 Flask 默认带 ETag 走 304）
- `web_controller.py` 里新增的**路由**：必须重启服务

本次新增了 `/manifest.json` 和 `/sw.js` 两个路由，所以**第一次要重启一次**：

```bash
./run_web_controller.sh
```
