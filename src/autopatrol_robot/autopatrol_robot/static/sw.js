/* eslint-disable no-restricted-globals */
/* ==========================================================================
 * Bouteillier Home — Service Worker（刻意写得很保守）
 * ==========================================================================
 *
 * 设计原则：**只缓存图标，绝不缓存页面和接口。**
 *
 * 原因：
 *   这个项目是「改完 HTML 立刻在真机上看效果」的迭代方式，
 *   web_controller.py 里甚至专门打开了 TEMPLATES_AUTO_RELOAD
 *   （见那里的注释：曾经因为模板缓存排查了很久「改了没生效」）。
 *   如果 SW 把 `/` 缓存下来，就会重新制造一模一样的坑，
 *   而且更隐蔽 —— 刷新、强制刷新都可能是旧页面，只有清站点数据才好。
 *
 *   所以 fetch 处理器里，非图标请求**什么都不做**（不调用 respondWith），
 *   浏览器按默认行为走网络。这样 SW 的存在只带来两个好处：
 *     1. Android Chrome 才认为这是「可安装的 PWA」
 *     2. 主屏图标 / manifest 秒开，不依赖网络
 *
 * 安全：绝不拦截 `/control`（会发 /cmd_vel）、`/socket.io` 等，
 *       避免任何控制指令被缓存或重放。
 * ========================================================================== */

// 改了图标或 vendor 文件就改这里，activate 时旧缓存会被整批删掉。
// v2: 图标换成管家机器人头部
const VERSION = 'v2';
const ICON_CACHE = `bouteillier-icons-${VERSION}`;

// 预缓存清单：全部是静态资源，改了就升 VERSION
const PRECACHE = [
    '/manifest.json',
    '/static/vendor/socket.io.min.js',
    '/static/icons/icon-192.png',
    '/static/icons/icon-512.png',
    '/static/icons/icon-maskable-512.png',
    '/static/icons/apple-touch-icon-180.png',
];

// 只有命中这些前缀的请求才走 SW 缓存，其余全部直连网络
// 注意：/static/vendor/ 里的文件带版本号且基本不变，
//       如果要换 socket.io 版本，必须同时改 VERSION 常量，否则会拿到旧文件。
const CACHEABLE_PREFIXES = ['/static/icons/', '/static/vendor/'];

// ---------------------------------------------------------------------------
// install：预缓存图标，并立即进入 waiting→active，不等旧页面关闭
// ---------------------------------------------------------------------------
self.addEventListener('install', (event) => {
    event.waitUntil((async () => {
        const cache = await caches.open(ICON_CACHE);
        // 逐个 add 而不是 addAll：任何一个 404 不应该让整批失败
        await Promise.all(PRECACHE.map(async (url) => {
            try {
                await cache.add(new Request(url, { cache: 'reload' }));
            } catch (err) {
                console.warn('[sw] 预缓存失败（忽略）:', url, err);
            }
        }));
        await self.skipWaiting();
    })());
});

// ---------------------------------------------------------------------------
// activate：清掉所有旧版本缓存，立刻接管页面
// ---------------------------------------------------------------------------
self.addEventListener('activate', (event) => {
    event.waitUntil((async () => {
        const keys = await caches.keys();
        await Promise.all(
            keys
                .filter((k) => k.startsWith('bouteillier-') && k !== ICON_CACHE)
                .map((k) => caches.delete(k))
        );
        await self.clients.claim();
    })());
});

// ---------------------------------------------------------------------------
// fetch：仅图标走 cache-first，其他一律放行
// ---------------------------------------------------------------------------
self.addEventListener('fetch', (event) => {
    const req = event.request;

    // 只处理 GET
    if (req.method !== 'GET') return;

    let url;
    try {
        url = new URL(req.url);
    } catch (e) {
        return;
    }

    // 跨域（例如 cdn.socket.io）不碰，交给浏览器
    if (url.origin !== self.location.origin) return;

    // 只接管图标和 manifest
    const isIcon = CACHEABLE_PREFIXES.some((p) => url.pathname.startsWith(p));
    const isManifest = url.pathname === '/manifest.json';
    if (!isIcon && !isManifest) return;   // ← 页面 / 接口 / socket.io 全部放行

    event.respondWith(cacheFirst(req));
});

async function cacheFirst(req) {
    const cache = await caches.open(ICON_CACHE);
    const hit = await cache.match(req);
    if (hit) return hit;

    try {
        const res = await fetch(req);
        // 只缓存正常的、同源的 200 响应
        if (res && res.ok && res.type === 'basic') {
            cache.put(req, res.clone());
        }
        return res;
    } catch (err) {
        console.warn('[sw] 取图标失败:', req.url, err);
        return Response.error();
    }
}

// ---------------------------------------------------------------------------
// message：允许页面主动请求「立刻更新」
// ---------------------------------------------------------------------------
self.addEventListener('message', (event) => {
    if (event.data === 'SKIP_WAITING') self.skipWaiting();
});
