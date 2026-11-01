/* ==========================================================================
 * Bouteillier Home — PWA 增强层
 * ==========================================================================
 *
 * 这里放的全是**可选能力**：任何一项失败都只是少一点体验，
 * 页面本身照常工作，绝不会报错中断。
 *
 * 三个关键现实（决定了下面的写法）：
 *
 *   1. 当前访问地址是 http://<局域网IP>:5000，不是 secure context。
 *      → Service Worker 注册会被浏览器**直接拒绝**
 *      → Screen Wake Lock 也不可用
 *      这两处都必须 try/catch 静默降级，否则控制台报错会让人以为页面坏了。
 *
 *   2. iOS 的「添加到主屏幕」并不依赖上面两个 API。
 *      它靠的是 control.html 里的 apple-mobile-web-app-capable，
 *      **不要求 HTTPS** —— 所以在明文局域网地址下依然能全屏运行。
 *
 *   3. 屏幕常亮在 HTTP 下拿不到 API 支持，
 *      替代做法是系统设置里把「自动锁定」改成「永不」
 *      （设置 → 显示与亮度 → 自动锁定 → 永不）。
 * ========================================================================== */

(function () {
    'use strict';

    var root = document.documentElement;

    // ------------------------------------------------------------------
    // 1. 标记当前运行模式
    //    从主屏图标启动 → display-mode 为 standalone（iOS 用私有属性
    //    navigator.standalone），加个 class 方便 CSS 针对性微调。
    // ------------------------------------------------------------------
    try {
        var standalone =
            (window.matchMedia &&
                window.matchMedia('(display-mode: standalone)').matches) ||
            window.navigator.standalone === true;

        root.classList.add(standalone ? 'pwa-standalone' : 'pwa-browser');
    } catch (e) {
        /* 忽略 */
    }

    // ------------------------------------------------------------------
    // 2. Service Worker
    //    只在 HTTPS / localhost 下才有意义。HTTP 环境下注册失败是
    //    预期行为，用 info 级别提示，不要用 error 吓人。
    // ------------------------------------------------------------------
    if ('serviceWorker' in navigator) {
        window.addEventListener('load', function () {
            navigator.serviceWorker
                .register('/sw.js', { scope: '/' })
                .then(function () {
                    console.info('[PWA] Service Worker 已启用');
                })
                .catch(function (err) {
                    console.info(
                        '[PWA] Service Worker 未启用（HTTP 局域网地址下属正常现象，'
                        + '需 HTTPS 才会生效）:',
                        (err && err.message) || err
                    );
                });
        });
    }

    // ------------------------------------------------------------------
    // 3. 屏幕常亮（尽力而为）
    // ------------------------------------------------------------------
    var wakeLock = null;
    var wanted = false;

    function acquireWakeLock() {
        if (!wanted || wakeLock) return;
        if (!('wakeLock' in navigator)) return;      // 不支持 → 静默跳过

        navigator.wakeLock
            .request('screen')
            .then(function (lock) {
                wakeLock = lock;
                // 系统可能在息屏/切后台时自动释放，这里跟着清掉本地引用
                lock.addEventListener('release', function () {
                    wakeLock = null;
                });
                console.info('[PWA] 屏幕常亮已生效');
            })
            .catch(function (err) {
                console.info(
                    '[PWA] 屏幕常亮不可用（需 HTTPS）:', (err && err.message) || err
                );
            });
    }

    function releaseWakeLock() {
        if (!wakeLock) return;
        try {
            wakeLock.release();
        } catch (e) {
            /* 忽略 */
        }
        wakeLock = null;
    }

    // 用户一碰屏幕就尝试申请（Wake Lock 必须由用户手势触发才算数）
    function onFirstGesture() {
        wanted = true;
        acquireWakeLock();
        document.removeEventListener('pointerdown', onFirstGesture);
        document.removeEventListener('touchstart', onFirstGesture);
        document.removeEventListener('click', onFirstGesture);
    }
    document.addEventListener('pointerdown', onFirstGesture, { passive: true });
    document.addEventListener('touchstart', onFirstGesture, { passive: true });
    document.addEventListener('click', onFirstGesture);

    // 切回前台时系统会释放锁，需要重新申请
    document.addEventListener('visibilitychange', function () {
        if (document.visibilityState === 'visible') {
            acquireWakeLock();
        } else {
            releaseWakeLock();
        }
    });

    // ------------------------------------------------------------------
    // 4. 在浏览器里打开时，提示一下可以装到主屏
    //    （只在控制台打一行，不弹窗打扰）
    // ------------------------------------------------------------------
    if (!root.classList.contains('pwa-standalone')) {
        console.info(
            '[PWA] 提示：Safari 里点「分享 → 添加到主屏幕」，'
            + '以后就能像 App 一样全屏打开，且不受签名过期影响。'
        );
    }
})();
