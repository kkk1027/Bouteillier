"""
tools/web_tools.py
AI 可调用的联网/实时信息工具。

目前只接「结构化数据源」而不是开放网页搜索，原因：
  * 本项目本地跑的是 3B 小模型，上下文只有 8192，塞一张网页就爆；
  * 免 key 的通用搜索接口（DuckDuckGo Instant Answer 等）对中文查询
    基本返回空，HTML 端点还会被反爬拦（实测 HTTP 202 + 验证页）。
所以这里选择「精准 API + 短结果」，既省 context 又不容易答错。

天气数据来自 Open-Meteo（完全免费、无需 API key、国内直连约 0.9s）。
"""

import datetime
import json

import httpx
from langchain_core.tools import tool

# ============================================================
# 网络配置
# ------------------------------------------------------------
# 系统里存在 socks:// 代理（all_proxy=socks://127.0.0.1:17891），
# httpx 在没装 socksio 时解析它会直接抛 ValidationError。
# 实测 Open-Meteo 直连可用（约 0.9s），所以默认 trust_env=False
# 走直连；万一直连失败，再退到本地 HTTP 代理重试一次。
# ============================================================
_TIMEOUT = httpx.Timeout(12.0, connect=6.0)
_DIRECT = httpx.Client(trust_env=False, timeout=_TIMEOUT)
_VIA_PROXY = httpx.Client(
    trust_env=False,
    timeout=_TIMEOUT,
    proxy="http://127.0.0.1:17891",
)

_GEO_URL = "https://geocoding-api.open-meteo.com/v1/search"
_FORECAST_URL = "https://api.open-meteo.com/v1/forecast"

# WMO 天气代码 → 中文描述
_WMO = {
    0: "晴", 1: "晴间多云", 2: "多云", 3: "阴",
    45: "有雾", 48: "雾凇",
    51: "轻微毛毛雨", 53: "毛毛雨", 55: "浓密毛毛雨",
    56: "轻微冻雨", 57: "冻雨",
    61: "小雨", 63: "中雨", 65: "大雨",
    66: "轻微冻雨", 67: "强冻雨",
    71: "小雪", 73: "中雪", 75: "大雪", 77: "雪粒",
    80: "小阵雨", 81: "阵雨", 82: "强阵雨",
    85: "小阵雪", 86: "强阵雪",
    95: "雷阵雨", 96: "雷阵雨伴小冰雹", 99: "雷阵雨伴大冰雹",
}

# 常用城市兜底坐标：地理编码服务偶尔抽风时用，省一次请求
_FALLBACK_COORDS = {
    "北京": (39.9075, 116.39723),
    "上海": (31.22222, 121.45806),
    "广州": (23.11667, 113.25),
    "深圳": (22.54554, 114.0683),
    "杭州": (30.29365, 120.16142),
    "成都": (30.66667, 104.06667),
    "武汉": (30.58333, 114.26667),
    "西安": (34.25833, 108.92861),
    "南京": (32.06167, 118.77778),
    "重庆": (29.56278, 106.55278),
}


def _get(url, params):
    """先直连，失败再走本地代理。返回 (json_dict, 错误信息)。"""
    errors = []
    for tag, client in (("直连", _DIRECT), ("代理", _VIA_PROXY)):
        try:
            r = client.get(url, params=params)
            if r.status_code != 200:
                errors.append("%s HTTP %d" % (tag, r.status_code))
                continue
            return r.json(), ""
        except Exception as e:  # noqa: BLE001
            errors.append("%s: %s" % (tag, str(e)[:50]))
    return None, "网络不可达（%s）" % "；".join(errors)


def _geocode(city):
    """城市名 → (纬度, 经度, 规范名称)。"""
    data, err = _get(_GEO_URL, {
        "name": city, "count": 1, "language": "zh", "format": "json",
    })
    if data:
        results = data.get("results") or []
        if results:
            r0 = results[0]
            name = r0.get("name") or city
            admin = r0.get("admin1") or ""
            label = name if (not admin or admin == name) else "%s·%s" % (admin, name)
            return r0["latitude"], r0["longitude"], label

    if city in _FALLBACK_COORDS:
        lat, lon = _FALLBACK_COORDS[city]
        return lat, lon, city
    return None, None, err or ("找不到「%s」这个城市" % city)


@tool
def get_weather(city: str = "") -> str:
    """查天气：当前天气 + 未来三天预报。

    用户问天气、下不下雨、冷不冷时用。city 填中文城市名，
    用户没说城市就留空，用默认城市。
    """
    import os

    city = (city or "").strip()
    if not city:
        city = os.environ.get("FISHBOT_DEFAULT_CITY", "北京").strip() or "北京"

    lat, lon, label = _geocode(city)
    if lat is None:
        return "❌ 查天气失败：%s" % label

    data, err = _get(_FORECAST_URL, {
        "latitude": lat,
        "longitude": lon,
        "current": "temperature_2m,relative_humidity_2m,apparent_temperature,"
                   "weather_code,wind_speed_10m",
        "daily": "weather_code,temperature_2m_max,temperature_2m_min,"
                 "precipitation_probability_max",
        "timezone": "auto",
        "forecast_days": 4,
    })
    if not data:
        return "❌ 查天气失败：%s" % err

    cur = data.get("current") or {}
    code = cur.get("weather_code")
    desc = _WMO.get(code, "未知")

    lines = [
        "🌤️ %s 当前：%s，气温 %.0f°C（体感 %.0f°C），"
        "湿度 %d%%，风速 %.0f km/h" % (
            label, desc,
            cur.get("temperature_2m", 0),
            cur.get("apparent_temperature", 0),
            cur.get("relative_humidity_2m", 0),
            cur.get("wind_speed_10m", 0),
        )
    ]

    daily = data.get("daily") or {}
    dates = daily.get("time") or []
    codes = daily.get("weather_code") or []
    highs = daily.get("temperature_2m_max") or []
    lows = daily.get("temperature_2m_min") or []
    rains = daily.get("precipitation_probability_max") or []

    weekday = ["周一", "周二", "周三", "周四", "周五", "周六", "周日"]
    today = datetime.date.today()
    for i in range(min(3, len(dates))):
        try:
            d = datetime.date.fromisoformat(dates[i])
        except Exception:  # noqa: BLE001
            d = today + datetime.timedelta(days=i)
        tag = "今天" if d == today else weekday[d.weekday()]
        rain = rains[i] if i < len(rains) and rains[i] is not None else 0
        parts = "%s %s %.0f~%.0f°C" % (
            tag,
            _WMO.get(codes[i], "未知") if i < len(codes) else "未知",
            lows[i] if i < len(lows) else 0,
            highs[i] if i < len(highs) else 0,
        )
        if rain:
            parts += "，降水概率 %d%%" % rain
        lines.append("　" + parts)

    lines.append("（数据来源：Open-Meteo）")
    return "\n".join(lines)


@tool
def get_current_datetime() -> str:
    """获取当前日期、时间、星期。

    用户问"今天几号/几点/星期几"，或话题涉及"今天/今年"时先调此工具，
    不要凭训练记忆猜日期。
    """
    now = datetime.datetime.now()
    weekday = ["周一", "周二", "周三", "周四", "周五", "周六", "周日"]
    return "🕐 现在是 %s %s（%s）" % (
        now.strftime("%Y年%m月%d日"),
        now.strftime("%H:%M"),
        weekday[now.weekday()],
    )


WEB_TOOLS = [get_weather, get_current_datetime]
