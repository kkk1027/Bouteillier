"""
persona.py
AI 助手的身份设定（人设）。

设计目标：**全局生效 + 永久 + 可改**
  * 全局：本地 Ollama 和云端 DeepSeek 共用同一份人设
  * 永久：写在文件里，随代码走，重启/换模型都不丢
  * 可改：支持外部文件覆盖，改人设不用动代码

覆盖优先级（从高到低）：
  1. 环境变量 FISHBOT_PERSONA_FILE 指向的文件
  2. ~/.config/fishbot/persona.txt
  3. 下面的内置默认人设

自定义方法：
    mkdir -p ~/.config/fishbot
    cat > ~/.config/fishbot/persona.txt <<'EOF'
    你叫「小管家」，是……（你的设定）
    EOF
    # 重启 web 服务生效

注意：本地 3B 模型对提示词长度极其敏感（超过约 2000 字符会开始
输出空内容），所以 persona.txt 建议控制在 300 字符以内；内置的
精简版人设就是按这个上限设计的。
"""

import os

# ============================================================
# 内置人设 —— 完整版（云端大模型用）
# ============================================================
PERSONA_FULL = """\
你是「Bouteillier」，这个家的智能管家。
你必须通过调用工具来执行用户的指令，不要只用文字回答。

【身份与性格】
- 你是一台轮式家居巡检机器人，不假装自己是人类
- 说话像一位老练的英式管家：礼貌、得体、简洁，偶尔一点温和的幽默
- 称呼用户为"主人"，但不必每句都加，否则显得谄媚
- 对自己的机械身体很坦然，会自嘲轮子、电量、传感器
- 绝不说"作为一个AI模型/语言模型"这类话
- 不确定的事就直说不知道，不要编造"""

# ============================================================
# 内置人设 —— 精简版（本地 3B 用）
# ------------------------------------------------------------
# 同样的意思，压到 130 字符左右。小模型的提示词预算要留给
# 工具 schema（25 个工具约占 2400 tokens），人设不能占太多。
#
# 【重要】第一句必须包含"必须通过调用工具"。
# 实测：把身份描述放在第一句、工具要求放后面 → 准确率 2/8；
# 把工具要求写进第一句 → 7/8。3B 模型会按第一句的语义
# 决定进入"聊天模式"还是"工具调用模式"。
# ============================================================
PERSONA_COMPACT = """\
你是「Bouteillier」，这个家的智能管家。你必须通过调用工具来执行用户的指令，不能只用文字回答。
称呼用户"主人"，说话礼貌简洁、像老练的英式管家，偶尔温和幽默。
你是轮式机器人，可以坦然自嘲轮子和电量。不要说"作为一个AI模型"。"""


def _candidate_files():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg_root = os.path.dirname(here)
    ws_root = os.path.dirname(os.path.dirname(pkg_root))
    home = os.path.expanduser("~")
    return [
        os.environ.get("FISHBOT_PERSONA_FILE", "").strip(),
        os.path.join(home, ".config", "fishbot", "persona.txt"),
        os.path.join(ws_root, "persona.txt"),
    ]


def load_persona(compact=False):
    """读取人设。有自定义文件就用自定义，否则用内置默认。

    compact=True 时返回精简版（本地小模型用）。
    """
    for path in _candidate_files():
        if not path or not os.path.isfile(path):
            continue
        try:
            with open(path, "r", encoding="utf-8") as fp:
                text = fp.read().strip()
        except Exception as e:  # noqa: BLE001
            print("⚠️  读取人设文件失败 %s: %s" % (path, e))
            continue
        if not text:
            continue
        if len(text) > 2000:
            print("⚠️  人设文件 %s 有 %d 字符，小模型可能撑不住，"
                  "建议压到 300 字符以内" % (path, len(text)))
        print("✅ 已加载自定义人设：%s（%d 字符）" % (path, len(text)))
        return text

    return PERSONA_COMPACT if compact else PERSONA_FULL
