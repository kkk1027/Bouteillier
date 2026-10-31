# 本地轻量模型部署（零 API 费用）

用本地小模型替代 DeepSeek 云 API 做工具调用，省下 API 开销。
本机实测：**24 条测试题 100% 命中，平均 1.2 秒/条**。

> ⚠️ **先读这一节**：2026-10-31 这套部署把整台电脑搞死机过一次。
> 原因和修复见第四节「坑 0」，相关参数**不要随意调回去**。

---

## 一、结论先说

| 项目 | 值 |
|---|---|
| 推理引擎 | Ollama 0.40.13（用户目录安装，不需要 sudo） |
| 模型 | `fishbot-qwen:3b`（基于 Qwen2.5-3B-Instruct，Q4 量化） |
| 模型体积 | 1.9 GB |
| 显存占用 | 约 2.5 GB / 4 GB（**100% GPU，不再溢出到 CPU**） |
| 系统内存 | 模型常驻约 0.3 GB + prompt 缓存（约 36MB/轮） |
| 单轮延迟 | 1.1 ~ 1.2 秒 |
| 工具选择准确率 | 24/24 = 100%（19 个工具） |
| 费用 | 0 |

为什么不用 DeepSeek 小模型：官方没有 3B 级别开源模型，
最小的也是 16B 以上，4 GB 显存放不下。千问有完整的 0.5B/1.5B/3B 系列，
所以选千问。

为什么不用 1.5B：本项目的工具多达 **25 个**，1.5B 在这种规模下
选错工具的概率明显上升。3B 是能稳定工作的最小尺寸。

---

## 二、目录结构

```
~/.local/ollama/
├── bin/ollama                 可执行文件
├── lib/ollama/                CUDA 加速库（cuda_v12 / cuda_v13）
├── models/                    模型权重（1.8 GB）
└── modelfiles/
    └── fishbot-qwen.Modelfile 自定义模型定义
```

工作空间内：

```
scripts/local_llm.sh                                    管理脚本
src/autopatrol_robot/autopatrol_robot/tools/local_llm_selftest.py   自检
```

---

## 三、日常使用

### 正常启动（推荐）

```bash
./run_web_controller.sh
```

脚本会自动：探测本地模型 → 没启动就拉起 Ollama → 优先使用本地模型。
本地模型不可用时才回退到 DeepSeek 云 API（需要配 key）。

### 手动管理

```bash
./scripts/local_llm.sh start     # 启动服务（已在跑则跳过）
./scripts/local_llm.sh status    # 查看状态 + 显存 + 模型内存
./scripts/local_llm.sh warm      # 预热，提前加载模型
./scripts/local_llm.sh guard     # 检查内存，超标就卸载模型
./scripts/local_llm.sh unload    # 手动卸载模型（释放缓存与显存）
./scripts/local_llm.sh watch     # 启动后台看门狗（每 60 秒自动 guard）
./scripts/local_llm.sh test      # 跑 24 条工具调用自检
./scripts/local_llm.sh stop      # 停止服务
```

### 切换模型来源

```bash
# 默认：自动（本地优先，不可用则云端）
./run_web_controller.sh

# 强制本地，本地起不来就报错退出
FISHBOT_LLM=local ./run_web_controller.sh

# 强制云端 DeepSeek
FISHBOT_LLM=deepseek ./run_web_controller.sh
```

### 环境变量速查

| 变量 | 默认值 | 说明 |
|---|---|---|
| `FISHBOT_LLM` | `auto` | `auto` / `local` / `deepseek` |
| `FISHBOT_LOCAL_MODEL` | `fishbot-qwen:3b` | 换模型时改这里 |
| `OLLAMA_BASE_URL` | `http://127.0.0.1:11434/v1` | Ollama 的 OpenAI 兼容地址 |
| `FISHBOT_MAX_HISTORY` | `12` | 送给模型的最近消息条数上限 |

---

## 四、踩过的坑（重要）

部署过程中遇到四个问题，都已经在代码里修掉，这里记录下来免得重蹈覆辙。

### 坑 0（最严重）：把整台电脑搞到死机

**现象**：连续跑了几十次工具调用测试后，屏幕卡死，日志在 14:50 突然完全停止，
14:56 手动硬重启。没有 OOM killer、没有 kernel panic、没有 GPU Xid 报错。

**证据**（`/var/log/kern.log`）：

```
llama-server: vmemmap alloc failure: order:9, mode:0x4cc0(...)
Node 0 DMA free:13316kB        ← 只剩 13MB 系统内存
active_anon:7496332kB          ← 7.5GB 匿名内存
NVRM: GPU0 ... Out of memory [NV_ERR_NO_MEMORY]   ← 连续 8 次
（7 分钟后）
nvidia 0000:01:00.0: Using 39-bit DMA addresses   ← GPU 驱动重新初始化
```

**根因（两个叠加）**：

1. **显存耗尽**：`num_ctx=8192` 时，模型权重 1.9GB + KV cache 0.8GB +
   Xorg/GNOME 0.6GB ≈ 3.3GB，正好顶满 4GB 显存上限。
   GPU 驱动连内部的 fault buffer 都分配不出来，反复报 OOM 后重新初始化。
2. **内存泄漏（真正推手）**：Ollama 会为**每一个不同的 prompt** 缓存一份
   完整 KV cache。实测 f16 下**每条约 70MB**，上限按内存推算高达 **8192 MiB**：

   ```
   cache state: 72 prompts, 5041.843 MiB (limits: 8192.000 MiB, 4096 tokens)
   ```

   我当时背靠背跑了 100 多次测试 → 缓存吃到 8.4GB → 15GB 内存
   （**且 0B swap**）被吃干净 → 内核连 order-9 连续页都分配不出来
   → GPU 驱动分配失败 → 整机硬死机。

**修复（四层防护，全部固化在 `scripts/local_llm.sh`）**：

| 措施 | 效果 |
|---|---|
| `num_ctx` 8192 → **4096** | KV cache 减半，KV 总算 0.4GB，显存留出 1GB 余量 |
| `OLLAMA_KV_CACHE_TYPE=q8_0` | prompt 缓存每条 **70MB → 36MB**，准确率不变 |
| `OLLAMA_GPU_OVERHEAD=512MB` | 给桌面环境预留显存，驱动不会再分配失败 |
| `OLLAMA_KEEP_ALIVE=10m` | 空闲 10 分钟自动卸载模型，缓存全部释放 |
| 后台看门狗（`local_llm.sh watch`） | 每 60 秒检查，模型内存 >3500MB 或系统可用 <1500MB 就卸载 |

修复后实测：

```
100% GPU（不再溢出到 CPU）  显存 2523 MiB / 4096，空闲 1.5GB
模型内存 344MB（基线）      系统可用 10.7GB
连跑 72 次后 3027MB → guard 自动卸载 → 可用内存回到 10.6GB
```

**给用户的建议（需要 sudo，代码里没做）**：加个 swap 分区。
15GB 内存 + 无 swap + 跑模型 + ROS + Chrome，本来就很危险：

```bash
sudo fallocate -l 8G /swapfile && sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### 坑 1：提示词太长，模型直接「装死」


**现象**：模型返回空字符串，`tool_calls` 也是空的，耗时只有 0.1 秒。

**原因**：原来的 `SYSTEM_PROMPT` 是给云端大模型写的，把 23 个工具的说明
又完整抄了一遍；而工具描述本身会通过 `bind_tools` 的 JSON schema 再发一次。
两者叠加后 prompt 达到 **3254 tokens**，3B 模型直接输出 EOS。

对照实验：

| 提示词 | prompt tokens | 结果 |
|---|---|---|
| 完整版（2001 字符） | 3254 | 6 次全部返回空 |
| 精简版（545 字符） | 约 2400 | 6 次全部正确 |

**修复**：新增 `SYSTEM_PROMPT_COMPACT`，本地模型专用，只保留「指令→工具」
的映射和输出风格要求，不再重复描述工具。云端仍用完整版。

相关代码在 `agent_graph.py`：

```python
ACTIVE_SYSTEM_PROMPT = (SYSTEM_PROMPT_COMPACT if LLM_PROVIDER == "local"
                       else SYSTEM_PROMPT)
```

### 坑 2：示例文本被当成答案抄

**现象**：用户说「带我回去」，模型不调用工具，直接回了一句
「正在前往起点」。

**原因**：精简提示词里原本写着

```
5. 导航类工具是异步的：调用后直接告诉用户"正在前往"，不要等待
```

3B 模型把引号里的「正在前往」当成了**要模仿的标准答案**照抄出来，
而跳过了调用工具这一步。

**修复**：删掉示例措辞，改成反面约束：

```
5. 导航类工具是异步的，调用后立刻就有结果返回，不要等待
注意：不要用文字描述"正在前往"来代替调用工具——必须先调用工具再有回复。
```

> 给 3B 模型写提示词时，**不要用引号举例输出格式**，它会直接抄。
> 要么用文字描述行为，要么用反面约束。

### 坑 3：多轮之后历史把上下文撑爆

**现象**：聊到第五六轮时，模型又开始返回空。

**原因**：LangGraph 的 `InMemorySaver` 会累积全部历史，
工具调用和工具结果都会保留，几轮之后就超上下文。

**修复**：`agent_node` 里加了 `_trim_history()`，只保留最近 12 条，
并且**保证 tool_call 与 tool_result 成对**——只留 AIMessage(tool_calls)
却丢掉对应的 ToolMessage，OpenAI 兼容接口会直接 400。

另外加了空回复兜底：一旦检测到「既没有 tool_calls 也没有文本」，
自动去掉历史用最后一句话重试一次。

---

## 五、自定义模型参数

`~/.local/ollama/modelfiles/fishbot-qwen.Modelfile`：

```
FROM qwen2.5:3b
PARAMETER num_ctx 4096        # 不要调回 8192，会顶爆 4GB 显存（见坑 0）
PARAMETER temperature 0.2     # 工具调用要确定性，压低温度
PARAMETER top_p 0.9
PARAMETER repeat_penalty 1.05
PARAMETER num_predict 256     # 回复本来就一句话，限上限防异常时狂生成
```

改完重新构建：

```bash
~/.local/ollama/bin/ollama create fishbot-qwen:3b \
    -f ~/.local/ollama/modelfiles/fishbot-qwen.Modelfile
```

> `num_ctx 4096` 时模型 **100% 跑在 GPU**，每轮约 1.1 秒。
> 早期用 8192 时 25% 的层溢出到 CPU，且显存顶到上限，
> 最终导致整机死机（坑 0）。
>
> 4096 上下文配 19 个工具（prompt 约 1600 tokens）够用；
> 如果以后工具加多了导致 prompt 超过 3500 tokens，再考虑调大，
> 但要同时把 `num_ctx` 和显存余量一起算。

---

## 六、换 / 加模型

```bash
export HTTPS_PROXY=http://127.0.0.1:17891   # 拉模型需要代理
ollama pull qwen2.5:1.5b                    # 更小更快，但 23 工具可能选错
ollama pull qwen2.5:7b                      # 更准，但 4 GB 显存放不下
```

拉下来后改 `FISHBOT_LOCAL_MODEL` 环境变量即可，不用改代码：

```bash
FISHBOT_LOCAL_MODEL=qwen2.5:1.5b ./run_web_controller.sh
```

> 拉模型（`ollama pull`）走外网，需要代理；
> 推理走 `127.0.0.1`，**绝对不能**走代理。
> 脚本里的 `local_llm.sh` 已经处理好：启动服务时清掉代理变量，
> 并设置 `NO_PROXY=127.0.0.1,localhost`。

---

## 七、卸载

```bash
./scripts/local_llm.sh stop
rm -rf ~/.local/ollama
rm -f ~/.local/bin/ollama
# 代码侧：把 FISHBOT_LLM 设为 deepseek，或直接删掉 SYSTEM_PROMPT_COMPACT 分支
```
