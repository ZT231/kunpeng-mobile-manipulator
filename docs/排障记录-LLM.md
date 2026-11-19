# 排障记录 · 阶段 2 LLM 意图解析层（本地自用，不进 Git）

> 记录区间：2026-11-08。格式固定：**现象 → 排查过程 → 根因 → 解决 → 一句话面试说辞**。
> 只记重要 / 关键 / 面试官关心的问题，鸡毛蒜皮不记。

## 第 1 条：`python -m unittest discover -s <目录>` 报 `ImportError: Start directory is not importable`

- **现象**：17:14 在 PowerShell 跑
  `python.exe -B -m unittest discover -s "D:\...\kunpeng\llm" -p "test_*.py"`，
  报 `ImportError: Start directory is not importable: 'D:\\...\\kunpeng\\llm'`，
  一条测试都没跑起来。当时容易误判成「刚写的代码有语法错 / 模块导入失败」。

- **排查过程**：
  1. `Glob kunpeng/llm/**` → `No files found`，确认**阶段 2 源码当时根本还没落盘**（按「点头才动手」约定，目录都不存在）；
  2. 17:18 落盘后先补 `__init__.py`，在目录内跑 `discover -s . -p "test_*.py" -v` → **26 项全绿**；
  3. 再用原样的绝对路径版 `discover -s "D:\...\kunpeng\llm"` 复跑 → exit code 0，报错不复现；
  4. 对照 CPython 3.11+ `unittest/loader.py`：`TestLoader.discover()` 要用 start_dir 推导模块名，要求该目录**可导入**；3.11 起不再支持 namespace package（无 `__init__.py` 的目录）作为发现起点。

- **根因**：**两个原因叠加**，都跟 `intent_parser.py` 的逻辑无关。
  ① 主因是**验证命令跑在代码落盘之前** —— 目录不存在，谈不上导入；
  ② 即便目录建好，Python 3.11+ 的 `unittest discover` 要求起始目录是 **regular package**（含 `__init__.py`），否则拒收。
  规范定义：PEP 420 引入 namespace package；CPython 3.11 的 unittest 移除了对它的 discover 支持（`TestLoader.discover` 里 `is_not_importable` 检查）。

- **解决**：
  1. **流程**：按「先点头 → 再落盘 → 最后给验证命令」执行；「待点头 / 尚未落盘」阶段不再把可整段复制的验证命令摆进代码块（已写进跨项目习惯）。
  2. **代码**：新增 `kunpeng/llm/__init__.py`（空文件 + docstring），让目录成为 regular package。
  3. **命令**：保留绝对路径 `-s` 也能过（包已可导入）；另给一条更稳的写法是先 `cd` 进目录跑 `discover -s .`。
  4. **实测**：`--self-test` → `SELF_TEST PASS`；`unittest discover` → `Ran 26 tests OK`；PowerShell 绝对路径版 → exit code 0。

- **一句话面试说辞**：「`unittest discover` 在 Python 3.11 之后要求起始目录是可导入包，我给模块补了 `__init__.py`；那次报错更主要的原因是验证命令跑在代码落盘之前，所以我把『先写后验』定成了流程。」

## 第 2 条：中文串口帧 XOR「按字节还是按字符」——跨语言对齐坑

- **现象**：16:35 做 ESP32 文本入口离线网关时，`--self-test` 一开始把演示用的空文本帧
  `$ASR,*3C\r\n` 判成 `CK_MISMATCH`（算出 `6C`、帧里写 `3C`），不是预期的 `EMPTY_TEXT`。
  更要紧的是：若在 Python 里按 `for ch in text: ck ^= ord(ch)` 算中文校验和，
  拿到的是 Unicode 码点（「抓」= 0x6293），远超 1 字节，与 C 固件对不上。

- **排查过程**：
  1. 回读 `Core/Src/uart_protocol.c`：L123 `ck ^= (uint8_t)payload[i]`（发包）、
     L523 `calc_ck ^= frame_buf[i]`（收包）都是对 **uint8_t 字节**做 XOR，L128-129 输出 2 位大写 HEX；
  2. 确认 Python 的 `str` 是 Unicode 码点序列，「抓」在 UTF-8 里是 3 字节 `E6 8A 93`；
  3. 用 S2 已实机的 `$PING*10` / `$OK,PONG*3E` 反查：CK("PING")=0x10、CK("OK,PONG")=0x3E，
     证明「逐字节 XOR」实现与固件一致，不是自说自话；
  4. 空文本帧那条其实是**演示用例手算 CK 写错**（`ASR,` 的 XOR 应为 6C），
     改用 `build_frame("")` 生成后，校验层才正常走到 `EMPTY_TEXT` 分支。

- **根因**：XOR 校验定义在**字节序列**上，而 Python `str` 是字符（码点）序列。
  跨语言对齐必须同时统一 ① 字符编码（UTF-8）② 字长（8 bit）。
  规范定义：纵向冗余校验（LRC）作用于 octet stream；`uint8_t` 逐字节累加后截断到 0~255，
  与 Unicode code point 不是同一数学对象。

- **解决**：
  1. `xor_checksum()` 里先 `payload.encode("utf-8")` 再逐字节 `ck ^= byte`，最后 `& 0xFF`；
  2. 把「逐字符算」与「逐字节算」两种结果都写进 `--self-test` / unittest 断言，
     用 `wrong_ck != manual` 把这条坑固化成回归用例；
  3. 用两条 S2 实机帧当锚点反查实现；
  4. 演示帧一律用 `build_frame()` 生成，不再手写 CK；
  5. 实测：`text_gateway.py --self-test` → `SELF_TEST PASS`；`unittest discover` → `Ran 44 tests OK`。

- **一句话面试说辞**：「串口帧的 XOR 是按字节算的，中文是 UTF-8 多字节；我在 Python 侧先 encode 再逐字节异或，并拿项目实机跑通的 `$PING*10`、`$OK,PONG*3E` 反查实现，避免上层语言按 Unicode 码点算导致将来一接板子就全部校验失败。」

## 本阶段诚实边界（写简历 / 面试别越线）

- ✅ 能写：「自然语言 → 结构化动作指令的**意图解析层**，mock 实现 + 26 项单元测试通过；契约定死、后端可替换」。
- ❌ 不能写：「接入了云端大模型」「自然语言控制机械臂已实现」「语音→动作闭环」「自动抓取」。
- 模块**只输出结构化指令、不下发**；`source` 恒 `"mock"`；`params` 恒空（不猜数值）；枚举里故意没有 `NAV`（车体 / 导航是阶段 6，命中即 `unknown` + warning）；`CloudBackend` 是占位、`parse()` 抛 `NotImplementedError`，`LLM_API_KEY` 只走环境变量、绝不写进代码。
- 当前真正能执行的是「人点画面 → `ArmIK` 逆解 → `$JOINT` 串口帧」这条**开环**链路；自动抓取、下降-闭合-抬起序列、视觉伺服闭环都还没做。
