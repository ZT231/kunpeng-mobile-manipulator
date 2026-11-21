# Core/tests —— 机械臂固件 host 单元测试（阶段 6）

> 补上「机械臂 `Core/` 无测试」这个缺口。底盘有 `car/tests/test_frame.c`，机械臂以前只有
> 「烧进板子拿串口敲」，现在能在 PC 上把刚改过的核心逻辑先跑一遍。

## 怎么跑（项目根 `RoboticArm_Kunpeng/` 下，Git Bash）

```sh
sh Core/tests/run_tests.sh
```

或手动两条：

```sh
gcc -std=c99 -Wall -Wextra -I Core/tests/stubs -I Core/Inc \
    Core/tests/test_arm_firmware.c -o Core/tests/build/test_arm_firmware.exe
./Core/tests/build/test_arm_firmware.exe
```

## 测了什么（对应阶段 1 / 阶段 2 的改动）

- **FAULT 状态机**：`ReportFault` 进 FAULT + 故障码锁存 + 冻结；冻结态拒运动命令；
  `STOP` 不覆盖 FAULT；`RESUME` 解不了 FAULT；`CLEARFAULT` 恢复并回 CMD。
- **错误码**：未知命令 → `ERR,2`；校验失败 → `ERR,1`；缺参数 → `ERR,3`；
  FAULT 态运动命令 → `ERR,5`；ESTOP 态 → `ERR,4`；STATE 末尾带 fault 字段；
  `$FAULT` / `$CLEARFAULT` 命令。

## README 三行法

- **① 用了什么**：`#include "arm_control.c" / "uart_protocol.c"` 把被测源码「展开」进测试
  文件，让 `static` 函数（`state_is_frozen` / `proto_dispatch` 等）对断言可见；
  `stubs/` 下两个假头文件顶掉 STM32 HAL 和 FreeRTOS，用「假 PCA9685 / 假示教 /
  假 ACS712 / 假串口发送」替换硬件。
- **② 为什么这么设计**：被测的是纯逻辑（状态机 + 协议），不该被真硬件绑定。把「不稳定的
  硬件依赖」换成「稳定可控的假实现」（测试替身），才能只测被测模块自己的判断，而且 1 秒跑完、
  不用接板子。
- **③ 不这么写会怎样**：机械臂固件就只能等烧进板子才能验证，改一次错误码要回教室烧一次；
  而且 CK 校验码必须用**独立脚本**算好再硬编码进测试，否则「实现和测试犯同一个错互相印证」。
  （本次写测试时就真踩了：手填 CK 全错，导致 9 项误报，改用 Python 独立算一遍才修对。）

## 红线

- `stubs/` 里的假头文件**只服务本测试，绝不进 Keil 工程**。
- `build/` 产物已在 `.gitignore` 排除，不提交。
