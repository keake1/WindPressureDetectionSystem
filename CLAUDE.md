# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概况

风压检测系统：Modbus RTU 主-从架构嵌入式系统。两枚 STM32 控制器（Controlboard + Hostboard）通过 PowerBus 二总线互联，驱动传感器从机和迪文屏。

| 板卡 | MCU | 构建 | 编译器 |
|------|-----|------|--------|
| Hostboard | STM32F407VET6 (M4F, 168MHz) | CMake+Ninja | `/opt/arm-gnu-toolchain-15.2.rel1/bin/arm-none-eabi-gcc` |
| 热敏打印机 | USART2 (PA2/PA3, 9600) | — | 连接到 Hostboard |
| Controlboard | STM32F070xB (M0, 48MHz) | CMake+Ninja | 同上 |
| 风压传感器 | STC8H1K28 (51) | SDCC | `sdcc -mmcs51` |
| CO/余压/7合1 | STC8H 系列 (51) | Keil (Windows) | C51 |

## 快速命令

| 操作 | 命令（项目根目录） |
|------|-------------------|
| 编译 Controlboard | `cd Controlboard && cmake --preset Debug && cmake --build --preset Debug` |
| 编译+烧录 Controlboard | `cd Controlboard && cmake --build build/Debug --target flash` |
| 编译 Hostboard | `cd Hostboard && cmake --preset Debug && cmake --build --preset Debug` |
| 烧录 Hostboard | `openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program Hostboard/build/Debug/Hostboard.elf verify reset exit"` |
| 编译风压传感器 | `cd 风压传感器 && ./tools/build.sh` |
| 烧录风压传感器 | `cd 风压传感器 && ./tools/flash.sh` |
| 烧录 7合1传感器 | `cd 7合1传感器 && ./tools/flash.sh` |
| 串口 Modbus 监控 | `python3 scripts/serial_monitor.py /dev/ttyUSB0 --modbus-poll --slave 1` |
| 调试 Hostboard (GDB) | `arm-none-eabi-gdb Hostboard/build/Debug/Hostboard.elf -ex "target remote :3333" -ex "load"` |
| 调试 Controlboard (GDB) | `arm-none-eabi-gdb Controlboard/build/Debug/Controlboard.elf -ex "target remote :3333" -ex "load"` |

## 快速阅读路径

要理解系统，按此顺序读源码：

1. **`Hostboard/Core/Src/main.c`** — 任务创建（8 个 FreeRTOS 任务）
2. **`Hostboard/Core/Src/modbus_polling.c`** — 轮询策略（50ms/地址, 0-128，正常 4bits/在线穿插 67bits 交替）
3. **`Hostboard/Core/Src/hostboard_registers.c`** — 线圈存储布局、零地址/重复地址检测
4. **`Hostboard/Core/Src/modbus_master_tasks.c`** — 发送/接收任务 + 动态超时
5. **`Hostboard/Core/Src/dwin_tasks.c`** — 迪文屏通信任务 + 图标更新 + 报警监测
6. **`Hostboard/Core/Src/dwin.c`** — DWIN 帧构建 API、NorFlash、RTC
7. **Controlboard 侧对应文件结构相同**（`modbus_polling.c` / `modbus_tasks.c` / `modbus_registers.c` / `dwin.c`）

## 核心架构：双控制器板间通信

```
Hostboard                      PowerBus 二总线              Controlboard
USART1 (PA9/PA10, 9600 8N1)  ─────────────  UART2 (PA2/PA3)
    Modbus 主站 (FC 0x02/0x03)                  Modbus 从站 (DIP 地址)
```

- **Controlboard** 同时是 **Modbus 主站**（USART1 轮询 0-63 传感器）和 **Modbus 从站**（UART2 响应 Hostboard）
- **Hostboard** 每 50ms 顺序扫描地址 0-128（FC 0x02），每 8 次正常轮询穿插 1 次在线控制器读 67bits(63-129)
- **迪文屏**（Hostboard USART3, 115200）：DGUS II 协议，0x82 写/0x83 读，ISR 状态机组装帧
- **热敏打印机**（Hostboard USART2, 9600）：IT 中断发送 + TX 完成信号量，通过独立任务 `TaskPrinterTx` + 4 槽队列接收报警打印作业

## 线圈寄存器布局（Hostboard hostboard_registers.h）

每个 Controlboard 存储 17 字节 coil_data[17]（130 bits）：

| 字节 | 位 | 内容 |
|------|----|------|
| 0-7 | 0-62 | 传感器 1-63 在线状态 |
| 8-15 | 63-125 | 传感器 1-63 报警标志 |
| 16 位 0 | 126 | 零地址存在 |
| 16 位 1 | 127 | 地址重复 |
| 16 位 2 | 128 | 全局报警（任一传感器报警） |
| 16 位 3 | 129 | 烟雾报警器 |

常量宏：`COIL_OFFSET_ONLINE(0)` `COIL_OFFSET_ALARM(63)` `COIL_GLOBAL_ALARM(128)` `COIL_SMOKE_ALARM(129)` 等定义在 `hostboard_registers.h`。

## DWIN 变量区布局（Hostboard dwin.h）

| 地址 | 用途 | 大小 |
|------|------|------|
| 0x1000 | 备注变量区 | 128 槽 × 0x10 地址 |
| 0x1800-0x187F | 控制器状态图标 | 128 word（0=离线 1=正常 2=故障 3=报警） |
| 0x1881 | 系统状态图标 | 1 word（3=重复地址 2=零地址 1=有报警 0=正常） |
| 0x1882-0x1885 | 详情界面状态 | 4 word（窗状态/全局报警/零地址/重复地址） |
| 0x1890-0x18CE | 传感器图标 | 63 word |
| 0x1900-0x19EF | 传感器数据 | 63 × 16 word |
| 0x2000-0x202C | 报警记录区 | 12 条循环记录 |
| 0x3100 | 选中控制器地址 | 1 word（屏→Hostboard） |

## 已知问题与修复

已修复的 Bug 和架构决策记录在 memory 中（`/home/keake/.claude/projects/.../memory/`），新会话建议优先阅读：

| 问题 | memory 文件 | 涉及文件 |
|------|------------|---------|
| ORE+IDLE 中断风暴 | [[usart1-ore-interrupt-storm-fix.md]] | `stm32f0xx_it.c` USART1_IRQHandler |
| FE/NE/PE→TXE 中断风暴 | [[stm32f0-fe-ne-pe-interrupt-storm.md]] | 两侧所有 `xxx_it.c` UART ISR |
| CubeMX + FreeRTOS 中断冲突 | [[cubemx-freertos-systick-conflict.md]] | `stm32f0xx_it.c` `port.c` |
| 信号量分发与跨任务保护 | [[modbus-signal-architecture.md]] | 所有发送/接收任务 |

## .gitignore 说明

- `Controlboard/.gitignore` 排除了 `*.sh` 和 `*.json`（`flash.sh`、`CMakePresets.json` 等不入库）
- 根 `.gitignore` 排除了 `.claude/`、`*.code-workspace`

## 工作规范

用户自行控制 git add/commit/push。允许查看状态但不擅自提交。
