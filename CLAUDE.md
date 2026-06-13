# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概况

风压检测系统：Modbus RTU 主-从架构嵌入式系统。两枚 STM32 控制器（Controlboard + Hostboard）通过 PowerBus 二总线互联，驱动传感器从机和迪文屏。

| 板卡 | MCU | 构建 | 编译器 |
|------|-----|------|--------|
| Hostboard | STM32F407VET6 (M4F, 168MHz) | CMake+Ninja | `/opt/arm-gnu-toolchain-15.2.rel1/bin/arm-none-eabi-gcc` |
| Controlboard | STM32F070xB (M0, 48MHz) | CMake+Ninja | 同上 |
| CO/风压/余压传感器 | STC8H 系列 (51) | Keil (Windows) | C51 |
| 7合1传感器 | STC8H 系列 (51) | Keil (Windows) | C51 |
| 温湿度传感器 | STC8H1K28 (51) | SDCC | `sdcc -mmcs51` |
| CO2传感器 | STC8H1K28 (51) | Keil (Windows) | C51 |
| 热敏打印机 | — USART2 (PA2/PA3, 9600) | — | 连接到 Hostboard |

## 快速命令

| 操作 | 命令（项目根目录） |
|------|-------------------|
| 编译 Controlboard | `cd Controlboard && cmake --preset Debug && cmake --build --preset Debug` |
| 编译+烧录 Controlboard | `cd Controlboard && cmake --build build/Debug --target flash` |
| 编译 Hostboard | `cd Hostboard && cmake --preset Debug && cmake --build --preset Debug` |
| 烧录 Hostboard | `openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program Hostboard/build/Debug/Hostboard.elf verify reset exit"` |
| 编译风压传感器 | `cd 风压传感器 && ./tools/build.sh` |
| 烧录 7合1传感器 | `cd 7合1传感器 && ./tools/flash.sh` |
| 串口 Modbus 监控 | `python3 scripts/serial_monitor.py /dev/ttyUSB0 --modbus-poll --slave 1` |
| 调试 Hostboard (GDB) | `arm-none-eabi-gdb Hostboard/build/Debug/Hostboard.elf -ex "target remote :3333" -ex "load"` |
| 调试 Controlboard (GDB) | `arm-none-eabi-gdb Controlboard/build/Debug/Controlboard.elf -ex "target remote :3333" -ex "load"` |

CMake Presets：两个 STM32 工程共用相同 preset 结构——`default`（隐藏，指定 Ninja + 工具链文件）、`Debug`、`Release`。`cmake --preset Debug` 只配置一次，之后只需 `cmake --build build/Debug` 增量编译。编译后自动在 `build/Debug/` 下生成 `.elf`/`.hex`/`.bin` 三种格式。

## 快速阅读路径

要理解系统，按此顺序读源码：

1. **`Hostboard/Core/Src/main.c`** — 任务创建（8 个 FreeRTOS 任务）
2. **`Hostboard/Core/Src/modbus_polling.c`** — 轮询策略（50ms/地址, 0-128，正常 4bits/在线穿插 67bits 交替，选中控制器时插队详情采集）
3. **`Hostboard/Core/Src/hostboard_registers.c`** — 线圈存储布局、零地址/重复地址检测
4. **`Hostboard/Core/Src/modbus_master_tasks.c`** — 发送/接收任务 + 动态超时 + 详情数据推屏
5. **`Hostboard/Core/Src/dwin_tasks.c`** — 迪文屏通信任务 + 图标更新 + 报警监测 + 详情界面双模式
6. **`Hostboard/Core/Src/dwin.c`** — DWIN 帧构建 API、NorFlash、RTC
7. **`Hostboard/Core/Src/detail_view_data.c`** — 详情界面传感器型号/线圈就绪缓冲
8. **Controlboard 侧对应文件结构相同**（`modbus_polling.c` / `modbus_tasks.c` / `modbus_registers.c` / `dwin.c`）

## 核心架构：双控制器板间通信

```
Hostboard                      PowerBus 二总线              Controlboard
USART1 (PA9/PA10, 9600 8N1)  ─────────────  UART2 (PA2/PA3)
    Modbus 主站 (FC 0x02/0x03)                  Modbus 从站 (DIP 地址)
```

- **Controlboard** 同时是 **Modbus 主站**（USART1 轮询 0-63 传感器，50ms/地址）和 **Modbus 从站**（UART2 响应 Hostboard）
- **Hostboard** 每 50ms 顺序扫描地址 0-128（FC 0x02），每 8 次正常轮询穿插 1 次读 67bits(63-129)
- **迪文屏**（Hostboard USART3, 115200）：DGUS II 协议，0x82 写/0x83 读，ISR 状态机组装帧
- **热敏打印机**（Hostboard USART2, 9600）：IT 中断发送 + TX 完成信号量，通过独立任务 `TaskPrinterTx` + 4 槽队列接收报警打印作业

## 传感器型号与数据布局

| 型号字节 | 类型 | 数据寄存器 | 协议帧格式（FC 0x03 响应） |
|---------|------|-----------|--------------------------|
| 0x01 | CO | reg_data[0] = CO 浓度 (÷100) | `[addr][0x03][3][0x01][CO_H][CO_L][CRC]` |
| 0x02 | 风压 | reg_data[0] = 压力值(Pa) | 同上（3字节数据） |
| 0x03 | 余压 | reg_data[0] = 余压值(Pa) | 同上（3字节数据） |
| 0x04 | 7合1 | [0]=eCO₂ [1]=eCH₂O [2]=TVOC [3]=PM2.5 [4]=PM10 [5]=温度(÷10) [6]=湿度(÷10) | `[addr][0x03][15][0x04][14字节数据][CRC]` |
| 0x05 | 温湿度 | [0]=温度(×10,int16) [1]=湿度(×10) | `[addr][0x03][5][0x05][temp_H][temp_L][hum_H][hum_L][CRC]` |
| 0x06 | CO2 | [0]=CO₂浓度(ppm) | `[addr][0x03][3][0x06][CO2_H][CO2_L][CRC]` |

Controlboard 接收任务（`modbus_tasks.c`）统一解析：`frame[2]`=数据字节数（含型号），`frame[3]`=型号，`frame[4..]`=数据。

报警策略：

| 类型 | 报警阈值 | 恢复阈值 |
|------|---------|---------|
| CO (0x01) | > 2500 | < 2000 |
| 7合1 (0x04) | eCO₂>1000 / eCH₂O>80 / TVOC>600 / PM2.5>35 / PM10>50 | 全部低于恢复值 |
| 温湿度 (0x05) | 不参与 | — |
| CO2 (0x06) | > 1000 | < 800 |

报警状态变化时：Controlboard 发 FC 0x06 到从机 register 0x0004（0x0001=亮红灯，0x0000=灭）。

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

常量宏定义在 `hostboard_registers.h`：`COIL_OFFSET_ONLINE(0)` `COIL_OFFSET_ALARM(63)` `COIL_GLOBAL_ALARM(128)` `COIL_SMOKE_ALARM(129)` 等。

## DWIN 变量区布局

### Hostboard（dwin.h）

| 地址 | 用途 | 大小 |
|------|------|------|
| 0x1000 | 备注变量区 | 128 槽 × 0x10 地址 |
| 0x1800-0x187F | 控制器状态图标 | 128 word |
| 0x1881 | 系统状态图标 | 1 word |
| 0x1882-0x1885 | 详情界面状态 | 4 word |
| 0x1890-0x18CE | 传感器图标 | 63 word |
| 0x1900-0x19EF | 传感器数据（每传感器 16 word） | 63 × 16 word |
| 0x2000-0x202C | 报警记录区 | 12 条循环记录 |
| 0x3100 | 选中控制器地址（屏→Hostboard） | 1 word |

### Controlboard（dwin.h）

| 地址 | 用途 | 大小 |
|------|------|------|
| 0x1400-0x1444 | 状态帧：图标(63) + 开窗 + 全局报警 + 零地址 + 重复地址 + 烟雾 | 69 word |
| 0x1500 | 控板 DIP 地址 | 1 word |
| VAR_BASE | 传感器数据（每传感器 16 word，偏移定义见下方） | 63 × 16 word |

Controlboard 传感器数据偏移常量（`dwin.h`）：

| 偏移 | 字段 | 类型 |
|------|------|------|
| 0-1 | CO 浓度 | float |
| 2-3 | 风压 | float |
| 4-5 | 余压 | uint16 |
| 6-7 | (预留) | — |
| 8-9 | (预留) | — |
| 10-11 | eCO₂ / CO₂ 浓度 | uint16 |
| 12-13 | 温度 | float (÷10) |
| 14-15 | 湿度 | float (÷10) |

## DWIN 初始化屏障

开机后 `TaskDwinRx` 需逐个从 NorFlash 恢复 128 条备注数据（每槽 100ms，约 12.8 秒）。在此期间 `TaskDwinIcons` 和 `TaskAlarmMonitor` 不应对屏发送数据。

机制：Counting Semaphore（max=2, init=0），一次性任务 `TaskDwinInit` 遍历完 128 槽后 Give 两次，两个等待任务（`TaskDwinIcons`、`TaskAlarmMonitor`）的 `for(;;)` 前调用 `xSemaphoreTake(xDwinInitDoneSem, portMAX_DELAY)` 阻塞等待。

## 已知问题与修复

已修复的 Bug 和架构决策记录在 memory 中，新会话建议优先阅读：

- `memory/usart1-ore-interrupt-storm-fix.md` — RS485 噪声致 ORE → HAL 关闭 RXNEIE → Modbus 接收静默失效
- `memory/stm32f0-fe-ne-pe-interrupt-storm.md` — 中断发送时 HAL 错误分支提前 return 导致 TXEIE 永不关闭
- `memory/cubemx-freertos-systick-conflict.md` — CubeMX 重新生成后 Weak Handler + SysTick 与 HAL 时基冲突
- `memory/modbus-signal-architecture.md` — 信号量分发策略、中断发送架构、64 位寄存器/环形缓冲区跨任务保护
- `memory/hostboard-naked-svc-pendsv-fix.md` — CubeMX 生成 C 包装调用破坏 EXC_RETURN 导致 HardFault

## 温度负值处理

AHT30 等传感器温度可到 -40℃。Modbus 传输为 uint16 大端，Controlboard 以 `uint16_t` 存储。转换为 float 前必须 cast 为 `int16_t`：

```c
int16_t s_temp = (int16_t)ModbusReg_GetData(slave, 5);  // 强制有符号
fval = (float)s_temp / 10.0f;
```

否则负值（如 -100 = 0xFF9C）被当作 6543.6℃。影响 7 合 1（0x04）和温湿度（0x05）的温度字段，湿度始终为正无需处理。

## .gitignore 说明

- `Controlboard/.gitignore` 排除了 `*.sh` 和 `*.json`（`flash.sh`、`CMakePresets.json` 等不入库）
- 根 `.gitignore` 排除了 `.claude/`、`*.code-workspace`

## 工作规范

用户自行控制 git add/commit/push。允许查看状态但不擅自提交。
