# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

风压检测系统：一个 Modbus RTU 主-从架构的嵌入式系统。包含两个 STM32 控制器（Controlboard + Hostboard）通过 PowerBus 二总线互联，以及多个传感器从机和一块迪文屏用于本地显示。

### Repository Structure

```
WindPressureDetectionSystem/
├── Controlboard/       # 下位机控制器 (STM32F070xB + FreeRTOS) — 连接传感器总线 + 迪文屏
│   ├── CMakeLists.txt  # CMake + Ninja 构建（含 flash 目标）
│   ├── CMakePresets.json
│   ├── flash.cfg       # OpenOCD 烧录配置
│   ├── flash.sh        # 一键烧录脚本
│   ├── .clangd         # clangd 指向 ARM GCC
│   ├── .vscode/        # Cortex-Debug 调试配置
│   ├── Core/
│   │   ├── Src/        # 应用代码
│   │   └── Inc/        # 头文件
│   ├── Drivers/
│   │   ├── STM32F0xx_HAL_Driver/
│   │   ├── CMSIS/
│   │   └── Middlewares/FreeRTOS/  # FreeRTOS v11.1.0
│   ├── cmake/
│   │   ├── gcc-arm-none-eabi.cmake
│   │   └── stm32cubemx/CMakeLists.txt
│   ├── startup_stm32f070xb.s
│   └── STM32F070XX_FLASH.ld
├── Hostboard/           # 上位机/中继板 (STM32F407VET6 + FreeRTOS) — 通过 PowerBus 二总线与 Controlboard 通信 + USART3 驱动迪文屏
│   ├── CMakeLists.txt   # CMake + Ninja 构建
│   ├── CMakePresets.json
│   ├── Core/
│   │   ├── Src/         # 应用代码（含 Modbus 主站驱动、DWIN 屏驱动）
│   │   └── Inc/         # 头文件
│   ├── Drivers/
│   │   ├── STM32F4xx_HAL_Driver/
│   │   ├── CMSIS/
│   │   └── Middlewares/FreeRTOS/  # FreeRTOS v11.1.0
│   ├── cmake/
│   │   ├── gcc-arm-none-eabi.cmake
│   │   └── stm32cubemx/CMakeLists.txt
│   ├── startup_stm32f407xx.s
│   └── STM32F407VETx_FLASH.ld
├── 风压传感器/          # 风压传感器固件 (STC8H1K28, SDCC)
│   ├── src/             # 源文件 (main, board, uart, crc16, pressure, display, sensor_modbus)
│   ├── include/         # 头文件 (含 stc8h.h SFR 定义)
│   ├── tools/           # 构建脚本 (build.sh + build.ps1 / flash.sh + flash.ps1)
│   └── build/           # 编译输出 (.ihx, .hex, .rel)
├── co传感器/            # CO 传感器固件 (STC8H MCU), Keil μVision (Windows only)
├── 余压传感器07/        # 余压传感器固件 (STC8H MCU), Keil μVision (Windows only)
├── 7合1传感器/          # 7合1空气质量传感器固件 (STC8H MCU), Keil μVision (Windows only)
│   ├── hex文件/          # Keil 编译输出的 .hex 文件
│   └── tools/            # Linux 烧录脚本 flash.sh（需 stcgal）
└── 屏幕/                # 迪文屏 DGUS 工程 (DGUS 工具设计，仅 Windows)
    ├── 风压监测控制器屏幕/
    └── 风压监测屏幕/
```

### MCU 与工具链

| 项目 | MCU | 构建 | 编译器 |
|------|-----|------|--------|
| Hostboard | STM32F407VET6 (Cortex-M4F, 168MHz) | CMake + Ninja | `arm-none-eabi-gcc` (/opt/arm-gnu-toolchain-15.2.rel1) |
| Controlboard | STM32F070xB (Cortex-M0, 48MHz) | CMake + Ninja | `arm-none-eabi-gcc` (/opt/arm-gnu-toolchain-15.2.rel1) |
| 风压传感器 | STC8H1K28-36I-LQFP32 (51 内核) | SDCC (bash/PowerShell) | `sdcc -mmcs51` |
| CO/余压/7合1传感器 | STC8H 系列 (51 内核) | Keil μVision (Windows) | C51 |

## 快速命令参考

| 操作 | 命令（从项目根目录） |
|------|----------------------|
| 编译 Controlboard | `cd Controlboard && cmake --preset Debug && cmake --build --preset Debug` |
| 编译 + 烧录 Controlboard | `cd Controlboard && cmake --build build/Debug --target flash` |
| 仅烧录 Controlboard | `cd Controlboard && ./flash.sh` |
| 编译 Hostboard | `cd Hostboard && cmake --preset Debug && cmake --build --preset Debug` |
| 烧录 Hostboard | `openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program Hostboard/build/Debug/Hostboard.elf verify reset exit"` |
| 编译风压传感器 | `cd 风压传感器 && ./tools/build.sh` |
| 烧录风压传感器 | `cd 风压传感器 && ./tools/flash.sh` |
| 烧录 7合1传感器 | `cd 7合1传感器 && ./tools/flash.sh` |
| 调试 Controlboard (GDB) | `cd Controlboard && arm-none-eabi-gdb build/Debug/Controlboard.elf -ex "target remote :3333" -ex "load"` |
| 调试 Hostboard (GDB) | `cd Hostboard && arm-none-eabi-gdb build/Debug/Hostboard.elf -ex "target remote :3333" -ex "load"` |

## Controlboard 外设

| 外设 | 用途 | 参数 |
|------|------|------|
| USART1 | Modbus RTU 主站 — 传感器总线 (PowerBus 二总线) | 9600 8N1, TX=PA9, RX=PA10 |
| USART2 | Modbus RTU 从站 — Hostboard 互联 (PowerBus 二总线) | 9600 8N1, TX=PA2, RX=PA3 |
| UART4 | 迪文屏 (DWIN) 串口 | 115200 8N1, TX=PC10, RX=PC11 |
| I2C1 | 烟雾报警器 IO 扩展 (PCF8574) | SCL=PB6, SDA=PB7 |
| GPIO — LED | GREEN_LED=PB13, ORANGE_LED=PB8, RED_LED=PB9 | 低电平点亮 |
| GPIO — 继电器 | WS(PB0) — 全局报警 OR 烟雾报警时吸合 | 高电平有效 |
| GPIO — Isolator | PA0 读取烟雾报警器隔离输入 | 低电平=报警 |
| GPIO — DIP | PE0-PE5 读取 6 位 DIP 地址开关 | 阻值分压网络 |

## Controlboard 架构

Controlboard 是下位机控制器（STM32F070xB, 48MHz），同时承担两个 Modbus 角色：
- **主站**（USART1）：轮询传感器总线上的 0-63 地址，采集传感器数据，进行报警阈值判定，驱动迪文屏显示
- **从站**（UART2）：响应 Hostboard 的 FC 0x02/0x03 请求，返回传感器寄存器数据

系统共 7 个 FreeRTOS 任务，分为传感器轮询链路和 Hostboard 从站链路两条独立的数据流。

### 模块文件

| 文件 | 职责 |
|------|------|
| `uart1_modbus.c/h` | Modbus 串口驱动层：发送队列、ISR 逐字节接收、IDLE 帧检测、CRC 计算 |
| `modbus_tasks.c/h` | FreeRTOS 任务层：发送任务（帧发送 + 3.5字符间隔）、接收任务（CRC校验 + 解析入库 + 报警判定 + DWIN 更新） |
| `modbus_registers.c/h` | 数据存储层：线圈（在线/报警/系统位）和保持寄存器的读写接口；离线检测；地址重复检测；控板 DIP 地址 |
| `modbus_polling.c/h` | FreeRTOS 轮询任务：周期扫描地址 0-63，穿插在线传感器轮询；每轮回绕时读取 DIP 开关 |
| `dwin.c/h` | 迪文屏驱动 + 系统状态管理：UART3 统一队列发送；LED/WS 控制；烟雾报警器读取 |

### 任务架构（共 7 个 FreeRTOS 任务）

| 任务 | 函数 | 优先级 | 栈(words) | 职责 |
|------|------|--------|-----------|------|
| ModbusSend | `TaskModbusSend` | 2 | 128 | 从队列取请求，构造 Modbus 帧，中断发送 + 等 TX 完成信号量，读指令等响应信号量，3.5 字符间隔 |
| ModbusRecv | `TaskModbusReceive` | 2 | 128 | 等待 IDLE 中断填的原始帧队列，CRC 校验，解析数据存入寄存器，报警判定，DWIN 更新，释放信号量 |
| ModbusPoll | `TaskModbusPoll` | 2 | 128 | 每 50ms 轮询一个地址（0-63 顺序 + 每 8 次插 1 次在线），每轮回绕读 DIP+StepCycle，翻转 GREEN_LED |
| SlaveRecv | `TaskSlaveRecv` | 2 | 128 | UART2 从站接收：等待原始帧，CRC 校验，解析 FC 0x02/0x03，读寄存器构造响应，入队发送队列 |
| SlaveSend | `TaskSlaveSend` | 2 | 128 | UART2 从站发送：从发送队列取响应帧，中断发送 + 等 TX 完成信号量，3.5 字符间隔 |
| DwinIcons | `TaskDwinIcons` | 1 | 128 | **每 500ms**：读取 Isolator 引脚→更新烟雾报警线圈位；入队状态帧+地址帧；控制 WS/ORANGE_LED/RED_LED |
| IDLE | `Idle` | 0 | - | FreeRTOS 空闲任务 |

### 数据流架构

```
轮询任务 (TaskModbusPoll)
  │  每 50ms: EnqueueRequest(addr=n)
  │  地址回绕时: 读 DIP → ModbusReg_SetBoardAddr() → ModbusReg_StepCycle()
  │  每 10 次: Toggle GREEN_LED
  │  每轮: DWIN_FlushQueue() 排空迪文屏更新队列

  [传感器总线 (PowerBus 二总线) — Modbus 主站]
  │
  ├──→ [xModbusSendQueue(8)] ──→ 发送任务(TaskModbusSend) ──UART1 TX──→ PowerBus 总线
  │                                    │
  │                              xSemaphoreTake(30ms 超时)
  │                                    │
  │                              IDLE中断 ──→ [xModbusRawRxQueue(4)] ──→ 接收任务(TaskModbusReceive)
  │                                                                           │
  │                               CRC校验 → 解析 → SetType/SetData           │
  │                               → 报警判定 → DWIN_UpdateSensor()           │
  │                               → xSemaphoreGive()                         │
  │                                                                           │
  ├── 线圈寄存器 ←── 在线/报警位 ─────────────────────────────────────────────┤
  ├── 保持寄存器 ←── 类型+数据 ──────────────────────────────────────────────┤

  [Hostboard 互联 (PowerBus 二总线) — Modbus 从站]
  │
  ├── UART2 ISR (RXNE+IDLE+ORE) → [xSlaveRawRxQueue(4)]
  │                                            │
  │                              从站接收任务(TaskSlaveRecv)
  │                                ├─ CRC 校验
  │                                ├─ 解析 FC 0x02/0x03
  │                                ├─ ModbusReg_ReadCoil / ReadHolding
  │                                └─ xQueueSend(xSlaveTxQueue)
  │                                            │
  │                              从站发送任务(TaskSlaveSend)
  │                                ├─ HAL_UART_Transmit
  │                                └─ vTaskDelay(4ms) → UART2 TX → PowerBus 总线

  [迪文屏状态任务]
  │
  └── 迪文屏状态任务 (TaskDwinIcons, 每500ms)
        ├── 读 Isolator PA0 → 烟雾报警 → ModbusReg_SetSmokeAlarm()
        ├── 入队 STATUS（图标+开窗+全局报警+零地址+重复地址）
        ├── 入队 BOARD_ADDR（控板 DIP 地址）
        ├── WS(PB0) = 全局报警 OR 烟雾报警 → 继电器
        ├── RED_LED = 全局报警 → 点亮
        └── ORANGE_LED = 零地址存在 OR 地址重复 → 点亮
```

### 接收任务数据处理

接收到传感器响应后，接收任务依次完成：
1. CRC 校验（失败则 `ModbusReg_RecordCrcError()` 统计错误 → 用于地址重复检测）
2. `ModbusReg_RecordResponse(slave)` — 更新周期追踪 + 标记在线
3. 提取 `[从机地址] [型号] [数据...]`
4. 存型号字节到 `reg_type[slave]`
5. 存数据字节到 `reg_data[slave][0..6]`（7合1 用满7个，简单传感器只用第0个）
6. **报警阈值判定** — 根据型号和阈值检查是否触发/解除报警，状态变化时：
   - 更新线圈位 `ModbusReg_SetAlarm()`
   - 入队 0x06 写寄存器指令通知对应从机（寄存器 0x0004，写 1 开/0 关）
7. **DWIN 显示更新** — `DWIN_UpdateSensor(slave, model)` 入队传感器数据
8. `xSemaphoreGive()` 通知发送任务可以发下一帧

### 离线检测机制

- **`ModbusReg_RecordResponse(slave)`** — 收到 CRC 正确的帧时调用，更新 `last_seen_cycle[slave] = current_cycle`
- **`ModbusReg_StepCycle()`** — 每轮完整扫描（地址 63→0 回绕）调用，推进 `current_cycle`，检查所有地址：
  - `last_seen_cycle[i] + 3 <= current_cycle` → 连续三轮无响应 → `SetOnline(i, 0)`
  - 对地址 0 同样处理 → `zero_addr_present = 0`

### 地址重复检测机制

- **检测原理**：总线地址重复时，多个从机会同时响应同一请求，导致帧碰撞、CRC 错误激增
- **`ModbusReg_RecordCrcError()`** — 接收任务 CRC 校验失败时调用，累加当前周期错误计数
- **`ModbusReg_StepCycle()`** — 每轮扫描结束时：
  1. 将当前周期 CRC 错误数存入 `crc_error_history[3]` 环形缓冲
  2. 重置计数器
  3. 检查历史：连续三轮每轮错误数 > 2 → `addr_conflict_flag = 1`，否则 = 0
- **查询**: `ModbusReg_GetAddrConflict()` / 线圈位 127

## Hostboard 架构

Hostboard（STM32F407VET6, 168MHz）是上位机/中继板，通过 PowerBus 二总线与 Controlboard 通信，读取 Controlboard 的传感器数据，同时通过 USART3 驱动第二块迪文屏实现人机交互。

### 模块文件

| 文件 | 职责 |
|------|------|
| `uart1_modbus_master.c/h` | Modbus RTU 主站驱动：ISR 接收（RXNE+IDLE+ORE）、发送队列、TX 完成信号量、CRC 计算 |
| `modbus_master_tasks.c/h` | FreeRTOS 任务层：发送任务（中断发送 + 等 TX 完成信号量 + 开/关接收窗口）、接收任务（CRC 校验 + 存储数据 + 释放信号量） |
| `modbus_polling.c/h` | FreeRTOS 轮询任务：每 50ms 顺序扫描地址 0-128，FC 0x02 读 130 bits 离散输入 |
| `hostboard_registers.c/h` | 寄存器模块：128 路 Controlboard 线圈数据存储 + 零地址/重复地址检测 |
| `dwin.c/h` | 迪文屏驱动：帧构建（0x82/0x83）、NorFlash 读写、RTC 读取、报警记录写入、CRC16 计算；队列初始化；ISR 驱动（状态机帧检测） |
| `dwin_tasks.c/h` | USART3 迪文屏 FreeRTOS 收发任务：TaskDwinTx（队列→中断发送）、TaskDwinRx（0x83 应答解析：备注/地址/RTC） |
| `modbus_callbacks.c` | HAL UART TX 完成回调（释放 `xMasterTxCompleteSem` / `xDwinTxCompleteSem`） |

### 任务架构（共 7 个 FreeRTOS 任务）

| 任务 | 函数 | 优先级 | 栈(words) | 职责 |
|------|------|--------|-----------|------|
| MstSend | `TaskModbusSend` | 2 | 128 | 从队列取请求，构造 Modbus 帧，中断发送 + 等 TX 完成信号量，开/关接收窗口，等响应信号量，3.5 字符间隔 |
| MstRecv | `TaskModbusRecv` | 2 | 128 | 等待 IDLE 中断填的原始帧队列，CRC 校验，解析后存入寄存器模块（`HostReg_StoreCoilData`/`RecordZeroAddrResponse`/`RecordError`），释放信号量 |
| MstPoll | `TaskModbusPoll` | 1 | 128 | 每 50ms 顺序扫描一个地址 (0-128)，FC 0x02 读 130 bits；回绕时调用 `HostReg_StepCycle()` 更新零地址和重复地址标志 |
| DwinTx | `TaskDwinTx` | 1 | 128 | 从 xDwinTxQueue 取帧，HAL_UART_Transmit_IT 中断发送，等 xDwinTxCompleteSem（50ms 超时） |
| DwinRx | `TaskDwinRx` | 1 | 128 | 从 xDwinRxQueue 取 ISR 状态机组装的完整帧，解析 0x83 应答（备注写入 NorFlash / 开机恢复 / 选中地址 / RTC 时间） |
| DwinIcons | `TaskDwinIcons` | 1 | 128 | 每 500ms 更新迪文屏图标：4 帧控制器图标（1-128 地址）+ 1 帧系统状态（重复地址/零地址/报警/正常） |
| IDLE | `Idle` | 0 | - | FreeRTOS 空闲任务 |

### USART1 外设

| 引脚 | 方向 | 用途 |
|------|------|------|
| PA9 (USART1_TX) | 输出 | PowerBus 二总线发送 → Controlboard UART2 RX |
| PA10 (USART1_RX) | 输入 | PowerBus 二总线接收 ← Controlboard UART2 TX |

ISR 处理与 Controlboard UART1 架构相同：RXNE 逐字节接收 → IDLE 入原始帧队列 → ORE 写 ICR 清除。

> **注意**：PowerBus 二总线收发器内置方向控制，无需软件切换 DE/RE 引脚。

## 板间通信架构（Hostboard ↔ Controlboard）

### 物理层

```
Hostboard                    PowerBus 二总线            Controlboard
USART1 (PA9/PA10)  ─── 9600 8N1 ───  UART2 (PA2/PA3)
    主站 (Master)                              从站 (Slave)
```

- **从站地址**：Controlboard DIP 开关读取的地址（`ModbusReg_GetBoardAddr()`）
- **主站轮询地址范围**：0-128（共 129 个地址），由 `modbus_polling.c` 动态循环扫描

### 数据流

```
  Hostboard                                    Controlboard
  ┌─────────────────────┐                     ┌──────────────────────────┐
  │ TaskModbusPoll      │                     │ USART2 ISR               │
  │  └─ EnqueueRequest() │                     │  ├─ RXNE → 逐字节存      │
  │    [xMasterSendQueue]│                     │  ├─ IDLE → 入原始队列     │
  │      ↓               │    PowerBus 总线帧   │  └─ ORE  → 写 ICR 清除   │
  │ TaskModbusSend      │ ─────────────────→ │    [xSlaveRawRxQueue]    │
  │  ├─ StartRx() 开窗   │                     │        ↓                 │
  │  ├─ Transmit_IT      │                     │  TaskSlaveRecv           │
  │  ├─ TX 完成信号量     │                     │   ├─ CRC 校验            │
  │  ├─ 等响应信号量      │                     │   ├─ 解析 FC 0x02/0x03   │
  │  └─ DisableRx() 关窗 │                     │   ├─ ModbusReg_Read*()   │
  │                      │                     │   └─ xQueueSend(TxQueue)  │
  │ TaskModbusRecv      │ ←────────────────── │        ↓                 │
  │  ├─ CRC 校验         │    PowerBus 总线响应 │  TaskSlaveSend           │
  │  └─ xSemaphoreGive() │                     │   ├─ Transmit_IT         │
  └─────────────────────┘                     │   ├─ TX 完成信号量        │
                                              │   └─ vTaskDelay(4ms)      │
                                              └──────────────────────────┘
```

### Modbus 协议

| 功能码 | 名称 | Controlboard 内部调用 |
|--------|------|----------------------|
| `0x02` | 读离散输入 (Read Discrete Inputs) | `ModbusReg_ReadCoil(addr)` — 读取线圈位（在线/报警/系统位） |
| `0x03` | 读保持寄存器 (Read Holding Registers) | `ModbusReg_ReadHolding(addr)` — 读取传感器类型和数据 |

### 当前轮询策略

Hostboard `TaskModbusPoll` 每 50ms 顺序扫描一个地址：
- **地址范围**：0-128（129 个地址），地址 0 用于检测误设为零地址的 Controlboard
- **功能码**：FC 0x02，读取 130 bits（0-129）离散输入寄存器
- **完整一轮**：129 × 50ms = **~6.45s**
- **回绕处理**：地址超过 128 时，调用 `HostReg_StepCycle()` 更新检测标志

接收任务 `TaskModbusRecv` 对响应进行三路分发：
- **CRC 通过 + 地址 0 + FC 0x02** → `HostReg_RecordZeroAddrResponse()`（零地址检测计数）
- **CRC 通过 + 地址 1-128 + FC 0x02** → `HostReg_StoreCoilData(addr, data, len)`（存入寄存器）
- **CRC 失败 + 收到数据** → `HostReg_RecordError()`（计入重复地址检测）

### Hostboard 寄存器模块 (hostboard_registers.c/h)

独立模块，管理与 Controlboard 通信相关的数据存储和状态检测，类似 Controlboard 的 `modbus_registers.c`。

**存储结构**：
```c
#define MAX_CTRLBD_ADDR     128     /* 最大 Controlboard 地址 */
#define COIL_BYTE_COUNT     17      /* 130 bits = 16.25 → 17 字节 */

static uint8_t board_coil_data[MAX_CTRLBD_ADDR + 1][COIL_BYTE_COUNT];
/* board_coil_data[0] 预留；board_coil_data[1..128] 对应地址 1-128 */
```

**线圈布局（每个 Controlboard 的 17 字节）**：

| 字节 | bit 范围 | 内容 |
|-----|---------|------|
| 0-7 | 0-63 | 传感器 #1-63 在线状态（位 0-62）+ 位 63 为传感器 #1 报警 |
| 8-15 | 64-127 | 传感器 #2-63 报警标志（位 64-125）+ 零地址存在(126) + 地址重复(127) |
| 16 | 128-129 | 全局报警(128) + 烟雾报警器状态(129) |

**零地址检测**：
- `HostReg_RecordZeroAddrResponse()` — 收到地址 0 的 CRC 有效 FC 0x02 响应时调用
- 3 轮历史 OR：任意一轮有响应 → `zero_addr_present = 1`

**重复地址检测（与 Controlboard 相同机制）**：
- `HostReg_RecordError()` — CRC 失败且收到数据时调用
- 3 轮历史累计 > 2 → `addr_conflict_flag = 1`

**API**：
```c
void    HostReg_StoreCoilData(uint8_t addr, const uint8_t *data, uint8_t len);
uint8_t HostReg_GetCoilByte(uint8_t addr, uint8_t byte_idx);
uint8_t HostReg_GetCoilBit(uint8_t addr, uint16_t bit_idx);
void    HostReg_RecordZeroAddrResponse(void);
uint8_t HostReg_GetZeroAddrPresent(void);
void    HostReg_RecordError(void);
void    HostReg_StepCycle(void);
uint8_t HostReg_GetAddrConflict(void);
```

## Hostboard USART3 迪文屏架构

Hostboard 通过 USART3 连接第二块迪文屏（DGUS II），实现双向通信：

| 外设 | 用途 | 参数 |
|------|------|------|
| USART3 | 迪文屏 (DWIN) 双向通信 | 115200 8N1, TX=PB10, RX=PB11 |

### 协议

DGUS II 变长帧协议，所有帧以 `0x5A 0xA5` 开头：

```
5A A5 [LEN] [CMD] [DATA...]
LEN = 数据域字节数（不含 0x5A 0xA5 LEN 三字节）
```

| 指令 | 方向 | 用途 |
|------|------|------|
| `0x82` | Hostboard → 屏 | 写变量（传感器数据、图标、RTC、NorFlash 操作等） |
| `0x83` | 屏 → Hostboard | 读变量应答（备注内容、选中地址、RTC 时间等） |

### ISR 状态机（字节级帧检测）

USART3_IRQHandler 实现 4 状态帧组装状态机：

```
WAIT_5A  →  收到 0x5A  →  WAIT_A5
WAIT_A5  →  收到 0xA5  →  WAIT_LEN （否则回 WAIT_5A）
WAIT_LEN →  收到 LEN 字节 → DATA（记录期望总长度 total = 3 + LEN）
DATA     →  累计到 total 字节 → 完整帧入队 xDwinRxQueue → WAIT_5A
```

- **RXNE**：调用 `DwinRxByteHandler(data)` 驱动状态机
- **ORE**：调用 `DwinRxReset()` 清状态机回 WAIT_5A
- 帧完成后通过 `xQueueSendFromISR(xDwinRxQueue)` 通知接收任务
- 若 `portYIELD_FROM_ISR` 返回 pdTRUE，立即上下文切换

### 队列与任务架构

```
DWIN_WriteVar() / DWIN_ReadVar() / DWIN_NorFlashWrite() / DWIN_ReadRTC()
  │  构建 0x82 或 0x83 帧 → UART3_Send()
  │    UART3_Send() → xQueueSend(xDwinTxQueue)
  │                          │
  │                    [xDwinTxQueue(8)]
  │                          │
  │                    TaskDwinTx（优先级 1）
  │                      ├─ xQueueReceive(xDwinTxQueue)
  │                      ├─ HAL_UART_Transmit_IT(&huart3)
  │                      └─ xSemaphoreTake(xDwinTxCompleteSem, 50ms)
  │
  │                    USART3 ISR（状态机）
  │                      ├─ RXNE → DwinRxByteHandler() → 状态机
  │                      └─ 完整帧 → xQueueSendFromISR(xDwinRxQueue)
  │                          │
  │                    [xDwinRxQueue(4)]
  │                          │
  │                    TaskDwinRx（优先级 1）
  │                      ├─ xQueueReceive(xDwinRxQueue)
  │                      └─ 解析 0x83 应答
```

### 模块文件

| 文件 | 职责 |
|------|------|
| `dwin.c/h` | 帧构建 API：`DWIN_WriteVar`（0x82 写变量）、`DWIN_ReadVar`（0x83 读请求）、`DWIN_NorFlashWrite/Read`、`DWIN_ReadRTC`、`DWIN_WriteAlarmRecord`、`DWIN_CalcCRC16`；队列初始化 `Dwin_InitQueues`；ISR 驱动 `DwinRxByteHandler`/`DwinRxReset`；发送入口 `UART3_Send` |
| `dwin_tasks.c/h` | TaskDwinTx（队列取帧→中断发送）、TaskDwinRx（0x83 应答解析）、TaskDwinIcons（每 500ms 更新图标） |

### 图标更新任务 (TaskDwinIcons)

每 500ms 组装 5 帧图标数据，通过 DWIN_WriteVar 批量发送：

| 帧 | 地址 | 内容 | data_len |
|----|------|------|---------|
| 1 | `0x1800` | 控制器 1-32 图标 | 64 字节 |
| 2 | `0x1820` | 控制器 33-64 图标 | 64 字节 |
| 3 | `0x1840` | 控制器 65-96 图标 | 64 字节 |
| 4 | `0x1860` | 控制器 97-128 图标 | 64 字节 |
| 5 | `0x1881` | 系统状态图标 | 2 字节 |

**控制器图标判定**（每地址独立）：
```
离线 → DWIN_ICON_OFFLINE (0x0000)
在线且零地址/重复地址 → DWIN_ICON_TROUBLE (0x0002)
在线且全局报警 → DWIN_ICON_ALARM (0x0003)
在线且正常 → DWIN_ICON_NORMAL (0x0001)
```

**系统状态图标判定**（全局）：
```
有重复地址 → 3
有零地址 → 2
任一控制器报警 → 1
正常 → 0
```

### 接收任务 (TaskDwinRx) 解析分支

收到 `0x5A 0xA5 [LEN] 0x83 [addrH] [addrL] [data...]` 后，按变量地址分发：

| 地址范围 | 用途 | 处理 |
|---------|------|------|
| `0x1000~0x17FF` | 备注写入（屏幕下发备注） | 校验 → 计算 CRC → 通过 DWIN_WriteVar 写入暂存区 → vTaskDelay(20ms) → DWIN_NorFlashWrite 写入 NorFlash → vTaskDelay(30ms) |
| `0x3220` | 开机恢复（NorFlash → 变量） | CRC 校验 → DWIN_WriteVar 写入屏变量区 |
| `0x3100` | 屏幕选中控制器地址 | 存入 `g_hostDwinStatus.selectAddr` |
| `0x0010` | RTC 时间 | 填充 `g_hostDwinStatus.rtcYear/Month/Day/Hour/Minute/Second`，置 `rtcReady=1` |

### 变量区布局

| 地址 | 用途 | 说明 |
|------|------|------|
| `0x1000` | 备注变量区起始 | 128 个槽位，每槽 0x10 地址，备注内容最长 30 字节 + CRC16 |
| `0x1800~0x187F` | 控制器状态图标 | 128 个 word，下标=地址-1，值：0=离线 1=正常 2=故障 3=报警 |
| `0x1881` | 系统状态图标 | 1 个 word，3=重复地址 2=零地址 1=有报警 0=正常 |
| `0x1890` | 传感器状态图标 | 对应地址 129~192 的传感器图标 |
| `0x1900` | 传感器数据区 | 屏幕读回后的传感器数据存放处 |
| `0x2000~0x202C` | 报警记录区 | 12 条循环记录，每条 8 字节（年月日时分+地址+槽位） |
| `0x3100` | 选中控制器地址 | 用户在屏幕上点选后回传 |
| `0x3200` | 备注写入暂存区 | 写入 NorFlash 前的中转缓冲区 |
| `0x3220` | 备注读回暂存区 | 从 NorFlash 读回后的中转缓冲区 |

### API 速查

```c
/* 核心发送 API（自动入队发送队列，不阻塞） */
void DWIN_WriteVar(uint16_t addr, const uint8_t *pData, uint8_t data_len);
void DWIN_ReadVar(uint16_t addr, uint8_t word_cnt);
void DWIN_NorFlashWrite(uint32_t flash_addr, uint16_t var_addr, uint16_t word_len);
void DWIN_NorFlashRead(uint32_t flash_addr, uint16_t var_addr, uint16_t word_len);
void DWIN_ReadRTC(void);
void DWIN_WriteAlarmRecord(const DWIN_RTC_t *rtc, uint8_t ctrlAddr, uint8_t sensorIdx);
uint16_t DWIN_CalcCRC16(const uint8_t *pData, uint16_t len);

/* 队列与信号量（外部访问） */
extern QueueHandle_t     xDwinTxQueue;
extern QueueHandle_t     xDwinRxQueue;
extern SemaphoreHandle_t xDwinTxCompleteSem;

/* 接收任务解析状态 */
extern DWIN_HostStatus_t g_hostDwinStatus;
```

### 图标值定义

| 值 | 含义 |
|----|------|
| `DWIN_ICON_ALARM` (0x0003) | 在线且报警 |
| `DWIN_ICON_TROUBLE` (0x0002) | 在线且故障 |
| `DWIN_ICON_NORMAL` (0x0001) | 在线且正常 |
| `DWIN_ICON_OFFLINE` (0x0000) | 离线 |

## Controlboard UART2 从站任务（新增）

参考 Controlboard UART1 主站架构，UART2 实现为 Modbus 从站：

| 文件 | 职责 |
|------|------|
| `uart2_modbus_slave.c/h` | 底层驱动：ISR 处理（RXNE+IDLE+ORE）、复用 `Modbus_CRC16()`、独立接收/发送队列 |
| `modbus_slave_tasks.c/h` | `TaskSlaveRecv`（解析 FC 0x02/0x03 → 读寄存器 → 构造响应）+ `TaskSlaveSend`（发送响应 + 3.5字符间隔） |

**USART2 ISR**：已修改 `stm32f0xx_it.c` 的 `USART2_IRQHandler`，与 UART1 相同的 RXNE/IDLE/ORE 处理方式。

**从站地址过滤**：`TaskSlaveRecv` 比较 `raw.frame[0]` 与 `ModbusReg_GetBoardAddr()`，只响应发给本机地址的请求。

## 迪文屏驱动 (dwin.c/h)

### 通信参数

UART3, 115200 8N1，中断发送（`HAL_UART_Transmit_IT`）。所有发送（传感器数据、状态帧、控板地址）统一经过环形缓冲区队列，`DWIN_ProcessQueue()` 是 `HAL_UART_Transmit_IT` 的唯一调用点。

### 协议

0x82 写变量指令：
```
5A A5 [LEN] 82 [AH] [AL] [D0] [D1] ... [Dn]
LEN = 3 + data_len（数据字节数）
数据以大端序传输
```

### 统一队列框架

```
DWIN_UpdateSensor(slave, model)          TaskDwinIcons (每 500ms)
  │ 入队 SENSOR_DATA                        │ 入队 STATUS
  └─ DWIN_ProcessQueue()                    │ 入队 BOARD_ADDR
         ↓                                  └─ DWIN_ProcessQueue()
   DWIN_ProcessQueue()          ←── 也由 DWIN_FlushQueue() 调用 (每 50ms)
     ├─ dwin_tx_busy? → return
     ├─ 出队一个条目，switch(type):
     │    SENSOR_DATA  → DWIN_BuildSensorFrame()  填充静态 buf
     │    STATUS       → DWIN_BuildStatusFrame()  填充静态 buf
     │    BOARD_ADDR   → DWIN_BuildBoardAddrFrame() 填充静态 buf
     ├─ dwin_tx_busy = 1
     └─ HAL_UART_Transmit_IT(&huart3, buf, len)   [唯一调用点]

HAL_UART_TxCpltCallback (ISR)
  └─ dwin_tx_busy = 0   （仅清除标志，不调 HAL，避免 ISR 中 gState 仍为 BUSY）
```

驱动时机：
- **传感器数据**：接收任务 `DWIN_UpdateSensor()` → 入队 → `DWIN_ProcessQueue()`（若空闲立即发）
- **后备排空**：轮询任务每 50ms `DWIN_FlushQueue()` → `DWIN_ProcessQueue()`
- **图标/标志**：`TaskDwinIcons` 每 500ms 入队 → `DWIN_ProcessQueue()` 发第一帧 → 50ms 内 `FlushQueue` 发第二帧

### 变量区布局

| 地址范围 | 大小 | 内容 | 更新机制 |
|---------|------|------|---------|
| 0x1000-0x13FF | 1024 addr | 64 路传感器数值（每路 16 地址） | 每次 Modbus 响应后 `DWIN_UpdateSensor()` 入队，排队发送 |
| 0x1400-0x143F | 64 addr | 传感器状态图标（uint16/路） | `TaskDwinIcons` 每 500ms 批量更新 |
| 0x1440 | 1 addr | 开窗图标（传感器全局报警 OR 烟雾报警，与 WS 引脚一致） | 同上 |
| 0x1441 | 1 addr | 全局报警图标（仅传感器报警，与线圈位 128 一致） | 同上 |
| 0x1442 | 1 addr | 零地址存在标志 | 同上 |
| 0x1443 | 1 addr | 地址重复标志 | 同上 |
| 0x1444 | 1 addr | 烟雾报警器图标（与线圈位 129 一致） | 同上 |
| 0x1500 | 1 addr | 控板 DIP 地址 | 同上 |

### 每传感器 16 地址布局

传感器 n 的屏上地址 = `0x1000 + (n-1) × 16`：

| 偏移 | 内容 | 屏上类型 | 转换公式 |
|-----|------|---------|---------|
| +0..+1 | CO 值 | float | raw÷100 |
| +2..+3 | 风压值 | float | raw 直转 |
| +4 | 余压值 | uint16 | raw 直写 |
| +5 | eCO₂ | uint16 | raw 直写 |
| +6 | eCH₂O | uint16 | raw 直写 |
| +7 | TVOC | uint16 | raw 直写 |
| +8..+9 | PM2.5 | float | raw 直转 |
| +10..+11 | PM10 | float | raw 直转 |
| +12..+13 | 温度 | float | raw÷10 |
| +14..+15 | 湿度 | float | raw÷10 |

### 图标值定义

| 值 | 含义 |
|----|------|
| 0 | 传感器离线 |
| 1 | 在线+报警 |
| 2 | 在线+正常 |

### TaskDwinIcons 完整职责

每 500ms 执行：
1. 读取 Isolator(PA0) → `ModbusReg_SetSmokeAlarm()` — 更新线圈位 129
2. 入队 STATUS 帧（图标、开窗、全局报警、零地址、地址重复）→ `DWIN_ProcessQueue()`
3. 入队 BOARD_ADDR 帧 → `DWIN_ProcessQueue()`
4. **WS(PB0)** = `DWIN_GetGlobalAlarm() || ModbusReg_GetSmokeAlarm()` — 驱动继电器
5. **RED_LED(PB9)** = `DWIN_GetGlobalAlarm()` — 传感器全局报警指示
6. **ORANGE_LED(PB8)** = `ModbusReg_GetZeroAddrPresent() || ModbusReg_GetAddrConflict()`

## 寄存器映射表

控板在内存中维护两类寄存器，应用通过 `modbus_registers.h` 的 API 读写。

### 线圈寄存器（位操作）

| 位范围 | 内容 | API |
|--------|------|-----|
| 位 0-62 | 传感器 1-63 在线状态 | `ModbusReg_SetOnline/GetOnline(slave)` |
| 位 63-125 | 传感器 1-63 报警标志 | `ModbusReg_SetAlarm/GetAlarm(slave)` |
| 位 126 | 零地址传感器存在 | `ModbusReg_GetZeroAddrPresent()` / `ReadCoil(126)` |
| 位 127 | 地址重复标志 | `ModbusReg_GetAddrConflict()` / `ReadCoil(127)` |
| 位 128 | 全局报警标志（仅传感器） | `ModbusReg_SetGlobalAlarm()` / `ModbusReg_GetGlobalAlarmCoil()` → `ReadCoil(128)` |
| 位 129 | 烟雾报警器标志（Isolator 引脚输入） | `ModbusReg_SetSmokeAlarm()` / `ModbusReg_GetSmokeAlarm()` → `ReadCoil(129)` |

**线圈位 128（全局报警）**：由 `DWIN_BuildStatusFrame()` 每次扫描传感器更新，仅反映 Modbus 传感器的报警状态，不含烟雾报警器。

**线圈位 129（烟雾报警器）**：由 `TaskDwinIcons` 每 500ms 读取 Isolator(PA0) 引脚更新，低电平=报警，高电平=正常。

**WS(PB0) 与 DWIN 0x1440 开窗图标**：全局报警 OR 烟雾报警，两者任一为 1 即有效。

### 保持寄存器（16 位）

| 地址 | 内容 | API |
|------|------|-----|
| 1-63 | 传感器类型字节 (0x01-0x04) | `ModbusReg_SetType/GetType(slave)` |
| 64-70 | 传感器 1 数据（7 个寄存器） | `ModbusReg_SetData/GetData(slave, index)` |
| 71-77 | 传感器 2 数据 | index 范围 0-6 |
| ... | ... | |
| 64+(n-1)*7 ~ +6 | 传感器 n 数据 | |
| 505 | 控板 DIP 地址 | `ModbusReg_GetBoardAddr()` / `ReadHolding(505)` |

**数据区说明**：每个传感器固定分配 7 个寄存器。简单传感器（CO/风压/余压）只用 `index=0`（数值），`index=1..6` 为 0；7 合 1 传感器用满 7 个：0=eCO₂, 1=eCH₂O, 2=TVOC, 3=PM2.5, 4=PM10, 5=温度×10, 6=湿度×10。

## Modbus 通信协议

所有传感器使用 PowerBus 二总线 + Modbus RTU，**9600 baud，8N1**。

### 型号字节

| 型号 | 传感器 | 数据格式 |
|------|--------|---------|
| `0x01` | CO 传感器 | 2 字节，值÷100 = ppm |
| `0x02` | 风压传感器 | 2 字节，uint16（单位：Pa?） |
| `0x03` | 余压传感器 | 2 字节，单位 Pa |
| `0x04` | 7 合 1 传感器 | 14 字节，7 项数据 |

### 读寄存器 (0x03) 响应格式

```
简单传感器: [地址] 03 [字节数] [型号] [数值高8] [数值低8] [CRC]
7合1传感器: [地址] 03 0F 04 [eCO2] [eCH2O] [TVOC] [PM2.5] [PM10] [温度×10] [湿度×10] [CRC]
```

### 写寄存器 (0x06)

| 寄存器 | 用途 |
|--------|------|
| `0x0003` | 余压传感器零点校准（写 1 触发） |
| `0x0004` | 报警标志/红灯控制，通用（写 0x0000 关，写 0x0001 开） |

## 应用开发指引

### 读传感器值

```c
/* 数据已被接收任务自动存入寄存器，直接读取即可 */
if (ModbusReg_GetOnline(addr)) {
    uint16_t raw = ModbusReg_GetData(addr, 0);
    /* 根据 ModbusReg_GetType(addr) 判断是哪种传感器，按对应公式换算 */
}
```

### 写报警/校准

```c
ModbusMaster_WriteRegister(NULL, 1, 0x0004, 1, 0);  /* 开报警 */
ModbusMaster_WriteRegister(NULL, 2, 0x0003, 1, 0);  /* 零点校准 */
```

### 主动发送 Modbus 请求并等回调

```c
ModbusRequest_t req = { .slave_addr = 1, .func_code = 0x03,
                        .reg_addr = 0x0000, .reg_value = 0x0001 };
ModbusMaster_EnqueueRequest(&req);
/* 接收任务会自动解析并存入寄存器 */
```

## Hostboard 构建与烧录

```bash
# 首次配置（必须从 Hostboard 目录执行）
cd Hostboard && cmake --preset Debug

# 编译（必须从 Hostboard 目录执行）
cmake --build --preset Debug          # 或 cmake --build build/Debug

# 输出文件
build/Debug/Hostboard.elf
```

### 烧录（需连接 ST-LINK/V2）

Hostboard 没有 flash.sh 脚本，直接使用 OpenOCD：

```bash
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program build/Debug/Hostboard.elf verify reset exit"
```

### 调试（终端 GDB）

```bash
# 终端 1：OpenOCD GDB 服务器
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "gdb_port 3333"

# 终端 2：arm-none-eabi-gdb
arm-none-eabi-gdb build/Debug/Hostboard.elf
(gdb) target remote :3333
(gdb) load
(gdb) monitor reset halt
```

## Controlboard 构建与烧录

```bash
# 首次（必须从 Controlboard 目录执行）
cd Controlboard && cmake --preset Debug

# 编译（必须从 Controlboard 目录执行）
cmake --build --preset Debug          # 或 cmake --build build/Debug

# 编译 + 烧录（需连接 ST-LINK/V2）
cmake --build build/Debug --target flash

# 仅烧录（跳过编译检查）
./flash.sh

# 清理
rm -rf Controlboard/build

# 输出文件
build/Debug/Controlboard.elf
```

### 烧录工具

- **调试器**: ST-LINK/V2（USB ID `0483:3748`）
- **OpenOCD** 配置: `Controlboard/flash.cfg`（`stlink.cfg` + `stm32f0x.cfg`）
- **`flash.sh`** 封装: `program build/Debug/Controlboard.elf verify reset exit`
- `reset` 参数使芯片烧录后自动复位运行，无需手动按复位键

### 报警阈值

接收任务自动对 CO 和 7 合 1 传感器进行阈值判定，状态变化时发 0x06 写从机寄存器 0x0004。

| 传感器 | 参数 | 报警阈值 | 恢复阈值 | 说明 |
|--------|------|---------|---------|------|
| CO (0x01) | CO 浓度 | 值 > 2500 (>25ppm) | 值 < 2000 (<20ppm) | 含回滞，防频繁抖动 |
| 7合1 (0x04) | eCO₂ [0] | > 1000 | < 800 | 任一项超阈值 → 报警 |
| | eCH₂O [1] | > 80 | < 70 | |
| | TVOC [2] | > 600 | < 500 | |
| | PM2.5 [3] | > 35 | < 25 | |
| | PM10 [4] | > 50 | < 40 | |
| | 温度 [5] | — | — | 不参与报警 |
| | 湿度 [6] | — | — | 不参与报警 |
| 风压 (0x02) | — | — | — | 暂不设阈值 |
| 余压 (0x03) | — | — | — | 暂不设阈值 |

**7 合 1 报警逻辑**：当前未报警（报警标志 = 0）时，任一参数超报警阈值 → 触发报警。当前已报警时，**所有**参数必须低于恢复阈值才能解除报警。温湿度不参与判定。

### 调试

1. 安装 **Cortex-Debug** 扩展（marus25）
2. `F5` 启动调试，自动停在 `main` 入口
3. 全局变量可用 **LIVE WATCH** 实时监视
4. 局部变量需在断点暂停后在 **VARIABLES** 面板查看
5. **RTOS** 面板显示 FreeRTOS 任务状态和栈使用

**调试变量（Live Watch 可用）：**

| 变量 | 文件 | 含义 |
|------|------|------|
| `poll_debug_tx_tick` | `modbus_polling.c` | 轮询任务发出请求时的系统节拍（1 tick = 1ms） |
| `poll_debug_rx_tick` | `modbus_tasks.c` | 接收任务处理完一帧时的系统节拍 |
| `debug_rx_frame` | `modbus_tasks.c` | 最近一帧原始字节数组（Live Watch 数组展开查看） |
| `debug_rx_len` | `modbus_tasks.c` | 最近一帧长度 |
| `debug_rx_count` | `modbus_tasks.c` | 累计接收帧数（持续增长 = 正常收帧） |
| `coil_online` | `modbus_registers.c` | 64 位在线状态位图 |
| `reg_data[][]` | `modbus_registers.c` | 传感器数据寄存器数组 |

终端 GDB（无需 VSCode）：
```bash
# 终端 1：OpenOCD GDB 服务器
openocd -f interface/stlink-v2.cfg -f target/stm32f0x.cfg -c "gdb_port 3333"

# 终端 2：arm-none-eabi-gdb
arm-none-eabi-gdb build/Debug/Controlboard.elf
(gdb) target remote :3333
(gdb) load
(gdb) monitor reset halt
(gdb) break TaskModbusPoll
(gdb) continue
```

### 性能参数

| 性能参数 | 值 | 说明 |
|------|----|------|
| 响应超时 | 30ms | 在线传感器通常 10-50ms 回复 |
| 3.5 字符间隔 | 4ms | 9600 baud 帧间距 |
| Controlboard 轮询间隔 | 50ms | TaskModbusPoll（Controlboard）每次地址间延时 |
| Controlboard 顺序:在线穿插比 | 8:1 | 每 8 次顺序轮询插 1 次在线 |
| Controlboard 全址扫描周期 | ~3.6s | 72 次 × 50ms（全在线） |
| 离线传感器耗时 | ~34ms/个 | 30ms 超时 + 4ms 间隔 |
| 离线检测阈值 | 连续 3 轮无响应 | 约 3 个完整扫描周期 |
| 地址重复检测窗口 | 连续 3 轮每轮错误 > 2 | 约 3 个完整扫描周期 |
| DWIN 图标更新 | 每 500ms | TaskDwinIcons 更新图标+标志+地址 |
| DWIN 传感器数据 | 逐帧发送 | 每次 Modbus 响应后排队发送，中断不阻塞 |
| GREEN_LED | 每 10 次轮询翻转 | ~1Hz，低电平点亮 |
| RED_LED / ORANGE_LED | 每 500ms | TaskDwinIcons 根据状态更新，低电平点亮 |
| **Hostboard 轮询间隔** | **50ms/地址** | **TaskModbusPoll 顺序扫描地址 0-128，FC 0x02 读 130 bits** |
| **Hostboard 完整一轮** | **~6.45s** | **129 地址 × 50ms** |
| **Hostboard 零地址/重复地址更新** | **每 6.45s** | **每轮回绕时 StepCycle() 更新一次** |
| **Hostboard 重复地址判定窗口** | **~19.35s** | **3 轮 × 6.45s** |
| UART2 从站响应 | 由 Hostboard 请求触发 | TaskSlaveRecv 解析 → TaskSlaveSend 回复 |
| UART2 从站过滤 | 仅响应本机 DIP 地址 | `slave_addr == ModbusReg_GetBoardAddr()` |
| USART1 TX 完成超时 | 50ms | 中断发送等 TX 完成信号量 |
| UART2 TX 完成超时 | 200ms | 从站发送等 TX 完成信号量（大帧 130 字节 @ 9600 ≈ 135ms） |
| USART3 DWIN TX 完成超时 | 50ms | TaskDwinTx 等待 xDwinTxCompleteSem |
| DWIN 帧最大长度 | 128 字节 | DwinFrame_t.data 缓冲区上限 |
| DWIN 接收队列深度 | 4 帧 | xDwinRxQueue 最大排队数 |
| **Hostboard DWIN 图标更新** | **每 500ms** | **TaskDwinIcons 更新 5 帧：4 帧控制器图标 + 1 帧系统状态图标** |

## 风压传感器固件架构

见 `风压传感器/README.md` 和代码注释。核心要点：
- **MCU**: STC8H1K28-36I-LQFP32, 11.0592MHz
- **工具链**: SDCC `-mmcs51 --std-c99 --model-small`
- **架构**: 超循环（无 RTOS），主循环轮询双 UART + 数码管扫描
- **压力模块**: UART1 115200，自定义 `0xFE`/`0xDC` 帧协议（非 Modbus），模块自主上报
- **控制器总线**: UART2 9600, Modbus RTU 从机，地址由 6-bit DIP 设定
- **Modbus 协议**: 0x03 读返回风压值(型号 0x02)，0x06 写 0x0004 控制红灯

### 构建与烧录

```bash
# 编译（从 风压传感器/ 目录执行）
cd 风压传感器 && ./tools/build.sh

# 烧录（需连接 USB 串口转换器，如 CH340/FTDI）
./tools/flash.sh                          # 自动检测串口
./tools/flash.sh -p /dev/ttyUSB0          # 指定串口

# 依赖安装
sudo pacman -S sdcc                       # SDCC 编译器
pip install stcgal                        # STC 烧录工具
```

输出文件：
```
build/SensorBoard.ihx
build/SensorBoard.hex
```

烧录时在提示 `Power cycle the MCU` 处断开目标板电源后重新上电，STC 芯片特有流程。

### 7合1传感器烧录（Linux）

7合1传感器用 Keil 在 Windows 上编译，`hex文件/main.hex` 是输出。Linux 烧录脚本：

```bash
# 自动检测串口
cd 7合1传感器 && ./tools/flash.sh

# 指定串口
./tools/flash.sh -p /dev/ttyUSB0
```

### STC8H1K28 未校准 IRC 问题

STC8H1K28（风压传感器和 7合1传感器使用）的 IRC 振荡器出厂未校准，stcgal 烧录时会报：

```
Protocol error: uncalibrated, please provide a trim value
```

**解决方案**：用 `-t` 参数指定实际频率（kHz），而非 `-o trim=val`：

```bash
# 11.0592MHz → 11059 kHz（注意 1000 倍换算！）
stcgal -p /dev/ttyUSB0 -b 115200 -t 11059 SensorBoard.hex
```

两个传感器的 `flash.sh` 已内置默认 TRIM_KHZ=11059。若使用不同频率的 MCU，用 `-t` 覆盖：

```bash
./tools/flash.sh -p /dev/ttyUSB0 -t 22118   # 22.1184MHz
```

## .gitignore 说明

仓库有两层 `.gitignore`，规则的并集决定了哪些文件不被跟踪：

| 位置 | 忽略内容 |
|------|---------|
| 根目录 `.gitignore` | `*.code-workspace`, `.claude/` |
| `Controlboard/.gitignore` | `.vscode/`, `build/`, `.clangd`, `*.sh`, `*.json`, `*.drawio`, `*.markdown`, `*.bkp` |

> **注意**：`Controlboard/.gitignore` 排除了 `*.sh` 和 `*.json`，因此 `flash.sh`、`CMakePresets.json`、`launch.json` 等文件不会被 git 跟踪。这是有意为之，这些文件属于本地开发环境配置，不随仓库分发。

## 工作规范

### Git 操作

用户要求自行控制 git add/commit/push 等操作。CLAUDE 不应擅自执行 git 提交、推送、cherry-pick 或合并。允许使用 git status/diff/log 等只读命令查看状态。修改文件后，列出变更让用户决定何时提交。

## 已知问题

### CubeMX 重新生成后中断冲突

CubeMX 每次重新生成 `Core/Src/stm32f0xx_it.c` 时，`SVC_Handler`、`PendSV_Handler`、`SysTick_Handler` 会被覆盖，与 FreeRTOS 移植层冲突。

**解决方案**：`stm32f0xx_it.c` 的 `USER CODE BEGIN 0` 区块已有：
```c
#pragma weak SVC_Handler
#pragma weak PendSV_Handler
#pragma weak SysTick_Handler
```
CubeMX 会保留此区块，`#pragma weak` 使三个函数变为弱符号，FreeRTOS 的强符号自动覆盖它们。

### HAL_IncTick() 与 FreeRTOS SysTick

FreeRTOS 的 `port.c`（`Drivers/Middlewares/FreeRTOS/portable/GCC/ARM_CM0/port.c`）中提供的 `SysTick_Handler` 末尾已额外调用 `HAL_IncTick()`，以保证 HAL 时间基准正常工作。CubeMX 不会覆盖 Middlewares 目录下的文件。

### USART1 中断风暴：ORE + IDLE 标志清除方式

**症状**：PowerBus 二总线空闲时，CPU 卡死在 `USART1_IRQHandler` → `HAL_UART_IRQHandler` 中，FreeRTOS 任务无法运行，GREEN_LED 不闪烁。

**根因**（两个问题叠加）：

1. **ORE（溢出错误）处理**：CubeMX 生成的代码将 ORE 留给 `HAL_UART_IRQHandler` 处理。但 HAL 的 ORE 处理路径会调用 `UART_EndRxTransfer()`，该函数**清除 RXNEIE 中断使能位**，导致 Modbus 自定义 RXNE 接收静默失效。总线噪声持续触发 IDLE 中断，接收任务处理空帧，系统陷入病态。

2. **IDLE 标志清除方式**：CubeMX 生成的 IDLE 清除序列是 `read ISR → read RDR`（F1 兼容方式），但 **STM32F0（新版 USART IP）必须写 ICR 寄存器清除**这两个标志。读序列无法清除，IDLE 持续置位导致无限中断。

**解决方案**：在 `stm32f0xx_it.c` 的 `USART1_IRQHandler` 中，`USER CODE BEGIN USART1_IRQn 0` 区块内，在调用 `HAL_UART_IRQHandler` 之前处理 ORE 和 IDLE 标志：

```c
/* IDLE — F0 必须写 ICR，不能用 F1 的 ISR→RDR 序列 */
if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
{
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    ModbusMaster_RxIdleHandler();
}

/* ORE — 必须在 HAL_UART_IRQHandler 之前清除，否则 HAL 会关闭 RXNEIE */
if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
{
    __HAL_UART_CLEAR_OREFLAG(&huart1);
}
```

**CubeMX 注意事项**：CubeMX 每次重新生成 `stm32f0xx_it.c` 时，`USER CODE BEGIN USART1_IRQn 0` 内的代码块会被保留。但如果 CubeMX 重新生成了整个文件（如升级 HAL 库），请在 `USER CODE BEGIN 0` 中确认保留此修复。

### FreeRTOS SysTick 与 HAL 时基冲突

`port.c`（`Drivers/Middlewares/FreeRTOS/portable/GCC/ARM_CM0/port.c`）的 `SysTick_Handler` 做了两个重要修改：

1. **`HAL_IncTick()` 调用挪到最前**：原始代码将其放在 `portCLEAR_INTERRUPT_MASK_FROM_ISR()` 之后（临界区外）。改为放在函数开始、`portSET_INTERRUPT_MASK_FROM_ISR()` 之前。这保证 `HAL_IncTick()` 总是在 SysTick 中断时被调用，不受 FreeRTOS 临界区影响，从而 `HAL_Delay()` 和其他 HAL 时基相关功能在临界区内也能正常推进。

2. **调度器未启动保护**：`HAL_Init()` 在 `main()` 一开始就启动了 SysTick（1kHz），而 FreeRTOS 内核此时尚未初始化（`pxDelayedTaskList` 等链表为 NULL）。在调度器启动前调用 `xTaskIncrementTick()` 会解引用空链表导致 HardFault。添加了 `xTaskGetSchedulerState()` 检查，调度器未启动时仅维护 HAL 时基后直接返回。

**注意**：CubeMX 不会覆盖 Middlewares 目录下的文件，因此此修改在 CubeMX 重新生成后仍然保留。

### 接收任务信号量管理

接收任务（`TaskModbusReceive` / `TaskModbusRecv`）**仅在 CRC 校验通过时**调用 `xSemaphoreGive()` 释放响应信号量。CRC 失败时不释放，发送任务超时后自动进入下一轮请求。这防止了总线噪声触发假 IDLE 导致信号量被非法置位。

受影响文件（两侧）：
- `Controlboard/Core/Src/modbus_tasks.c` — `TaskModbusReceive`
- `Hostboard/Core/Src/modbus_master_tasks.c` — `TaskModbusRecv`

### 中断发送架构（TX 完成信号量）

两侧的 USART1 和 Controlboard 的 UART2 均使用 **`HAL_UART_Transmit_IT` + TX 完成信号量** 替代阻塞轮询发送。任务在等待 TX 完成时可以被高优先级任务抢占，减少系统延迟。

| UART | TX 完成信号量 | 超时 | 回调位置 |
|------|-------------|------|---------|
| Controlboard USART1 (master) | `xModbusTxCompleteSem` | 50ms | `dwin.c` → `HAL_UART_TxCpltCallback` |
| Controlboard UART2 (slave) | `xSlaveTxCompleteSem` | 200ms | `dwin.c` → `HAL_UART_TxCpltCallback` |
| Hostboard USART1 (master) | `xMasterTxCompleteSem` | 50ms | `modbus_callbacks.c` → `HAL_UART_TxCpltCallback` |
| Hostboard USART3 (DWIN) | `xDwinTxCompleteSem` | 50ms | `modbus_callbacks.c` → `HAL_UART_TxCpltCallback` |

### 64 位线圈寄存器跨任务保护

`coil_online` 和 `coil_alarm` 是 `uint64_t` 变量，在 Cortex-M0 上读写需要多条指令。`ModbusReg_SetOnline`/`GetOnline`/`SetAlarm`/`GetAlarm` 四个函数使用 `taskENTER_CRITICAL`/`taskEXIT_CRITICAL` 保护，防止 `TaskDwinIcons`（优先级 1）读时被 `TaskModbusReceive`（优先级 2）写导致读撕裂。

### DWIN 环形缓冲区竞态保护

`DWIN_Enqueue` 和 `DWIN_Dequeue` 使用 `taskENTER_CRITICAL`/`taskEXIT_CRITICAL` 保护队列指针操作。`DWIN_Enqueue` 被 `TaskModbusReceive`（优先级 2）和 `TaskDwinIcons`（优先级 1）同时调用，无保护会导致指针写入丢失。

### ORE 中断重置 rx_index

USART1 和 USART2 的 ORE（溢出错误）中断处理中增加了 `ModbusMaster_ResetRx()` / `ModbusSlave_ResetRx()` 调用，将接收缓冲区索引 `rx_index` 清零。防止 ORE 导致字节丢失后，后续字节与旧帧残余拼接成错误帧。受影响的 ISR：`stm32f0xx_it.c` → `USART1_IRQHandler`、`USART2_IRQHandler`。

### FE/NE/PE → TXE 中断风暴（中断发送专用）

**影响范围：** Controlboard USART1/USART2 + Hostboard USART1/USART3

**根因：** 将 `HAL_UART_Transmit()` 阻塞发送改为 `HAL_UART_Transmit_IT()` 中断发送后启用 TXEIE。`HAL_UART_IRQHandler` 检测到 `errorflags != 0`（FE/NE/PE 置位）且 RXNEIE 使能时，进入错误分支**提前 return**，永不处理 TXE 分支 → TXEIE 永不关闭 → 无限 TXE 中断风暴 → 发送卡死。

**修复：** 在调用 `HAL_UART_IRQHandler()` 之前，手动写 ICR（F0）/ SR（F4）清除 FE/NE/PE：

```c
/* Controlboard (stm32f0xx_it.c) — F0 系列写 ICR */
if (huart1.Instance->ISR & (USART_ISR_FE | USART_ISR_NE | USART_ISR_PE))
{
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    ModbusMaster_ResetRx();
}

/* Hostboard (stm32f4xx_it.c) — F4 系列写 SR，语义相同 */
if (huart1.Instance->SR & (USART_SR_FE | USART_SR_NE | USART_SR_PE))
{
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    ModbusMaster_ResetRx();
}
```

与 ORE 相同模式——所有 UART 错误标志（ORE/FE/NE/PE）都必须在 HAL 处理之前自行清除。

**受影响文件：**
- `Controlboard/Core/Src/stm32f0xx_it.c` — USART1_IRQHandler、USART2_IRQHandler
- `Hostboard/Core/Src/stm32f4xx_it.c` — USART1_IRQHandler、USART3_IRQHandler
- `Hostboard/Core/Src/uart1_modbus_master.c` — 新增 `ModbusMaster_ResetRx()`
