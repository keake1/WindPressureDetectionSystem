# Hostboard 多 Controlboard 轮询实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 改写 Hostboard 从单 Controlboard 轮询扩展为轮询 128 个 Controlboard 的离散输入寄存器

**Architecture:** 新增 `hostboard_registers.c/h` 模块存储 128 路 × 17 字节线圈数据，维护零地址和重复地址检测。重写 `modbus_polling.c` 为 50ms 间隔的地址 0-128 顺序扫描。修改接收任务将 CRC 通过的响应存入寄存器模块。

**Tech Stack:** STM32F407VET6, FreeRTOS, Modbus RTU, PowerBus 二总线

---

## 文件清单

| 操作 | 文件 | 说明 |
|------|------|------|
| **新建** | `Hostboard/Core/Inc/hostboard_registers.h` | 寄存器模块头文件 |
| **新建** | `Hostboard/Core/Src/hostboard_registers.c` | 寄存器模块实现 |
| **修改** | `Hostboard/Core/Inc/modbus_polling.h` | 移除单地址宏 |
| **修改** | `Hostboard/Core/Src/modbus_polling.c` | 重写为地址 0-128 轮询 |
| **修改** | `Hostboard/Core/Src/modbus_master_tasks.c` | 接收任务：存储数据和错误统计 |
| **修改** | `Hostboard/CMakeLists.txt` | 添加新源文件 |

---

### Task 1: 新建 `hostboard_registers.h`

**文件：** `Hostboard/Core/Inc/hostboard_registers.h`

- [ ] **Step 1: 创建头文件**

写入以下完整内容：

```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hostboard_registers.h
  * @brief          : Hostboard 寄存器模块 — 存储 Controlboard 线圈数据
  *
  * 管理 128 个 Controlboard (地址 1-128) 的离散输入寄存器数据，
  * 以及零地址检测和地址重复检测的全局标志。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __HOSTBOARD_REGISTERS_H__
#define __HOSTBOARD_REGISTERS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported constants --------------------------------------------------------*/

/** @defgroup HostReg_Constants 地址范围和线圈布局 */
/** @{ */
#define MAX_CTRLBD_ADDR     128     /* 最大 Controlboard 地址 */
#define COIL_BYTE_COUNT     17      /* 130 bits (0-129) = 16.25 → 17 字节 */

/* 线圈位偏移定义（对应 Controlboard 的 coil 寄存器映射） */
#define COIL_OFFSET_ONLINE      0    /* 位 0-62:  传感器 1-63 在线状态 */
#define COIL_OFFSET_ALARM       63   /* 位 63-125: 传感器 1-63 报警标志 */
#define COIL_OFFSET_ZERO_ADDR   126  /* 位 126:  零地址传感器存在 */
#define COIL_OFFSET_ADDR_CONF   127  /* 位 127:  地址重复标志 */
#define COIL_GLOBAL_ALARM       128  /* 位 128:  全局报警（任一传感器报警） */
#define COIL_SMOKE_ALARM        129  /* 位 129:  烟雾报警器状态 */
/** @} */

/* USER CODE BEGIN EConst */

/* USER CODE END EConst */

/* Exported functions prototypes ---------------------------------------------*/

/** @defgroup HostReg_Data_Storage 线圈数据存储与读取 */
/** @{ */
void    HostReg_StoreCoilData(uint8_t addr, const uint8_t *data, uint8_t len);
uint8_t HostReg_GetCoilByte(uint8_t addr, uint8_t byte_idx);
uint8_t HostReg_GetCoilBit(uint8_t addr, uint16_t bit_idx);
/** @} */

/** @defgroup HostReg_Zero_Addr 零地址检测 */
/** @{ */
void    HostReg_RecordZeroAddrResponse(void);
uint8_t HostReg_GetZeroAddrPresent(void);
/** @} */

/** @defgroup HostReg_Conflict 地址重复检测 */
/** @{ */
void    HostReg_RecordError(void);
void    HostReg_StepCycle(void);
uint8_t HostReg_GetAddrConflict(void);
/** @} */

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __HOSTBOARD_REGISTERS_H__ */
```

- [ ] **Step 2: 提交**

```bash
git add Hostboard/Core/Inc/hostboard_registers.h
git commit -m "feat: 添加 hostboard_registers.h 头文件

定义 MAX_CTRLBD_ADDR (128)、COIL_BYTE_COUNT (17)、线圈位偏移常量，
以及数据存储、零地址检测、重复地址检测的 API 声明。

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 2: 新建 `hostboard_registers.c`

**文件：** `Hostboard/Core/Src/hostboard_registers.c`

- [ ] **Step 1: 创建源文件**

写入以下完整内容：

```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : hostboard_registers.c
  * @brief          : 128 路 Controlboard 线圈数据存储 + 检测逻辑
  *
  * 存储由接收任务解析的 FC 0x02 响应数据，提供查询 API。
  * 维护零地址检测（三轮 OR）和地址重复检测（三轮错误计数累加 > 2）。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "hostboard_registers.h"
#include <string.h>

/* USER CODE BEGIN 0 */

/* ==================== 零地址检测 ==================== */

/**
 * @brief  零地址检测状态
 *
 * 每轮完整扫描 (0-128) 中: zero_addr_found_this_cycle 记录是否收到 addr0 有效响应
 * StepCycle() 时移入 zero_addr_history[3] 环形缓冲
 * zero_addr_present = history[0] || history[1] || history[2]
 */
static uint8_t  zero_addr_found_this_cycle;     /* 本轮发现标志 */
static uint8_t  zero_addr_history[3];            /* 三轮历史 */
static uint8_t  zero_addr_present;               /* 综合结果 */

/* ==================== 重复地址检测 ==================== */

/**
 * @brief  地址重复检测状态
 *
 * 每轮扫描中: crc_error_count 累计非超时错误
 * StepCycle() 时移入 crc_error_history[3] 环形缓冲
 * 三轮累加 > 2 → addr_conflict_flag = 1
 */
static uint32_t crc_error_count;                 /* 本轮错误计数 */
static uint32_t crc_error_history[3];            /* 三轮历史 */
static uint8_t  addr_conflict_flag;              /* 综合结果 */

/* ==================== 轮次跟踪 ==================== */

static uint8_t  current_cycle;                   /* 0, 1, 2 循环 */

/* ==================== 线圈数据存储 ==================== */

/**
 * board_coil_data[0] 预留（地址 0 有效响应不计入存储）
 * board_coil_data[1..128] 对应 Controlboard 地址 1-128
 */
static uint8_t  board_coil_data[MAX_CTRLBD_ADDR + 1][COIL_BYTE_COUNT];

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 1 */

/* ==================== 数据存储/读取 API ==================== */

/**
 * @brief  存储 Controlboard 的线圈数据
 * @param  addr  Controlboard 地址 (1-128)
 * @param  data  线圈数据指针
 * @param  len   数据长度（上限 COIL_BYTE_COUNT）
 */
void HostReg_StoreCoilData(uint8_t addr, const uint8_t *data, uint8_t len)
{
    if (addr < 1 || addr > MAX_CTRLBD_ADDR)
        return;
    if (len > COIL_BYTE_COUNT)
        len = COIL_BYTE_COUNT;
    memcpy(board_coil_data[addr], data, len);
}

/**
 * @brief  读取 Controlboard 线圈数据的一个字节
 * @param  addr       Controlboard 地址 (1-128)
 * @param  byte_idx   字节索引 (0-16)
 * @return 线圈数据字节，越界返回 0
 */
uint8_t HostReg_GetCoilByte(uint8_t addr, uint8_t byte_idx)
{
    if (addr < 1 || addr > MAX_CTRLBD_ADDR)
        return 0;
    if (byte_idx >= COIL_BYTE_COUNT)
        return 0;
    return board_coil_data[addr][byte_idx];
}

/**
 * @brief  读取 Controlboard 线圈数据的一个位
 * @param  addr       Controlboard 地址 (1-128)
 * @param  bit_idx    位索引 (0-129)
 * @return 线圈位值 (0/1)，越界返回 0
 */
uint8_t HostReg_GetCoilBit(uint8_t addr, uint16_t bit_idx)
{
    if (addr < 1 || addr > MAX_CTRLBD_ADDR)
        return 0;
    if (bit_idx >= COIL_BYTE_COUNT * 8)
        return 0;
    return (board_coil_data[addr][bit_idx / 8] >> (bit_idx % 8)) & 1;
}

/* ==================== 零地址检测 API ==================== */

/**
 * @brief  记录收到地址 0 的 CRC 有效响应
 */
void HostReg_RecordZeroAddrResponse(void)
{
    zero_addr_found_this_cycle = 1;
}

/**
 * @brief  查询零地址是否存在
 * @retval 1 存在 Controlboard 误设为地址 0
 */
uint8_t HostReg_GetZeroAddrPresent(void)
{
    return zero_addr_present;
}

/* ==================== 重复地址检测 API ==================== */

/**
 * @brief  记录一次非超时错误（CRC 失败/帧异常）
 */
void HostReg_RecordError(void)
{
    crc_error_count++;
}

/**
 * @brief  查询是否存在地址重复
 * @retval 1 存在地址重复
 */
uint8_t HostReg_GetAddrConflict(void)
{
    return addr_conflict_flag;
}

/* ==================== 周期更新 ==================== */

/**
 * @brief  每轮完整扫描结束时调用
 *
 * 1. 将本轮 CRC 错误数存入历史环形缓冲，清零计数
 * 2. 三轮累加 > 2 → addr_conflict_flag = 1
 * 3. 将本轮零地址发现标志移入历史，清零
 * 4. zero_addr_present = 三轮 OR
 */
void HostReg_StepCycle(void)
{
    /* 重复地址检测：三轮错误计数累加 > 2 */
    crc_error_history[current_cycle] = crc_error_count;
    crc_error_count = 0;

    uint32_t sum = crc_error_history[0] + crc_error_history[1] + crc_error_history[2];
    addr_conflict_flag = (sum > 2) ? 1 : 0;

    /* 零地址检测：三轮 OR */
    zero_addr_history[current_cycle] = zero_addr_found_this_cycle;
    zero_addr_found_this_cycle = 0;
    zero_addr_present = zero_addr_history[0]
                     || zero_addr_history[1]
                     || zero_addr_history[2];

    current_cycle = (current_cycle + 1) % 3;
}

/* USER CODE END 1 */
```

- [ ] **Step 2: 提交**

```bash
git add Hostboard/Core/Src/hostboard_registers.c
git commit -m "feat: 添加 hostboard_registers.c 实现

128 路线圈数据存储、零地址检测（三轮 OR）、
重复地址检测（三轮错误计数 > 2）、API 实现。

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 3: 改写 `modbus_polling.h`

**文件：** `Hostboard/Core/Inc/modbus_polling.h`

- [ ] **Step 1: 移除单地址宏，清理头文件**

将整个文件替换为：

```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : modbus_polling.h
  * @brief          : Modbus 轮询任务 — 定时读取所有 Controlboard 离散输入
  *
  * 轮询内容：FC 0x02 读取 0-129 位离散输入寄存器，获取每个 Controlboard
  * 的传感器在线/报警状态、零地址/重复地址/全局报警标志。
  *
  * 轮询地址范围：0-128（含地址 0 用于零地址检测）
  * 轮询间隔：50ms（每个地址之间固定延时）
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MODBUS_POLLING_H__
#define __MODBUS_POLLING_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported functions prototypes ---------------------------------------------*/

void TaskModbusPoll(void *arg);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_POLLING_H__ */
```

- [ ] **Step 2: 提交**

```bash
git add Hostboard/Core/Inc/modbus_polling.h
git commit -m "refactor: 清理 modbus_polling.h

移除 CONTROLBOARD_ADDR 和 POLL_INTERVAL_MS 宏（由动态轮询替代）。

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 4: 改写 `modbus_polling.c`

**文件：** `Hostboard/Core/Src/modbus_polling.c`

- [ ] **Step 1: 重写轮询任务**

```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : modbus_polling.c
  * @brief          : Modbus 轮询任务 — 顺序扫描地址 0-128
  *
  * 任务流：
  *   TaskModbusPoll (每 50ms)
  *     └── 构造 FC 0x02 请求 (slave_addr=current, 130 bits)
  *           └── ModbusMaster_EnqueueRequest() → [xMasterSendQueue]
  *                 └── 发送任务发帧 → Controlboard 回复 → 接收任务解析
  *
  * 地址回绕时 (current_addr > 128)：
  *   └── HostReg_StepCycle() — 更新零地址标志和重复地址标志
  *
  * 参考 Controlboard/Core/Src/modbus_polling.c 架构。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "modbus_polling.h"
#include "uart1_modbus_master.h"
#include "hostboard_registers.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==================== 轮询任务 ==================== */

void TaskModbusPoll(void *arg)
{
    (void)arg;
    uint16_t current_addr = 0;

    for (;;)
    {
        /* ---- 轮询间隔 50ms ---- */
        vTaskDelay(pdMS_TO_TICKS(50));

        /* ---- 构造 FC 0x02 请求：读 130 bits (0-129) ---- */
        ModbusMasterRequest_t req;
        req.slave_addr = (uint8_t)current_addr;
        req.func_code  = MODBUS_FUNC_READ_DISCRETE_INPUTS;
        req.reg_addr   = 0;
        req.reg_value  = 130;
        ModbusMaster_EnqueueRequest(&req);

        /* ---- 地址推进 ---- */
        current_addr++;
        if (current_addr > MAX_CTRLBD_ADDR)
        {
            current_addr = 0;
            HostReg_StepCycle();
        }
    }
}

/* USER CODE END 0 */
```

- [ ] **Step 2: 提交**

```bash
git add Hostboard/Core/Src/modbus_polling.c
git commit -m "refactor: 重写 modbus_polling.c 支持多地址轮询

从每 1000ms 轮询单个 Controlboard 改为每 50ms 顺序扫描地址 0-128，
回绕时调用 HostReg_StepCycle() 更新检测标志。

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 5: 修改 `modbus_master_tasks.c` 接收任务

**文件：** `Hostboard/Core/Src/modbus_master_tasks.c`

- [ ] **Step 1: 添加 `#include "hostboard_registers.h"`**

在文件头部 includes 区域添加：

```c
#include "hostboard_registers.h"
```

放在 `#include "modbus_master_tasks.h"` 之后：

```c
#include "modbus_master_tasks.h"
#include "uart1_modbus_master.h"
#include "hostboard_registers.h"
#include "usart.h"
```

- [ ] **Step 2: 修改接收任务的数据解析逻辑**

将接收任务中 `if (crc_ok)` 块（当前第 118-133 行）替换为：

```c
        if (crc_ok)
        {
            uint8_t  slave_addr = raw.frame[0];
            uint8_t  func_code  = raw.frame[1];
            uint8_t  byte_cnt   = raw.frame[2];

            /* 地址 0：记录零地址响应，不存入数组 */
            if (slave_addr == 0)
            {
                HostReg_RecordZeroAddrResponse();
            }
            /* 地址 1-128：验证是 FC 0x02 响应且数据长度合理后存入寄存器 */
            else if (slave_addr >= 1 && slave_addr <= MAX_CTRLBD_ADDR
                     && func_code == 0x02
                     && byte_cnt > 0
                     && byte_cnt <= COIL_BYTE_COUNT
                     && (3 + byte_cnt + 2) <= raw.length)
            {
                HostReg_StoreCoilData(slave_addr, &raw.frame[3], byte_cnt);
            }
            /* else: 功能码异常或格式错误 → 忽略不计 */

            /* 仅 CRC 通过时释放信号量：通知发送任务可以发下一帧 */
            xSemaphoreGive(xMasterRxSem);
        }
        else
        {
            /* CRC 失败且收到过数据 → 计入重复地址检测 */
            if (raw.length > 0)
                HostReg_RecordError();
        }
```

- [ ] **Step 3: 提交**

```bash
git add Hostboard/Core/Src/modbus_master_tasks.c
git commit -m "feat: 修改接收任务支持多 Controlboard 数据存储

CRC 通过 + 地址 0 → RecordZeroAddrResponse()
CRC 通过 + 地址 1-128 → StoreCoilData()
CRC 失败 + 有数据 → RecordError()

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 6: 更新 CMakeLists.txt

**文件：** `Hostboard/CMakeLists.txt`

- [ ] **Step 1: 添加 `hostboard_registers.c` 到源文件列表**

在第 49 行附近（`modbus_callbacks.c` 之后）添加：

```cmake
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/hostboard_registers.c
```

期望的文件顺序（按字母排列）：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/freertos_hooks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/hostboard_registers.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/uart1_modbus_master.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/modbus_callbacks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/modbus_master_tasks.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/modbus_polling.c
    ...
```

- [ ] **Step 2: 提交**

```bash
git add Hostboard/CMakeLists.txt
git commit -m "build: 添加 hostboard_registers.c 到 CMakeLists.txt

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 7: 编译验证

- [ ] **Step 1: 编译 Hostboard**

```bash
cd /home/keake/Projects/WindPressureDetectionSystem/Hostboard && cmake -B build/Debug 2>&1 && cmake --build build/Debug 2>&1
```

期望输出：0 errors, 0 warnings

- [ ] **Step 2: 验证新文件存在**

```bash
ls -la Hostboard/Core/Src/hostboard_registers.c Hostboard/Core/Inc/hostboard_registers.h
```

- [ ] **Step 3: 提交最终状态**

```bash
git add -A && git status
# 确认只有预期文件被改动
git commit -m "feat: Hostboard 多 Controlboard 轮询功能完成

- 新增 hostboard_registers.c/h 存储 128 路线圈数据
- 实现零地址检测（三轮 OR）和重复地址检测（三轮错误 > 2）
- 重写 modbus_polling.c 为 50ms 间隔地址 0-128 顺序扫描
- 修改接收任务解析 FC 0x02 响应并存入寄存器模块

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```
