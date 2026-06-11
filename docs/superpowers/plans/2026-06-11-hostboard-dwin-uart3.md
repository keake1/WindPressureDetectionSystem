# Hostboard USART3 迪文屏收发驱动实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为 Hostboard USART3 实现迪文屏双向收发驱动 — 两个 FreeRTOS 任务（TX/RX）+ ISR 状态机组装 + 队列驱动

**Architecture:** TX 路径：应用 → UART3_Send()入队 → TaskDwinTx → Transmit_IT → 信号量等待。RX 路径：USART3 RXNE ISR 状态机（0x5A 0xA5 帧头 + LEN 检测）→ xDwinRxQueue → TaskDwinRx（将来解析）。全双工，无握手。

**Tech Stack:** C (STM32F407, FreeRTOS), USART3 @115200 8N1

---

## 文件结构

| 文件 | 操作 | 职责 |
|------|------|------|
| `Core/Inc/dwin.h` | 修改 | 添加 `DwinFrame_t`、队列/信号量 extern、函数声明、FreeRTOS 头文件 |
| `Core/Src/dwin.c` | 修改 | 添加 `UART3_Send()`、`DwinRxByteHandler()` 状态机、`DwinRxReset()`、`Dwin_InitQueues()` |
| `Core/Inc/dwin_tasks.h` | **新建** | `TaskDwinTx` / `TaskDwinRx` 声明 |
| `Core/Src/dwin_tasks.c` | **新建** | TX/RX 任务实现 |
| `Core/Src/stm32f4xx_it.c` | 修改 | `USART3_IRQHandler` 添加 RXNE + ORE 处理 |
| `Core/Src/modbus_callbacks.c` | 修改 | `HAL_UART_TxCpltCallback` 添加 huart3 分支 |
| `Core/Src/main.c` | 修改 | 添加 `Dwin_InitQueues()` + `xTaskCreate` |
| `CMakeLists.txt` | 修改 | 添加 `dwin_tasks.c` |

---

### Task 1: dwin.h — 类型定义与声明扩展

**Files:**
- Modify: `Hostboard/Core/Inc/dwin.h`

- [ ] **Step 1: 添加 FreeRTOS 头文件引用和类型定义**

在 `dwin.h` 的 `USER CODE BEGIN Includes` 区块添加：

```c
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
/* USER CODE END Includes */
```

在 `USER CODE BEGIN EConst` 或现有宏定义区域后添加常量：

```c
/* USER CODE BEGIN EConst */

/* ==================== USART3 迪文屏队列参数 ==================== */
#define DWIN_TX_FRAME_MAX   128
#define DWIN_RX_FRAME_MAX   128
#define DWIN_TX_QUEUE_LEN   8
#define DWIN_RX_QUEUE_LEN   4

/* USER CODE END EConst */
```

在 `USER CODE BEGIN ET` 区块添加 DwinFrame_t 类型：

```c
/* USER CODE BEGIN ET */

/**
 * @brief  迪文屏帧数据结构（发送/接收队列项）
 */
typedef struct {
    uint8_t  data[128];   /* 帧数据（含 0x5A 0xA5 帧头） */
    uint16_t length;       /* 帧长度（字节数） */
} DwinFrame_t;

/* USER CODE END ET */
```

在现有函数声明之后（`USER CODE BEGIN EFP` 区块）添加队列/信号量 extern 和函数声明：

```c
/* USER CODE BEGIN EFP */

/* ==================== USART3 队列与信号量 ==================== */
extern QueueHandle_t     xDwinTxQueue;         /* 发送队列（应用 → TX 任务） */
extern QueueHandle_t     xDwinRxQueue;         /* 接收队列（ISR → RX 任务） */
extern SemaphoreHandle_t xDwinTxCompleteSem;   /* USART3 TX 完成信号量 */

/* ==================== 驱动 API ==================== */
void UART3_Send(const uint8_t *buf, uint16_t len);
void DwinRxByteHandler(uint8_t data);
void DwinRxReset(void);
void Dwin_InitQueues(void);

/* USER CODE END EFP */
```

---

### Task 2: dwin.c — UART3_Send + 状态机 + 队列初始化

**Files:**
- Modify: `Hostboard/Core/Src/dwin.c`

- [ ] **Step 1: 在文件开头的 Includes 区域添加头文件**

`dwin.c` 当前包含 `"dwin.h"` 和 `<string.h>`。确保 `"dwin.h"` 已包含，无需额外头文件。

- [ ] **Step 2: 在 `USER CODE BEGIN 0` 区块添加接收状态机和队列定义**

在 `dwin.c` 的 `USER CODE BEGIN 0` 区块添加（注意现有代码没有这个区块，需要在文件顶部添加，或确认文件结构）：

```c
/* USER CODE BEGIN 0 */

/* ==================== 队列与信号量定义 ==================== */
QueueHandle_t     xDwinTxQueue         = NULL;
QueueHandle_t     xDwinRxQueue         = NULL;
SemaphoreHandle_t xDwinTxCompleteSem   = NULL;

/* ==================== USART3 RX 状态机组装 ==================== */
#define DWIN_RX_BUF_SIZE  128

/* 状态机状态 */
#define DWIN_RX_WAIT_5A   0   /* 等待帧头 0x5A */
#define DWIN_RX_WAIT_A5   1   /* 已收 0x5A，等待 0xA5 */
#define DWIN_RX_WAIT_LEN  2   /* 已收帧头，等待 LEN 字节 */
#define DWIN_RX_DATA      3   /* 正在收集帧数据 */

static volatile uint8_t  dwin_rx_state;        /* 状态机当前状态 */
static volatile uint8_t  dwin_rx_buf[DWIN_RX_BUF_SIZE];  /* 接收缓冲区 */
static volatile uint16_t dwin_rx_idx;          /* 缓冲区当前写入位置 */
static volatile uint16_t dwin_rx_expected;     /* 预期总长度（含帧头+LEN） */

/* USER CODE END 0 */
```

- [ ] **Step 3: 添加 `Dwin_InitQueues()` 实现**

在 `DWIN_CalcCRC16` 函数之前或之后，放在 `USER CODE BEGIN 1` 区域（或合适的 USER CODE 区块）：

```c
/* ==================== 队列/信号量初始化 ==================== */

/**
 * @brief  初始化 USART3 迪文屏队列和信号量
 * @note   必须在创建 FreeRTOS 任务之前调用。
 */
void Dwin_InitQueues(void)
{
    if (xDwinTxQueue == NULL)
        xDwinTxQueue = xQueueCreate(DWIN_TX_QUEUE_LEN, sizeof(DwinFrame_t));

    if (xDwinRxQueue == NULL)
        xDwinRxQueue = xQueueCreate(DWIN_RX_QUEUE_LEN, sizeof(DwinFrame_t));

    if (xDwinTxCompleteSem == NULL)
        xDwinTxCompleteSem = xSemaphoreCreateBinary();
}
```

- [ ] **Step 4: 添加 `UART3_Send()` 实现**

```c
/**
 * @brief  将一帧数据送入 USART3 发送队列
 * @param  buf  帧数据指针（含 0x5A 0xA5 帧头）
 * @param  len  帧长度（字节数）
 *
 * 被 DWIN_WriteVar()、DWIN_ReadVar() 等所有帧构造函数调用。
 * 若队列满，阻塞等待直到有空位。
 */
void UART3_Send(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0 || len > DWIN_TX_FRAME_MAX)
        return;
    if (xDwinTxQueue == NULL)
        return;

    DwinFrame_t frame;
    frame.length = len;
    for (uint16_t i = 0; i < len; i++)
        frame.data[i] = buf[i];

    xQueueSend(xDwinTxQueue, &frame, portMAX_DELAY);
}
```

注意：用 `for` 循环而非 `memcpy`，避免潜在的对齐问题（`DwinFrame_t` 中 `data[]` 与 `buf` 来源不同）。若确认对齐一致可改用 `memcpy`。

- [ ] **Step 5: 添加 `DwinRxByteHandler()` 状态机实现**

```c
/**
 * @brief  USART3 字节接收中断处理 — 迪文屏帧状态机组装
 * @param  data  收到的字节
 *
 * 状态机：
 *   WAIT_5A → 收到 0x5A → WAIT_A5
 *   WAIT_A5 → 收到 0xA5 → WAIT_LEN；收到 0x5A → 重置
 *   WAIT_LEN → 校验 LEN，有效 → DATA，无效 → 重置
 *   DATA → 收满 LEN 字节 → 入 xDwinRxQueue → 重置
 *
 * 在 USART3_IRQHandler 的 RXNE 分支调用。
 */
void DwinRxByteHandler(uint8_t data)
{
    BaseType_t xWoken = pdFALSE;

    switch (dwin_rx_state)
    {
        case DWIN_RX_WAIT_5A:
            if (data == 0x5A)
            {
                dwin_rx_buf[0] = 0x5A;
                dwin_rx_idx = 1;
                dwin_rx_state = DWIN_RX_WAIT_A5;
            }
            break;

        case DWIN_RX_WAIT_A5:
            if (data == 0xA5)
            {
                dwin_rx_buf[1] = 0xA5;
                dwin_rx_idx = 2;
                dwin_rx_state = DWIN_RX_WAIT_LEN;
            }
            else if (data == 0x5A)
            {
                /* 连续 0x5A，重新开始 */
                dwin_rx_buf[0] = 0x5A;
                dwin_rx_idx = 1;
                /* state 保持 WAIT_A5 */
            }
            else
            {
                dwin_rx_state = DWIN_RX_WAIT_5A;
                dwin_rx_idx = 0;
            }
            break;

        case DWIN_RX_WAIT_LEN:
            dwin_rx_buf[2] = data;
            dwin_rx_idx = 3;
            if (data > 0 && data <= (DWIN_RX_BUF_SIZE - 3))
            {
                dwin_rx_expected = 3 + data;   /* 总帧长 = 帧头2 + LEN1 + LEN */
                dwin_rx_state = DWIN_RX_DATA;
            }
            else
            {
                /* 无效 LEN，重置 */
                dwin_rx_state = DWIN_RX_WAIT_5A;
                dwin_rx_idx = 0;
            }
            break;

        case DWIN_RX_DATA:
            if (dwin_rx_idx < DWIN_RX_BUF_SIZE)
                dwin_rx_buf[dwin_rx_idx] = data;
            dwin_rx_idx++;

            if (dwin_rx_idx >= dwin_rx_expected)
            {
                /* 完整帧收齐 → 入队列 */
                DwinFrame_t frame;
                frame.length = (dwin_rx_idx <= DWIN_RX_FRAME_MAX)
                               ? (uint16_t)dwin_rx_idx : DWIN_RX_FRAME_MAX;
                for (uint16_t i = 0; i < frame.length; i++)
                    frame.data[i] = dwin_rx_buf[i];

                if (xDwinRxQueue != NULL)
                    xQueueSendFromISR(xDwinRxQueue, &frame, &xWoken);

                /* 重置状态机 */
                dwin_rx_state = DWIN_RX_WAIT_5A;
                dwin_rx_idx = 0;
            }
            break;
    }

    portYIELD_FROM_ISR(xWoken);
}
```

- [ ] **Step 6: 添加 `DwinRxReset()` 实现**

```c
/**
 * @brief  重置 USART3 接收状态机
 * @note   在 ORE 溢出错误时调用，丢弃当前不完整帧。
 */
void DwinRxReset(void)
{
    dwin_rx_state = DWIN_RX_WAIT_5A;
    dwin_rx_idx = 0;
}
```

---

### Task 3: dwin_tasks.h — 任务声明（新文件）

**Files:**
- Create: `Hostboard/Core/Inc/dwin_tasks.h`

- [ ] **Step 1: 创建 dwin_tasks.h**

```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : dwin_tasks.h
  * @brief          : USART3 迪文屏发送/接收 FreeRTOS 任务声明
  *
  * TaskDwinTx： 从发送队列取帧 → HAL_UART_Transmit_IT → 等 TX 完成信号量
  * TaskDwinRx： 从接收队列取帧 → 将来解析帧内容
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __DWIN_TASKS_H__
#define __DWIN_TASKS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported functions prototypes ---------------------------------------------*/

void TaskDwinTx(void *arg);
void TaskDwinRx(void *arg);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __DWIN_TASKS_H__ */
```

---

### Task 4: dwin_tasks.c — 任务实现（新文件）

**Files:**
- Create: `Hostboard/Core/Src/dwin_tasks.c`

- [ ] **Step 1: 创建 dwin_tasks.c**

```c
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : dwin_tasks.c
  * @brief          : USART3 迪文屏发送/接收 FreeRTOS 任务实现
  *
  * 发送任务 (TaskDwinTx)：
  *   1. 阻塞等待 xDwinTxQueue
  *   2. HAL_UART_Transmit_IT 中断发送
  *   3. 等 xDwinTxCompleteSem（50ms 超时）
  *
  * 接收任务 (TaskDwinRx)：
  *   1. 阻塞等待 xDwinRxQueue（ISR 状态机填充）
  *   2. 将来：解析帧内容
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "dwin_tasks.h"
#include "dwin.h"
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==================== USART3 发送任务 ==================== */

void TaskDwinTx(void *arg)
{
    (void)arg;
    DwinFrame_t frame;

    for (;;)
    {
        /* ---- 1. 等待发送队列 ---- */
        if (xQueueReceive(xDwinTxQueue, &frame, portMAX_DELAY) != pdPASS)
            continue;

        /* ---- 2. 中断发送 ---- */
        HAL_UART_Transmit_IT(&huart3, frame.data, frame.length);

        /* ---- 3. 等待发送完成（50ms 超时） ---- */
        xSemaphoreTake(xDwinTxCompleteSem, pdMS_TO_TICKS(50));
    }
}

/* ==================== USART3 接收任务 ==================== */

void TaskDwinRx(void *arg)
{
    (void)arg;
    DwinFrame_t frame;

    for (;;)
    {
        /* ---- 1. 等待完整帧（ISR 状态机组装后入队） ---- */
        if (xQueueReceive(xDwinRxQueue, &frame, portMAX_DELAY) != pdPASS)
            continue;

        /* ---- 2. 将来：解析帧内容 ----
         * 根据 frame.data[3] (CMD) 分发：
         *   0x83 → 读应答（RTC 时间、传感器数据等）
         *   0x82 → 写应答确认
         */
    }
}

/* USER CODE END 0 */
```

---

### Task 5: stm32f4xx_it.c — USART3 IRQ 处理扩展

**Files:**
- Modify: `Hostboard/Core/Src/stm32f4xx_it.c`

- [ ] **Step 1: 在 Includes 区域添加 dwin.h 引用**

```c
/* USER CODE BEGIN Includes */
#include "uart1_modbus_master.h"
#include "dwin.h"       /* 新增 */
/* USER CODE END Includes */
```

- [ ] **Step 2: 扩展 USART3_IRQHandler**

将现有的 `USART3_IRQHandler` 函数体（USER CODE BEGIN USART3_IRQn 0 区块）替换为：

```c
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* ---- RXNE：收到一个字节，送入迪文屏状态机 ---- */
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
  {
      uint8_t data = (uint8_t)(huart3.Instance->DR & 0xFF);
      DwinRxByteHandler(data);
  }

  /* ---- ORE：溢出错误，清标志 + 重置 RX 状态机 ---- */
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE))
  {
      __HAL_UART_CLEAR_OREFLAG(&huart3);
      DwinRxReset();
  }

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}
```

---

### Task 6: modbus_callbacks.c — TX 完成回调扩展

**Files:**
- Modify: `Hostboard/Core/Src/modbus_callbacks.c`

- [ ] **Step 1: 添加 dwin.h 引用**

```c
/* USER CODE BEGIN 0 */
#include "dwin.h"   /* 新增：引用 xDwinTxCompleteSem */
/* USER CODE END 0 */
```

- [ ] **Step 2: 扩展 HAL_UART_TxCpltCallback 添加 huart3 分支**

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        BaseType_t xWoken = pdFALSE;
        xSemaphoreGiveFromISR(xMasterTxCompleteSem, &xWoken);
        portYIELD_FROM_ISR(xWoken);
    }
    /* ---- USART3 迪文屏 TX 完成 ---- */
    else if (huart == &huart3)
    {
        BaseType_t xWoken = pdFALSE;
        xSemaphoreGiveFromISR(xDwinTxCompleteSem, &xWoken);
        portYIELD_FROM_ISR(xWoken);
    }
}
```

---

### Task 7: main.c — 队列初始化与任务创建

**Files:**
- Modify: `Hostboard/Core/Src/main.c`

- [ ] **Step 1: 在 Includes 添加 dwin_tasks.h 引用**

```c
/* USER CODE BEGIN Includes */
#include "uart1_modbus_master.h"
#include "modbus_master_tasks.h"
#include "modbus_polling.h"
#include "dwin_tasks.h"       /* 新增 */
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */
```

- [ ] **Step 2: 在 MX_USART3_UART_Init() 之后、ModbusMaster_InitQueues() 之前或之后添加 Dwin_InitQueues()**

```c
  /* USER CODE BEGIN 2 */

  /* ---- 初始化 Modbus 主站队列和信号量 ---- */
  ModbusMaster_InitQueues();

  /* ---- 初始化 USART3 迪文屏队列和信号量 ---- */
  Dwin_InitQueues();

  /* ---- 创建 Modbus 发送任务 ---- */
  xTaskCreate(TaskModbusSend, "MstSend", 128, NULL, 2, NULL);
```

- [ ] **Step 3: 在最后一个 xTaskCreate 之后、vTaskStartScheduler 之前添加两个 DWIN 任务创建**

```c
  /* ---- 创建 USART3 迪文屏发送/接收任务 ---- */
  xTaskCreate(TaskDwinTx, "DwinTx", 128, NULL, 1, NULL);
  xTaskCreate(TaskDwinRx, "DwinRx", 128, NULL, 1, NULL);

  /* ---- 启动 FreeRTOS 调度器 ---- */
  vTaskStartScheduler();
```

---

### Task 8: CMakeLists.txt — 添加源文件

**Files:**
- Modify: `Hostboard/CMakeLists.txt`

- [ ] **Step 1: 在 target_sources 中添加 dwin_tasks.c**

在现有 `dwin.c` 条目之后添加：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dwin.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/dwin_tasks.c          # 新增
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/freertos_hooks.c
    ...
```

---

### Task 9: 编译验证

**Files:**
- None

- [ ] **Step 1: 编译 Hostboard**

```bash
cd /home/keake/Projects/WindPressureDetectionSystem/Hostboard \
  && cmake -B build/Debug \
  && cmake --build build/Debug 2>&1
```

预期输出：`0 errors, 0 warnings`

---

### 自检

**Spec 覆盖检查：**
- [x] 发送队列 `xDwinTxQueue` + TX 任务 → Task 1, 2, 4
- [x] 接收队列 `xDwinRxQueue` + RX 任务 → Task 1, 2, 4
- [x] ISR 状态机组装（0x5A 0xA5 帧头 + LEN 检测）→ Task 2 Step 5
- [x] `UART3_Send()` 入队 → Task 2 Step 4
- [x] TX 完成信号量 → Task 2 Step 2, Task 6
- [x] ORE 溢出处理 → Task 2 Step 6, Task 5
- [x] 两个 FreeRTOS 任务创建 → Task 4, Task 7
- [x] USART3_IRQHandler 扩展 → Task 5
- [x] CMakeLists.txt 更新 → Task 8
