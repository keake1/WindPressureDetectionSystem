# Hostboard USART3 迪文屏收发驱动设计

> **状态：** 已批准设计
> **日期：** 2026-06-11

## 1. 背景

Hostboard（STM32F407VET6）需要连接一块迪文屏，使用 USART3（PB10 TX / PB11 RX），波特率 115200 8N1。

与 Controlboard 的迪文屏只发送不同，Hostboard 的迪文屏需要**双向收发**：
- **发送**：写变量指令（0x82）、读变量指令（0x83）、NorFlash 操作等
- **接收**：迪文屏应答帧（如 0x83 读回数据、RTC 时间等）

## 2. 架构

```
应用层 (DWIN_WriteVar/DWIN_ReadVar/DWIN_NorFlashWrite/...)
    │
    ├── UART3_Send(buf, len) → [xDwinTxQueue] → TaskDwinTx → HAL_UART_Transmit_IT → USART3 TX
    │                                                  ↑              ↓
    │                                          xDwinTxCompleteSem ← HAL_UART_TxCpltCallback
    │
    └── USART3 RXNE ISR → DwinRxByteHandler (状态机组装) → [xDwinRxQueue] → TaskDwinRx → (将来解析)
```

### 2.1 任务

| 任务 | 函数 | 优先级 | 栈(words) | 职责 |
|------|------|--------|-----------|------|
| DwinTx | `TaskDwinTx` | 1 | 128 | 阻塞等 `xDwinTxQueue` → `HAL_UART_Transmit_IT` → 等 `xDwinTxCompleteSem` (50ms) |
| DwinRx | `TaskDwinRx` | 1 | 128 | 阻塞等 `xDwinRxQueue` → 将来解析帧内容 |

### 2.2 队列与信号量

| 名称 | 类型 | 长度 | 项大小 | 用途 |
|------|------|------|--------|------|
| `xDwinTxQueue` | Queue | 8 | `DwinFrame_t` | 发送帧队列（应用 → TX 任务） |
| `xDwinRxQueue` | Queue | 4 | `DwinFrame_t` | 接收帧队列（ISR → RX 任务） |
| `xDwinTxCompleteSem` | Binary Semaphore | — | — | USART3 TX 完成通知 |

### 2.3 DwinFrame_t

```c
#define DWIN_TX_FRAME_MAX   128
#define DWIN_RX_FRAME_MAX   128

typedef struct {
    uint8_t  data[128];   /* 帧数据（含帧头 0x5A 0xA5 等） */
    uint16_t length;       /* 帧长度（字节数） */
} DwinFrame_t;
```

## 3. 发送路径

`UART3_Send()` 是统一的发送入口，被 `DWIN_WriteVar()`、`DWIN_ReadVar()` 等所有帧构造函数调用。

```
应用调用 UART3_Send(buf, len)
  └── 参数校验
  └── 复制到 DwinFrame_t
  └── xQueueSend(xDwinTxQueue, &frame, portMAX_DELAY)
         ↓
  TaskDwinTx 出队
  └── HAL_UART_Transmit_IT(&huart3, frame.data, frame.length)
  └── xSemaphoreTake(xDwinTxCompleteSem, pdMS_TO_TICKS(50))   ← 任务阻塞等待发送完成
         ↓
  HAL_UART_TxCpltCallback(&huart3)
  └── xSemaphoreGiveFromISR(xDwinTxCompleteSem)                ← 唤醒 TX 任务
```

TX 任务优先级 1，50ms 超时防止死等。若超时，丢弃当前帧处理下一帧。

## 4. 接收路径（ISR 状态机组装）

USART3 RXNE 中断触发 `DwinRxByteHandler(data)`，在 ISR 中用有限状态机按迪文屏协议组装完整帧。

### 4.1 迪文屏应答帧格式

```
5A A5 [LEN] [CMD] [AH] [AL] [DATA...]
  帧头     长度   指令  地址  数据
```
- `LEN`：从 CMD 起算的后续字节数（含 CMD + ADDR + DATA）
- 完整帧总长度 = 2 (帧头) + 1 (LEN) + LEN
- 所有应答均以 `0x5A 0xA5` 作为帧头

### 4.2 状态机

```
WAIT_5A ──收到0x5A──→ WAIT_A5 ──收到0xA5──→ WAIT_LEN ──收到LEN──→ DATA
  ↑                      ↑                    无效LEN↓              │
  └──────────────────────┴─────────────────────────← 重置           │
                                                                    │
                                              收满(LEN)字节──────→→ 入 xDwinRxQueue → 重置
```

**状态说明：**

| 状态 | 含义 | 进入条件 | 动作 |
|------|------|---------|------|
| `WAIT_5A` | 等待帧起始 | 复位/帧结束 | 字节 == 0x5A → 存 buffer，切 WAIT_A5 |
| `WAIT_A5` | 等待帧头第二字节 | 已收 0x5A | 0xA5 → 存 buffer，切 WAIT_LEN；0x5A → 重头开始；其他 → 丢弃，重置 |
| `WAIT_LEN` | 等待 LEN 字节 | 收到完整帧头 | 存 buffer，校验 LEN（>0 且 ≤125，保证总帧长 ≤ 128），切 DATA；无效 → 重置 |
| `DATA` | 收集帧数据 | 已获 LEN | 每字节入 buffer，计数器 +1；达到预期长度 → `xQueueSendFromISR` → 重置 |

### 4.3 ISR 处理

```c
void USART3_IRQHandler(void)
{
    /* USER CODE BEGIN USART3_IRQn 0 */
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
    {
        uint8_t data = (uint8_t)(huart3.Instance->DR & 0xFF);
        DwinRxByteHandler(data);
    }
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE))
    {
        __HAL_UART_CLEAR_OREFLAG(&huart3);
        DwinRxReset();   /* ORE 时重置状态机 */
    }
    /* USER CODE END USART3_IRQn 0 */
    HAL_UART_IRQHandler(&huart3);
}
```

### 4.4 异常处理

- **LEN 无效或超长**：重置状态机，丢弃当前帧
- **ORE 溢出**：清 ORE 标志 + 重置状态机
- **帧中数据含 0x5A**：在 DATA 状态下不检测帧头，依 LEN 精确计数，不会误判
- **总线噪声**：状态机在收到下一个合法 `0x5A 0xA5` 时自动重新同步

### 4.5 RX 任务

`TaskDwinRx` 当前只阻塞接收完整帧，不解析。将来扩展解析逻辑：

```c
void TaskDwinRx(void *arg)
{
    DwinFrame_t frame;
    for (;;)
    {
        xQueueReceive(xDwinRxQueue, &frame, portMAX_DELAY);
        /* 将来解析：根据 frame.data[3] (CMD) 分发
         *  0x83 → 读应答（RTC 时间、传感器数据等）
         *  0x82 → 写应答确认
         */
    }
}
```

## 5. 文件变更清单

| 文件 | 操作 | 内容 |
|------|------|------|
| `Core/Inc/dwin_tasks.h` | **新建** | `TaskDwinTx` / `TaskDwinRx` 声明 |
| `Core/Src/dwin_tasks.c` | **新建** | 两个任务实现 |
| `Core/Inc/dwin.h` | **修改** | 添加 `DwinFrame_t` 类型、队列/信号量 extern、`UART3_Send()` / `DwinRxByteHandler()` / `DwinRxReset()` / `Dwin_InitQueues()` 声明；添加 FreeRTOS 头文件引用 |
| `Core/Src/dwin.c` | **修改** | 添加 `UART3_Send()`、`DwinRxByteHandler()` 状态机、`DwinRxReset()`、`Dwin_InitQueues()`、静态接收缓冲区与状态变量 |
| `Core/Src/stm32f4xx_it.c` | **修改** | `USART3_IRQHandler` 添加 RXNE 取数 + ORE 清除 + 调用 `DwinRxReset()` |
| `Core/Src/modbus_callbacks.c` | **修改** | `HAL_UART_TxCpltCallback` 添加 `huart3` 分支 Give `xDwinTxCompleteSem` |
| `Core/Src/main.c` | **修改** | 添加 `Dwin_InitQueues()` + 两个 `xTaskCreate` |
| `CMakeLists.txt` | **修改** | 添加 `Core/Src/dwin_tasks.c` |

## 6. 边界条件与约束

- **发送队列满**：`UART3_Send` 使用 `portMAX_DELAY` 阻塞等待，调用方任务会等待队列有空位
- **TX 完成超时**：50ms，@115200 最长帧 ~128 字节 ≈ 11ms，超时充裕。超时后 TX 任务继续处理下一帧
- **RX 缓冲区**：128 字节，单帧最大支持 LEN=125（总帧长 128），超长帧被丢弃
- **全双工**：TX 与 RX 完全独立并行，无需握手或信号量协调
