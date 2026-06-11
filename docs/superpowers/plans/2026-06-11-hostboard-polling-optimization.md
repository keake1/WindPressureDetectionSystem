# Hostboard 精简轮询实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 Hostboard 轮询从每地址 130 bits 改为仅读 3 bits (126-128)，每 8 次轮询穿插 1 次在线控制器轮询。

**Architecture:** 发送任务发帧前将请求参数存入全局变量 `g_host_last_req`，接收任务据此将响应 1 字节写入 `coil_data` 的正确偏移。轮询任务 8+1 穿插：8 次正常地址 + 1 次在线控制器。

**Tech Stack:** C (STM32F407, FreeRTOS), Modbus RTU 主站

---

### Task 1: uart1_modbus_master.h — 添加全局请求记录类型

**Files:**
- Modify: `Hostboard/Core/Inc/uart1_modbus_master.h`

- [ ] **Step 1: 添加 `HostboardLastReq_t` 类型定义**

在 `uart1_modbus_master.h` 的 `USER CODE BEGIN ET` 区块添加：

```c
/* USER CODE BEGIN ET */

/**
 * @brief  最近一次发送的 Modbus 请求信息
 * @note   发送任务在发帧前写入，接收任务据此解析响应数据的位置。
 *         同一时刻只有一帧在飞行，不存在竞态。
 */
typedef struct {
    uint8_t  slave_addr;   /* 请求的从机地址 */
    uint16_t reg_addr;     /* 寄存器起始地址（位偏移） */
    uint16_t bit_count;    /* 请求的位数 */
} HostboardLastReq_t;

/* USER CODE END ET */
```

- [ ] **Step 2: 添加 `g_host_last_req` 外部声明**

在现有的 `extern` 队列/信号量声明后面（约第 80 行）添加：

```c
extern HostboardLastReq_t g_host_last_req;
```

放在 `extern SemaphoreHandle_t xMasterTxCompleteSem;` 之后。

---

### Task 2: hostboard_registers.h/.c — 添加局部位写入接口

**Files:**
- Modify: `Hostboard/Core/Inc/hostboard_registers.h`
- Modify: `Hostboard/Core/Src/hostboard_registers.c`

- [ ] **Step 1: `hostboard_registers.h` — 添加声明**

在 `HostReg_GetAddrConflict()` 后或 `HostReg_Online` 组中添加：

```c
/** @defgroup HostReg_Partial 局部位写入（精简轮询） */
/** @{ */
void HostReg_StorePartialBits(uint8_t addr, uint16_t reg_addr,
                              uint16_t bit_count, const uint8_t *data);
/** @} */
```

- [ ] **Step 2: `hostboard_registers.c` — 添加 `HostReg_StorePartialBits()` 实现**

在 `HostReg_IsOnline()` 之后的合适位置添加：

```c
/* ==================== 局部位写入（精简轮询） ==================== */

/**
 * @brief  将响应数据按位写入 coil_data 的指定偏移
 * @param  addr       Controlboard 地址 (1-128)
 * @param  reg_addr   位起始地址（如 126）
 * @param  bit_count  位数（如 3）
 * @param  data       响应数据指针（大端序填充）
 *
 * 用于精简轮询场景：只读取 3 bits (126-128)，收到 1 字节后
 * 按位复制到 ctrl_boards[addr].coil_data 的正确位置。
 */
void HostReg_StorePartialBits(uint8_t addr, uint16_t reg_addr,
                              uint16_t bit_count, const uint8_t *data)
{
    if (addr < 1 || addr > MAX_CTRLBD_ADDR)
        return;

    for (uint16_t i = 0; i < bit_count; i++)
    {
        uint8_t  bit_val  = (data[i / 8] >> (i % 8)) & 1;
        uint16_t dst_bit  = reg_addr + i;
        uint8_t  byte_off = dst_bit / 8;
        uint8_t  bit_off  = dst_bit % 8;

        if (byte_off >= COIL_BYTE_COUNT)
            break;

        ctrl_boards[addr].coil_data[byte_off] &= ~(1 << bit_off);
        ctrl_boards[addr].coil_data[byte_off] |= (bit_val << bit_off);
    }

    /* 收到有效响应 → 更新在线标志 */
    ctrl_boards[addr].online = 1;
    ctrl_boards[addr].last_seen_cycle = current_cycle;
}
```

---

### Task 3: modbus_master_tasks.c — 发送任务记录 + 接收任务解析

**Files:**
- Modify: `Hostboard/Core/Src/modbus_master_tasks.c`

- [ ] **Step 1: 发送任务 — 在发帧前记录 `g_host_last_req`**

在 `TaskModbusSend` 中，构造帧之前或之后、`HAL_UART_Transmit_IT` 之前添加：

```c
        /* ---- 2.5 记录当前请求信息（供接收任务解析响应） ---- */
        g_host_last_req.slave_addr = req.slave_addr;
        g_host_last_req.reg_addr   = req.reg_addr;
        g_host_last_req.bit_count  = req.reg_value;
```

放在 `ModbusMaster_StartRx()` 和 `HAL_UART_Transmit_IT` 之间，例如在清除信号量之后：

```c
        /* ---- 2. 开启接收中断窗口 ---- */
        ModbusMaster_StartRx();

        /* 清除可能残留的信号量（上次超时后接收任务可能仍会 Give） */
        xSemaphoreTake(xMasterRxSem, 0);

        /* 记录当前请求信息（供接收任务解析响应数据的位置） */
        g_host_last_req.slave_addr = req.slave_addr;
        g_host_last_req.reg_addr   = req.reg_addr;
        g_host_last_req.bit_count  = req.reg_value;

        /* ---- 3. 构造 Modbus RTU 帧 (8 字节) ---- */
        tx_buf[0] = req.slave_addr;
        ...
```

- [ ] **Step 2: 接收任务 — 根据 `g_host_last_req` 分支处理**

在 `TaskModbusRecv` 中，将原有的 `HostReg_StoreCoilData` 调用路径改为分支：

对于 CRC 通过的响应（地址 1-128，FC 0x02），判断响应长度：

```c
            /* 地址 1-128：验证是 FC 0x02 响应后解析 */
            else if (slave_addr >= 1 && slave_addr <= MAX_CTRLBD_ADDR
                     && func_code == 0x02
                     && byte_cnt > 0
                     && (3 + byte_cnt + 2) <= raw.length)
            {
                if (byte_cnt >= COIL_BYTE_COUNT)
                {
                    /* 全量 17 字节响应 → 完整覆盖 */
                    HostReg_StoreCoilData(slave_addr, &raw.frame[3], byte_cnt);
                }
                else if (byte_cnt == 1 && slave_addr == g_host_last_req.slave_addr)
                {
                    /* 精简 1 字节响应 → 按位写入指定偏移 */
                    HostReg_StorePartialBits(slave_addr,
                                             g_host_last_req.reg_addr,
                                             g_host_last_req.bit_count,
                                             &raw.frame[3]);
                }
                /* else: 其他长度 → 忽略（不应发生） */
            }
```

---

### Task 4: modbus_polling.c — 8+1 穿插轮询

**Files:**
- Modify: `Hostboard/Core/Src/modbus_polling.c`

- [ ] **Step 1: 重写轮询任务 — 8 正常 + 1 在线穿插**

将 `TaskModbusPoll` 整体替换为：

```c
void TaskModbusPoll(void *arg)
{
    (void)arg;
    uint16_t current_addr = 0;       /* 正常轮询地址 0-128 */
    uint8_t  poll_count    = 0;       /* 正常轮询计数（0-7） */
    uint16_t online_idx    = 1;       /* 在线控制器扫描起始 */
    uint8_t  do_online     = 0;       /* 本次是否穿插在线 */

    for (;;)
    {
        /* ---- 轮询间隔 50ms ---- */
        vTaskDelay(pdMS_TO_TICKS(50));

        /* ---- 构造 FC 0x02 请求：读 3 bits (126-128) ---- */
        ModbusMasterRequest_t req;
        req.func_code  = MODBUS_FUNC_READ_DISCRETE_INPUTS;
        req.reg_addr   = 126;
        req.reg_value  = 3;

        /* 每 8 次正常轮询穿插 1 次在线控制器 */
        if (poll_count >= 8)
        {
            poll_count = 0;
            do_online = 1;
        }

        if (do_online)
        {
            /* ---- 在线穿插：找下一个在线控制器 ---- */
            uint8_t found = 0;
            for (uint16_t i = 0; i < MAX_CTRLBD_ADDR; i++)
            {
                uint16_t addr = online_idx + i;
                if (addr > MAX_CTRLBD_ADDR)
                    addr -= MAX_CTRLBD_ADDR;
                if (HostReg_IsOnline((uint8_t)addr))
                {
                    req.slave_addr = (uint8_t)addr;
                    online_idx = (uint16_t)(addr + 1);
                    if (online_idx > MAX_CTRLBD_ADDR)
                        online_idx = 1;
                    found = 1;
                    break;
                }
            }

            if (found)
            {
                ModbusMaster_EnqueueRequest(&req);
                do_online = 0;
                continue;   /* 跳过正常轮询，下次继续正常地址 */
            }
            else
            {
                /* 无在线控制器 → 跳过穿插，执行正常轮询 */
                do_online = 0;
            }
        }

        /* ---- 正常地址轮询 ---- */
        req.slave_addr = (uint8_t)current_addr;
        ModbusMaster_EnqueueRequest(&req);

        poll_count++;
        current_addr++;
        if (current_addr > MAX_CTRLBD_ADDR)
        {
            current_addr = 0;
            HostReg_StepCycle();
            poll_count = 0;     /* 新一轮开始 */
        }
    }
}
```

---

### Task 5: 编译验证

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
- [x] 全局变量 `g_host_last_req` 记录请求信息 → Task 1
- [x] 所有轮询读 3 bits (126-128) → Task 4 (reg_addr=126, reg_value=3)
- [x] 接收任务根据 `g_host_last_req` 写入正确偏移 → Task 3 Step 2 + Task 2
- [x] 每 8 次正常穿插 1 次在线控制器 → Task 4 (poll_count threshold + online scan)
- [x] 在线控制器动态选择 → Task 4 (HostReg_IsOnline loop)
- [x] 地址回绕 StepCycle → Task 4 (current_addr wrap)
