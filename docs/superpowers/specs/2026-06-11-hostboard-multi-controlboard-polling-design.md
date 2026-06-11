# Hostboard 多 Controlboard 轮询设计

> **设计目标**：Hostboard 通过 PowerBus 二总线轮询最多 128 个 Controlboard（地址 1-128），读取离散输入寄存器，检测零地址和地址重复。

---

## 1. 物理拓扑

```
Hostboard USART1 (PA9/PA10)
  │  Modbus RTU 主站 @ 9600 8N1
  │
  └── PowerBus 二总线
        ├── Controlboard #1  (DIP 地址 1)
        ├── Controlboard #2  (DIP 地址 2)
        ├── ...
        └── Controlboard #128 (DIP 地址 128)

轮询地址范围：0-128（共 129 个地址）
  地址 0：用于检测是否有 Controlboard 误设为 0 地址
  地址 1-128：正常 Controlboard 地址
```

## 2. 系统架构

```
轮询任务 (TaskModbusPoll)     每 50ms 轮询一个地址
  │
  │  EnqueueRequest(FC 0x02, addr=n, bit_start=0, bit_count=130)
  │
  ├──→ [xMasterSendQueue]
  │       │
  │       TaskModbusSend ──UART1 TX──→ PowerBus
  │         ├─ HAL_UART_Transmit_IT()
  │         ├─ 等 TX 完成信号量
  │         ├─ 等响应信号量 (30ms 超时)
  │         └─ 帧间隔 4ms
  │
  │      USART1 ISR (RXNE+IDLE+ORE)
  │       └─ [xMasterRawRxQueue]
  │             │
  │             TaskModbusRecv
  │               ├─ CRC 校验
  │               ├─ CRC 通过 + addr=0     → HostReg_RecordZeroAddrResponse()
  │               ├─ CRC 通过 + addr=1-128 → HostReg_StoreCoilData()
  │               ├─ CRC 失败 + 有数据     → HostReg_RecordError()
  │               └─ xSemaphoreGive(xMasterRxSem)  (仅 CRC 通过时)
  │
  └── hostboard_registers.c  ← 存储/查询 128 路线圈数据 + 检测标志
```

## 3. 模块划分

### 3.1 新增：`hostboard_registers.c/h`

独立寄存器模块，管理与 Controlboard 通信相关的数据存储和状态检测。

**存储结构**：

```c
#define MAX_CTRLBD_ADDR     128
#define COIL_BYTE_COUNT     17      /* 130 bits = 16.25 → 17 字节 */

static uint8_t  board_coil_data[MAX_CTRLBD_ADDR + 1][COIL_BYTE_COUNT];
/* board_coil_data[0] 始终为全零，地址 0 响应不计入数组 */
/* board_coil_data[1..128] 对应 Controlboard 地址 1-128 */
```

**17 字节数据布局**（与 Controlboard 线圈寄存器映射一致）：

| 字节偏移 | bit 范围  | 内容                           |
|---------|----------|-------------------------------|
| 0       | 0-7      | 传感器 #1-8 在线状态            |
| 1       | 8-15     | 传感器 #9-16 在线状态           |
| 2-6     | 16-55    | 传感器 #17-56 在线状态          |
| 7       | 56-63    | bit 56-62: 传感器 #57-63 在线<br>bit 63: 传感器 #1 报警标志 |
| 8-15    | 64-127   | bit 64-125: 传感器 #2-63 报警标志<br>bit 126: 零地址传感器存在<br>bit 127: 地址重复标志 |
| 16      | 128-129  | bit 128: 全局报警<br>bit 129: 烟雾报警器状态 |

**API**：

```c
/* ===== 数据存储与读取 ===== */
void    HostReg_StoreCoilData(uint8_t addr, const uint8_t *data, uint8_t len);
uint8_t HostReg_GetCoilByte(uint8_t addr, uint8_t byte_idx);
uint8_t HostReg_GetCoilBit(uint8_t addr, uint16_t bit_idx);

/* ===== 零地址检测 ===== */
void    HostReg_RecordZeroAddrResponse(void);
uint8_t HostReg_GetZeroAddrPresent(void);

/* ===== 地址重复检测 ===== */
void    HostReg_RecordError(void);
void    HostReg_StepCycle(void);
uint8_t HostReg_GetAddrConflict(void);
```

**零地址检测逻辑**：

- `HostReg_RecordZeroAddrResponse()` — 收到地址 0 的 CRC 有效帧时调用，标记本轮发现零地址
- `HostReg_StepCycle()` — 每轮完整扫描（0→128→回绕）结束时调用：
  - 将 `zero_addr_found_this_cycle` 移入 `zero_addr_history[3]` 环形缓冲
  - 清零当前周期标志
  - `zero_addr_present = history[0] || history[1] || history[2]`

**重复地址检测逻辑**：

- `HostReg_RecordError()` — 接收任务在以下情况调用：
  - CRC 校验失败且 `raw.length > 0`（收到了数据但校验失败）
  - 帧过短（`raw.length < 4`，收到了垃圾数据）
- `HostReg_StepCycle()` — 每轮完整扫描结束时：
  - `crc_error_history[current_cycle] = crc_error_count`
  - `crc_error_count = 0`
  - 累计三轮错误计数 `sum = history[0] + history[1] + history[2]`
  - `addr_conflict_flag = (sum > 2) ? 1 : 0`

**内部状态变量**：

```c
static uint8_t  zero_addr_found_this_cycle;  /* 本轮是否收到 addr0 有效帧 */
static uint8_t  zero_addr_history[3];          /* 三轮历史 */
static uint8_t  zero_addr_present;             /* 综合标志 */

static uint32_t crc_error_count;               /* 本轮累计错误 */
static uint32_t crc_error_history[3];          /* 三轮历史 */
static uint8_t  addr_conflict_flag;             /* 综合标志 */

static uint8_t  current_cycle;                 /* 0,1,2 循环 */
```

### 3.2 修改：`modbus_polling.c/h`

**重写 `modbus_polling.h`**：移除单地址宏，`MAX_CTRLBD_ADDR` 定义在 `hostboard_registers.h` 中，本文件通过包含该头文件使用。

**重写 `modbus_polling.c`**：轮询任务改为 50ms 间隔的顺序地址扫描。

```c
void TaskModbusPoll(void *arg)
{
    uint16_t current_addr = 0;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(50));

        ModbusMasterRequest_t req = {
            .slave_addr = (uint8_t)current_addr,
            .func_code  = MODBUS_FUNC_READ_DISCRETE_INPUTS,
            .reg_addr   = 0,
            .reg_value  = 130
        };
        ModbusMaster_EnqueueRequest(&req);

        current_addr++;
        if (current_addr > MAX_CTRLBD_ADDR)
        {
            current_addr = 0;
            HostReg_StepCycle();
        }
    }
}
```

时序总结：

| 参数 | 值 |
|------|-----|
| 轮询间隔 | 50ms（连续两个地址之间） |
| 完整一轮时间 | 129 × 50ms = **6.45s** |
| 零地址/重复地址更新周期 | 每 6.45s 一次（每轮 StepCycle） |
| 重复地址判定窗口 | 3 轮 ≈ **19.35s** |

### 3.3 修改：`modbus_master_tasks.c`

**修改接收任务 `TaskModbusRecv`**：

```c
void TaskModbusRecv(void *arg)
{
    (void)arg;
    ModbusMasterFrame_t raw;

    for (;;)
    {
        if (ModbusMaster_DequeueRawFrame(&raw, portMAX_DELAY) != 0)
            continue;

        /* CRC 校验 */
        uint8_t crc_ok = 0;
        if (raw.length >= 4)
        {
            uint16_t calc_crc = ModbusMaster_CRC16(raw.frame, raw.length - 2);
            uint16_t recv_crc = (uint16_t)raw.frame[raw.length - 2]
                              | ((uint16_t)raw.frame[raw.length - 1] << 8);
            crc_ok = (calc_crc == recv_crc);
        }

        if (crc_ok)
        {
            uint8_t slave_addr = raw.frame[0];
            uint8_t func_code  = raw.frame[1];
            uint8_t byte_cnt   = raw.frame[2];

            if (slave_addr == 0)
            {
                /* 地址 0：记录零地址，不存数组 */
                HostReg_RecordZeroAddrResponse();
            }
            else if (slave_addr >= 1 && slave_addr <= MAX_CTRLBD_ADDR
                     && func_code == 0x02
                     && byte_cnt > 0
                     && byte_cnt <= COIL_BYTE_COUNT
                     && (3 + byte_cnt + 2) <= raw.length)
            {
                /* 有效 FC 0x02 响应 → 存入寄存器模块 */
                HostReg_StoreCoilData(slave_addr, &raw.frame[3], byte_cnt);
            }
            /* else: 功能码异常或格式错误 → 忽略不计 */

            xSemaphoreGive(xMasterRxSem);
        }
        else
        {
            /* CRC 失败且收到过数据 → 非超时错误 */
            if (raw.length > 0)
                HostReg_RecordError();
        }
    }
}
```

### 3.4 已有模块保持不变

| 模块 | 原因 |
|------|------|
| `uart1_modbus_master.c/h` | 缓冲区足够（64 字节 ≥ 22 字节响应），队列/信号量机制不变 |
| `modbus_master_tasks.c` 发送任务 | 不需修改，TX 信号量/响应信号量机制不变 |
| `modbus_callbacks.c` | TX 完成回调不变 |
| `usart.c` / `stm32f4xx_it.c` | ISR 处理不变 |

## 4. 接收任务异常处理总结

| 场景 | 行为 |
|------|------|
| CRC 通过 + 地址 0 | `HostReg_RecordZeroAddrResponse()` → 信号量 Give |
| CRC 通过 + 地址 1-128 + FC 0x02 | `HostReg_StoreCoilData()` → 信号量 Give |
| CRC 通过 + 地址 1-128 + 非 FC 0x02 | 仅信号量 Give，忽略 |
| CRC 失败 + `raw.length > 0` | `HostReg_RecordError()` |
| 超时（无数据） | 发任务超时继续，不计错误 |

## 5. 分阶段实施计划

### 阶段 A：最小可行（在当前会话实施）

1. 新建 `hostboard_registers.c/h` — 数据存储、API、零地址和重复地址检测
2. 重写 `modbus_polling.c/h` — 地址 0-128 循环、50ms 间隔
3. 修改 `modbus_master_tasks.c` — 接收任务数据存储 + 错误统计
4. CMakeLists.txt 添加新源文件
5. 编译验证

### 阶段 B：未来扩展（不在本设计范围内）

- 读保持寄存器（传感器类型 + 数据）：新增 HostReg API 扩展入口
- 根据 HostReg 数据驱动本地显示：通过 SPI/LCD 接口展示各 Controlboard 状态
- 应用层报警联动：解析线圈位后触发本地报警动作

---

## 设计检查清单

- [x] 轮询地址：0-128（129 个地址）
- [x] 功能码：FC 0x02，读 130 bits
- [x] 轮询间隔：50ms（固定）
- [x] 数据存储：128 路 × 17 字节
- [x] 零地址检测：CRC 通过 + 地址 0 → 记录，3 轮 OR
- [x] 重复地址检测：三轮 CRC 错误累计 > 2 → 标记
- [x] 非超时错误定义：CRC 失败 + 收到数据
- [x] 功能码异常：忽略不计
- [x] 距离 3.5 字符帧间隔：由发送任务保证（4ms）
- [x] 缓冲区大小：现有 64 字节 ≥ 22 字节响应
- [x] API 设计：与 Controlboard 对称，易于理解
