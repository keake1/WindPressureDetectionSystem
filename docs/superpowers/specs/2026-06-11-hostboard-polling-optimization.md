# Hostboard 精简轮询设计 — 3-bit 读取 + 在线控制器穿插

> **设计目标**：将每个 Controlboard 的 FC 0x02 轮询从全量 130 bits 改为仅读取 126-128 三位（1 字节响应），并通过全局变量记录请求信息让接收任务正确解析。同时增加在线控制器穿插轮询机制，提高在线控制器的更新频率。
>
> **关联设计**：`2026-06-11-hostboard-multi-controlboard-polling-design.md`、`2026-06-11-hostboard-controller-info-struct.md`

---

## 1. 全局变量：记录当前请求信息

发送任务在发送 Modbus 帧前，将当前请求的参数存入全局变量，接收任务据此解析响应数据的位置。

```c
/* Hostboard/Core/Inc/hostboard_polling.h 或 uart1_modbus_master.h */

typedef struct {
    uint8_t  slave_addr;   /* 请求的从机地址 */
    uint16_t reg_addr;     /* 寄存器起始地址（位偏移） */
    uint16_t bit_count;    /* 请求的位数 */
} HostboardLastReq_t;

extern HostboardLastReq_t g_host_last_req;
```

### 生命周期

```
发送任务 (TaskModbusSend):
  1. 从 xMasterSendQueue 取请求
  2. g_host_last_req = { req.slave_addr, req.reg_addr, req.reg_value }
  3. 构造帧 → 发帧 → 等响应

接收任务 (TaskModbusRecv):
  1. CRC 校验
  2. 从 g_host_last_req 读取 reg_addr / bit_count
  3. 根据位偏移将响应数据写入 ctrl_boards[addr].coil_data
  4. xSemaphoreGive(xMasterRxSem)

发送任务:
  5. 收到信号量 → 继续下一轮
```

由于 Modbus 主站同一时刻仅有一个请求在飞行，`g_host_last_req` 不存在竞态。

---

## 2. 精简读取：仅读取位 126-128

### 为什么读这三位？

| 位 | 含义 | 作用 |
|----|------|------|
| 126 | 零地址传感器存在 | 检测是否有 Controlboard 误设为 0 地址 |
| 127 | 地址重复标志 | 检测总线上地址冲突 |
| 128 | 全局报警 | 是否有传感器报警 |

这三位反映了每个 Controlboard 的核心系统状态。在线/离线状态由 `HostReg_StepCycle()` 独立检测（基于是否收到 CRC 正确的响应），不依赖线圈数据。

### Modbus 帧

**请求（8 字节）：**
```
[addr] [0x02] [0x00] [0x7E] [0x00] [0x03] [CRC_lo] [CRC_hi]
          ^      ^      ^      ^      ^
       FC 0x02  reg_hi=0  reg_lo=126  count_hi=0  count_lo=3
```

**响应（6 字节）：**
```
[addr] [0x02] [0x01] [data] [CRC_lo] [CRC_hi]
                   ^      ^
               byte_cnt=1  data: 位 126-128 在低 3 位
```

### 响应数据写入逻辑

收到 1 字节 `data`，需写入 `coil_data` 的正确位置：

```c
/* 根据 g_host_last_req.reg_addr = 126: */
uint8_t byte_offset = 126 / 8;     /* = 15 */
uint8_t bit_offset  = 126 % 8;     /* = 6 */

/* 位 126-127 → coil_data[15] 的位 6-7 */
ctrl_boards[addr].coil_data[15] &= ~(0xC0);                  /* 清除位 6-7 */
ctrl_boards[addr].coil_data[15] |= (data & 0x03) << 6;       /* 写入位 126-127 */

/* 位 128 → coil_data[16] 的位 0 */
ctrl_boards[addr].coil_data[16] &= ~(0x01);                  /* 清除位 0 */
ctrl_boards[addr].coil_data[16] |= (data & 0x04) >> 2;       /* 写入位 128 */
```

通用化实现：给定 `reg_addr` 和 `bit_count`，逐位复制到 `coil_data` 的正确偏移：

```c
void HostReg_StorePartialBits(uint8_t addr, uint16_t reg_addr,
                              uint16_t bit_count, const uint8_t *data)
{
    for (uint16_t i = 0; i < bit_count; i++)
    {
        uint8_t bit_val = (data[i / 8] >> (i % 8)) & 1;
        uint16_t dst_bit = reg_addr + i;
        uint8_t *dst_byte = &ctrl_boards[addr].coil_data[dst_bit / 8];
        *dst_byte &= ~(1 << (dst_bit % 8));
        *dst_byte |= (bit_val << (dst_bit % 8));
    }
    ctrl_boards[addr].online = 1;
    ctrl_boards[addr].last_seen_cycle = current_cycle;
}
```

---

## 3. 轮询策略：8+1 穿插

### 序列定义

```
正常地址轮询：每 50ms 读 3 bits @ 126-128
在线控制器穿插：每 8 次正常轮询后插入 1 次在线控制器轮询（同样读 3 bits @ 126-128）

第 1 次: addr=0   (正常)
第 2 次: addr=1   (正常)
...
第 8 次: addr=7   (正常)
第 9 次: online_A (在线穿插)
第10次: addr=8   (正常)
...
第17次: addr=15  (正常)
第18次: online_B (在线穿插)
...
```

### 在线控制器选择

轮询任务维护一个 `online_poll_idx`（1-128），每次需要在线穿插时：

1. 从 `online_poll_idx` 开始向后扫描，找到第一个 `HostReg_IsOnline() == 1` 的地址
2. 没找到则从 1 重新开始扫描
3. 更新 `online_poll_idx` 到最后扫描位置 + 1

如果总线上没有任何在线控制器，则跳过本次穿插，继续正常轮询。

### 地址回绕

正常地址走完 0-128 时，`HostReg_StepCycle()` 更新检测标志。

### 每轮总次数

128 个正常地址 + 约 16 次在线穿插 = ~145 次
每轮时间：145 × 50ms ≈ **7.25s**

---

## 4. 涉及的文件

| 文件 | 变更 |
|------|------|
| `Hostboard/Core/Src/modbus_polling.c` | **重写** — 8+1 穿插逻辑，reg_addr=126, bit_count=3 |
| `Hostboard/Core/Src/modbus_master_tasks.c` | **修改** — 发送任务记录 `g_host_last_req`；接收任务新增 `HostReg_StorePartialBits` 调用 |
| `Hostboard/Core/Inc/hostboard_registers.h` | **修改** — 添加 `HostReg_StorePartialBits()` 声明 |
| `Hostboard/Core/Src/hostboard_registers.c` | **修改** — 添加 `HostReg_StorePartialBits()` 实现 |
| `Hostboard/Core/Inc/uart1_modbus_master.h` | **修改** — 添加 `HostboardLastReq_t` 类型 + `g_host_last_req` 声明 |

---

## 设计检查清单

- [x] 所有轮询（含在线穿插）仅读 3 bits (126-128)
- [x] 全局变量 `g_host_last_req` 记录当前请求信息
- [x] 接收任务根据 `reg_addr`/`bit_count` 将数据写入 `coil_data` 正确位置
- [x] 每 8 次正常轮询穿插 1 次在线控制器轮询
- [x] 在线控制器动态选择（在线状态自动判定）
- [x] 地址回绕时 `HostReg_StepCycle()` 不变
