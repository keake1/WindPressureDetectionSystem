# Hostboard Controller Info Struct — 统一结构体 + 在线离线检测

> **设计目标**：将 `board_coil_data[129][17]` 裸二维数组替换为 `CtrlBoardEntry_t` 结构体数组，合并每路 Controlboard 的线圈数据、在线标志和离线检测追踪信息，方便检索和阅读。
>
> **关联设计**：`2026-06-11-hostboard-multi-controlboard-polling-design.md`

---

## 1. 变更动机

当前 `hostboard_registers.c` 用裸二维数组 `board_coil_data[129][17]` 存储线圈数据，没有每个 Controlboard 的元信息：

- 无法快速查询某个 Controlboard 是否在线
- 没有类似 Controlboard 的离线检测机制（`last_seen_cycle` + `StepCycle` 已实现但不含在线状态）
- 检索数据时须同时维护线圈数据和在线标志两个独立变量

**解决方案**：将线圈数据、在线标志、离线追踪整合为一个结构体，定义 128 个元素的数组。

---

## 2. 结构体定义

```c
#define MAX_CTRLBD_ADDR     128
#define COIL_BYTE_COUNT     17

typedef struct {
    uint8_t  coil_data[COIL_BYTE_COUNT];  /* 17 字节原始线圈数据 */
    uint32_t last_seen_cycle;              /* 最近收到正确响应时的周期号 */
    uint8_t  online;                       /* 1=在线, 0=离线 */
} CtrlBoardEntry_t;
```

**存储方式**：

```c
static CtrlBoardEntry_t ctrl_boards[MAX_CTRLBD_ADDR + 1];
/* ctrl_boards[0] 预留（地址 0 有效响应不计入存储） */
/* ctrl_boards[1..128] 对应地址 1-128 的 Controlboard */
```

与当前 `board_coil_data[129][17]` 的内存布局兼容 —— `ctrl_boards[addr].coil_data[byte]` 等价于 `board_coil_data[addr][byte]`。

---

## 3. 线圈数据含义（位映射）

17 字节 = 136 bits，实际使用 130 bits（位 0-129），与 Controlboard 线圈寄存器映射一致：

| 字节偏移 | 位范围 | 内容 |
|---------|--------|------|
| 0-6 | 位 0-55 | 传感器 #1-56 在线状态（每传感器 1 bit） |
| 7 | 位 56-62 | 传感器 #57-63 在线状态 |
| 7 | 位 63 | 传感器 #1 报警标志 |
| 8-14 | 位 64-118 | 传感器 #2-58 报警标志 |
| 15 | 位 119-125 | 传感器 #59-63 报警标志 |
| 15 | 位 126 | 零地址传感器存在标志 |
| 15 | 位 127 | 地址重复标志 |
| 16 | 位 128 | 全局报警（任一传感器报警） |
| 16 | 位 129 | 烟雾报警器状态 |

API 保持不变，通过 `HostReg_GetCoilBit(addr, bit_idx)` 配合上表常量（`COIL_OFFSET_ONLINE`、`COIL_OFFSET_ALARM` 等）访问。

---

## 4. 在线离线检测机制

参考 Controlboard `modbus_registers.c` 的 `last_seen_cycle` 机制：

### 4.1 记录响应

`HostReg_StoreCoilData(addr, data, len)` 被调用时（收到 CRC 有效的 FC 0x02 响应）：

```c
ctrl_boards[addr].last_seen_cycle = current_cycle;
ctrl_boards[addr].online = 1;
```

### 4.2 周期性检测

`HostReg_StepCycle()` 每轮完整扫描（地址 0→128→回绕）结束时调用，扩展为：

```c
void HostReg_StepCycle(void)
{
    current_cycle++;

    /* ---- 在线检测：检查所有 Controlboard 地址 ---- */
    for (uint8_t addr = 1; addr <= MAX_CTRLBD_ADDR; addr++)
    {
        if (ctrl_boards[addr].last_seen_cycle + 3 <= current_cycle)
        {
            ctrl_boards[addr].online = 0;  /* 连续三轮无响应 → 离线 */
        }
    }

    /* ---- 零地址检测（不变） ---- */
    zero_addr_history[current_cycle % 3] = zero_addr_found_this_cycle;
    zero_addr_found_this_cycle = 0;
    zero_addr_present = zero_addr_history[0] || zero_addr_history[1] || zero_addr_history[2];

    /* ---- 地址重复检测（不变） ---- */
    crc_error_history[current_cycle % 3] = crc_error_count;
    crc_error_count = 0;
    uint32_t sum = crc_error_history[0] + crc_error_history[1] + crc_error_history[2];
    addr_conflict_flag = (sum > 2) ? 1 : 0;
}
```

### 4.3 查询在线状态

新增 API：

```c
uint8_t HostReg_IsOnline(uint8_t addr);
/* 返回 ctrl_boards[addr].online，addr 越界返回 0 */
```

### 4.4 时序

| 参数 | 值 |
|------|-----|
| 轮询间隔 | 50ms |
| 完整一轮 | 129 × 50ms = 6.45s |
| 离线判定窗口 | 连续 3 轮无响应 ≈ 19.35s |

---

## 5. API 变更总览

| API | 类型 | 说明 |
|-----|------|------|
| `HostReg_StoreCoilData(addr, data, len)` | **修改** | 写入后自动更新 `last_seen_cycle` + `online` |
| `HostReg_GetCoilByte(addr, byte_idx)` | 不变 | 改为读 `ctrl_boards[addr].coil_data` |
| `HostReg_GetCoilBit(addr, bit_idx)` | 不变 | 同上 |
| `HostReg_RecordZeroAddrResponse()` | 不变 | — |
| `HostReg_GetZeroAddrPresent()` | 不变 | — |
| `HostReg_RecordError()` | 不变 | — |
| `HostReg_GetAddrConflict()` | 不变 | — |
| `HostReg_StepCycle()` | **修改** | 增加在线检测循环 |
| `HostReg_IsOnline(addr)` | **新增** | 查询 Controlboard 在线状态 |

---

## 6. 内部状态变量变更

**替换**：

```c
/* 替换前 */
static uint8_t board_coil_data[MAX_CTRLBD_ADDR + 1][COIL_BYTE_COUNT];

/* 替换后 */
static CtrlBoardEntry_t ctrl_boards[MAX_CTRLBD_ADDR + 1];
```

**新增**：

```c
static uint32_t current_cycle;  /* 已存在，扩展用途：从 0..2 循环改为递增计数器 */
```

**其余状态变量**（零地址检测、重复地址检测）保持不变。

---

## 7. 对其他模块的影响

| 模块 | 影响 |
|------|------|
| `modbus_master_tasks.c` | 无影响。接收任务继续调用 `HostReg_StoreCoilData()`，在线/离线更新在模块内部自动完成 |
| `modbus_polling.c` | 无影响。轮询任务仅调用 `HostReg_StepCycle()`，在线/离线检测在模块内部自动完成 |
| `hostboard_registers.h` | 仅需添加 `HostReg_IsOnline()` 声明和 `CtrlBoardEntry_t` 类型定义（可选是否暴露类型） |
| 应用层 | 通过 `HostReg_IsOnline(addr)` 查询在线状态 |

---

## 设计检查清单

- [x] 统一结构体包含：线圈数据 + 在线标志 + 离线追踪
- [x] 128 个元素的数组（地址 1-128）
- [x] 在线离线检测机制：`last_seen_cycle` + `StepCycle` 每轮检查
- [x] 保持与现有 `board_coil_data` 内存布局兼容
- [x] 位含义通过头文件常量清晰表达，不拆分子数组
- [x] 零地址检测和重复地址检测机制不变
- [x] 对外部模块无侵入
