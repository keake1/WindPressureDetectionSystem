# Hostboard Controller Info Struct 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 `board_coil_data[129][17]` 替换为 `CtrlBoardEntry_t` 结构体数组，合并线圈数据+在线标志+离线检测追踪。

**Architecture:** 定义 `CtrlBoardEntry_t { coil_data[17], last_seen_cycle, online }`，在 `StoreCoilData` 中自动更新在线标志和周期，在 `StepCycle` 中检测连续三轮无响应→标记离线。

**Tech Stack:** C (STM32F407, FreeRTOS), Hostboard 寄存器模块

---

### Task 1: 更新 hostboard_registers.h — 添加结构体类型和 API 声明

**Files:**
- Modify: `Hostboard/Core/Inc/hostboard_registers.h`

- [ ] **Step 1: 添加 `CtrlBoardEntry_t` 类型定义**

在 `hostboard_registers.h` 的 `USER CODE BEGIN ET` 区块（Exported types）中添加：

```c
/* USER CODE BEGIN ET */

/**
 * @brief  Controlboard 信息结构体
 *
 * 每个地址 (1-128) 对应一个结构体实例，合并存储线圈数据、
 * 在线标志和离线检测追踪信息。
 */
typedef struct {
    uint8_t  coil_data[COIL_BYTE_COUNT];  /* 17 字节原始线圈数据 */
    uint32_t last_seen_cycle;              /* 最近收到正确响应时的周期号 */
    uint8_t  online;                       /* 1=在线, 0=离线 */
} CtrlBoardEntry_t;

/* USER CODE END ET */
```

- [ ] **Step 2: 添加 `HostReg_IsOnline()` 声明**

在 `HostReg_Data_Storage` 组或新建一个在线检测组，添加：

```c
/** @defgroup HostReg_Online 在线检测 */
/** @{ */
uint8_t HostReg_IsOnline(uint8_t addr);
/** @} */
```

放到现有函数声明后面，例如在 `HostReg_GetAddrConflict()` 之后。

---

### Task 2: 更新 hostboard_registers.c — 替换存储结构 + 添加在线检测逻辑

**Files:**
- Modify: `Hostboard/Core/Src/hostboard_registers.c`

- [ ] **Step 1: 替换静态存储变量**

将：
```c
static uint8_t  board_coil_data[MAX_CTRLBD_ADDR + 1][COIL_BYTE_COUNT];
```
替换为：
```c
static CtrlBoardEntry_t ctrl_boards[MAX_CTRLBD_ADDR + 1];
/* ctrl_boards[0] 预留（地址 0 有效响应不计入存储） */
/* ctrl_boards[1..128] 对应地址 1-128 的 Controlboard */
```

并且将 `current_cycle` 从 `uint8_t` 循环索引改为 `uint32_t` 递增计数器：

```c
/* 修改前 */
static uint8_t  current_cycle;                   /* 0, 1, 2 循环 */

/* 修改后 */
static uint32_t current_cycle;                   /* 递增周期计数器，用于在线检测 */
```

- [ ] **Step 2: 修改 `HostReg_StoreCoilData()` — 写入后自动更新在线标志**

```c
void HostReg_StoreCoilData(uint8_t addr, const uint8_t *data, uint8_t len)
{
    if (addr < 1 || addr > MAX_CTRLBD_ADDR)
        return;
    if (len > COIL_BYTE_COUNT)
        len = COIL_BYTE_COUNT;
    memcpy(ctrl_boards[addr].coil_data, data, len);

    /* 收到有效响应 → 更新在线标志和周期 */
    ctrl_boards[addr].online = 1;
    ctrl_boards[addr].last_seen_cycle = current_cycle;
}
```

- [ ] **Step 3: 修改 `HostReg_GetCoilByte()` — 改用结构体数组**

```c
uint8_t HostReg_GetCoilByte(uint8_t addr, uint8_t byte_idx)
{
    if (addr < 1 || addr > MAX_CTRLBD_ADDR)
        return 0;
    if (byte_idx >= COIL_BYTE_COUNT)
        return 0;
    return ctrl_boards[addr].coil_data[byte_idx];
}
```

- [ ] **Step 4: 修改 `HostReg_GetCoilBit()` — 改用结构体数组**

```c
uint8_t HostReg_GetCoilBit(uint8_t addr, uint16_t bit_idx)
{
    if (addr < 1 || addr > MAX_CTRLBD_ADDR)
        return 0;
    if (bit_idx >= COIL_BYTE_COUNT * 8)
        return 0;
    return (ctrl_boards[addr].coil_data[bit_idx / 8] >> (bit_idx % 8)) & 1;
}
```

- [ ] **Step 5: 添加 `HostReg_IsOnline()` 实现**

在 `StepCycle` 函数前面或后面合适位置添加：

```c
/* ==================== 在线检测 API ==================== */

/**
 * @brief  查询 Controlboard 在线状态
 * @param  addr  Controlboard 地址 (1-128)
 * @retval 1 在线，0 离线或地址越界
 */
uint8_t HostReg_IsOnline(uint8_t addr)
{
    if (addr < 1 || addr > MAX_CTRLBD_ADDR)
        return 0;
    return ctrl_boards[addr].online;
}
```

- [ ] **Step 6: 修改 `HostReg_StepCycle()` — 增加在线检测循环**

零地址和重复地址检测的环形缓冲索引改为 `current_cycle % 3`：

```c
void HostReg_StepCycle(void)
{
    current_cycle++;

    /* ---- 在线检测：检查所有 Controlboard (1-128) ---- */
    for (uint8_t addr = 1; addr <= MAX_CTRLBD_ADDR; addr++)
    {
        if (ctrl_boards[addr].last_seen_cycle + 3 <= current_cycle)
        {
            ctrl_boards[addr].online = 0;  /* 连续三轮无响应 → 离线 */
        }
    }

    /* ---- 重复地址检测：三轮错误计数累加 > 2 ---- */
    crc_error_history[current_cycle % 3] = crc_error_count;
    crc_error_count = 0;

    uint32_t sum = crc_error_history[0] + crc_error_history[1] + crc_error_history[2];
    addr_conflict_flag = (sum > 2) ? 1 : 0;

    /* ---- 零地址检测：三轮 OR ---- */
    zero_addr_history[current_cycle % 3] = zero_addr_found_this_cycle;
    zero_addr_found_this_cycle = 0;
    zero_addr_present = zero_addr_history[0]
                     || zero_addr_history[1]
                     || zero_addr_history[2];
}
```

---

### Task 3: 编译验证

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
- [x] CtrlBoardEntry_t 结构体含 coil_data[17] + last_seen_cycle + online → Task 1 Step 1
- [x] 128 元素数组（地址 1-128）→ Task 2 Step 1
- [x] 在线离线检测：StoreCoilData 自动更新 + StepCycle 每轮检查 → Task 2 Step 2, Step 6
- [x] 查询在线状态 HostReg_IsOnline() → Task 1 Step 2 + Task 2 Step 5
- [x] 零地址/重复地址检测不变 → Task 2 Step 6 (环形缓冲改为 %3)
- [x] 对外部模块无侵入 → 无其他文件修改
- [x] 位含义通过头文件常量表达 → 已有 COIL_OFFSET_* 常量不变
