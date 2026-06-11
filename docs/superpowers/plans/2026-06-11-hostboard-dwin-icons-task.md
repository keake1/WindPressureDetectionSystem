# Hostboard DWIN 图标更新任务实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增 TaskDwinIcons，每 500ms 批量更新 Hostboard 迪文屏图标（128 个控制器图标 + 1 个系统状态图标）

**Architecture:** 将图标更新逻辑封装到 `dwin_tasks.c` 中新增的 `TaskDwinIcons` 任务中，复用已有的发送队列机制。每轮组装 5 帧（4 帧控制器 × 32 图标 + 1 帧系统状态）调用 `DWIN_WriteVar` 发送，通过 `UART3_Send` → `xDwinTxQueue` → `TaskDwinTx` 串行发送。

**Tech Stack:** STM32F407 + FreeRTOS + DGUS II

---

## 文件清单

| 操作 | 文件 | 说明 |
|------|------|------|
| 修改 | `Core/Inc/dwin.h` | 新增 2 个宏定义 |
| 修改 | `Core/Src/dwin_tasks.c` | 新增 TaskDwinIcons + 2 个辅助函数 |
| 修改 | `Core/Inc/dwin_tasks.h` | 声明 TaskDwinIcons |
| 修改 | `Core/Src/main.c` | 创建 TaskDwinIcons 任务 |

---

### Task 1: dwin.h — 添加常量

**Files:**
- Modify: `Core/Inc/dwin.h`

- [ ] **Step 1: 新增系统状态图标地址宏**

在 `DWIN_CTRL_ICON_COUNT` 定义之后（或 `USER CODE BEGIN EConst` 内）添加：

```c
/* 系统状态图标地址（紧接控制器图标区，0x1881 处 1 个 word）*/
#define DWIN_HOST_STATUS_ICON_ADDR  0x1881U
#define DWIN_ICONS_PER_FRAME        32U       /* 每帧图标数 */
```

- [ ] **Step 2: 编译检查**

```bash
cd /home/keake/Projects/WindPressureDetectionSystem/Hostboard && cmake --preset Debug && cmake --build build/Debug 2>&1 | head -20
```
Expected: 编译通过，无错误

---

### Task 2: dwin_tasks.c — TaskDwinIcons 实现

**Files:**
- Modify: `Core/Src/dwin_tasks.c`

- [ ] **Step 1: 在 USER CODE BEGIN 0 末尾新增辅助函数声明**

```c
/* ==================== 图标计算辅助函数 ==================== */

/**
 * @brief  计算单个控制器的图标值
 * @param  addr  控制器地址 (1-128)
 * @return uint16_t 图标值（DWIN_ICON_OFFLINE/NORMAL/TROUBLE/ALARM）
 */
static uint16_t HostCalcCtrlIcon(uint8_t addr)
{
    if (!HostReg_IsOnline(addr))
        return DWIN_ICON_OFFLINE;

    if (HostReg_GetCoilBit(addr, COIL_OFFSET_ZERO_ADDR) ||
        HostReg_GetCoilBit(addr, COIL_OFFSET_ADDR_CONF))
        return DWIN_ICON_TROUBLE;

    if (HostReg_GetCoilBit(addr, COIL_GLOBAL_ALARM))
        return DWIN_ICON_ALARM;

    return DWIN_ICON_NORMAL;
}

/**
 * @brief  计算系统状态图标值
 * @return uint16_t 图标值
 *   3 = 地址重复  2 = 存在零地址  1 = 有报警  0 = 正常
 */
static uint16_t HostCalcSysIcon(void)
{
    if (HostReg_GetAddrConflict())
        return 3;

    if (HostReg_GetZeroAddrPresent())
        return 2;

    for (uint16_t addr = 1; addr <= MAX_CTRLBD_ADDR; addr++)
    {
        if (HostReg_IsOnline((uint8_t)addr) &&
            HostReg_GetCoilBit((uint8_t)addr, COIL_GLOBAL_ALARM))
        {
            return 1;
        }
    }
    return 0;
}
```

- [ ] **Step 2: 在 TaskDwinRx 函数之前（或 USER CODE BEGIN 0 末尾）新增 TaskDwinIcons**

```c
/* ==================== 迪文屏图标更新任务（每 500ms） ==================== */

void TaskDwinIcons(void *arg)
{
    (void)arg;
    uint8_t buf[64];   /* 32 icons × 2 bytes = 64 字节 */

    for (;;)
    {
        /* ---- 帧 1~4：控制器图标，每帧 32 个控制器 ---- */
        for (uint8_t frame = 0; frame < 4; frame++)
        {
            uint16_t base_addr = DWIN_CTRL_ICON_BASE_ADDR + (uint16_t)frame * DWIN_ICONS_PER_FRAME * 2U;
            uint8_t  base_slave = frame * DWIN_ICONS_PER_FRAME + 1;

            for (uint8_t i = 0; i < DWIN_ICONS_PER_FRAME; i++)
            {
                uint16_t icon = HostCalcCtrlIcon(base_slave + i);
                buf[i * 2]     = (uint8_t)(icon >> 8);
                buf[i * 2 + 1] = (uint8_t)(icon & 0xFF);
            }

            DWIN_WriteVar(base_addr, buf, DWIN_ICONS_PER_FRAME * 2);
        }

        /* ---- 帧 5：系统状态图标（地址 0x1881，1 个 word） ---- */
        uint16_t sys_icon = HostCalcSysIcon();
        buf[0] = (uint8_t)(sys_icon >> 8);
        buf[1] = (uint8_t)(sys_icon & 0xFF);
        DWIN_WriteVar(DWIN_HOST_STATUS_ICON_ADDR, buf, 2);

        /* ---- 等待 500ms 进入下一轮 ---- */
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

- [ ] **Step 3: 添加缺失的 #include**

确保文件顶部包含 `hostboard_registers.h`。在 `dwin_tasks.c` 的现有 include 列表中已有 `dwin.h`，但未包含 `hostboard_registers.h`。在 `/* USER CODE BEGIN Includes */` 区域添加：

```c
/* USER CODE BEGIN Includes */
#include "hostboard_registers.h"
/* USER CODE END Includes */
```

---

### Task 3: dwin_tasks.h — 声明 TaskDwinIcons

**Files:**
- Modify: `Core/Inc/dwin_tasks.h`

- [ ] **Step 1: 在已有函数声明之后添加**

```c
void TaskDwinIcons(void *arg);
```

---

### Task 4: main.c — 创建任务

**Files:**
- Modify: `Core/Src/main.c`

- [ ] **Step 1: 在 main.c 的 USART3 任务创建代码后，vTaskStartScheduler 之前添加**

```c
  /* ---- 创建迪文屏图标更新任务（优先级 1） ---- */
  xTaskCreate(TaskDwinIcons, "DwinIcons", 128, NULL, 1, NULL);
```

完整上下文（修改后）：
```c
  /* ---- 创建 USART3 迪文屏发送/接收任务 ---- */
  xTaskCreate(TaskDwinTx, "DwinTx", 128, NULL, 1, NULL);
  xTaskCreate(TaskDwinRx, "DwinRx", 128, NULL, 1, NULL);

  /* ---- 创建迪文屏图标更新任务（优先级 1） ---- */
  xTaskCreate(TaskDwinIcons, "DwinIcons", 128, NULL, 1, NULL);
```

---

### Task 5: 编译验证

- [ ] **Step 1: 编译 Hostboard**

```bash
cd /home/keake/Projects/WindPressureDetectionSystem/Hostboard && cmake --build build/Debug 2>&1
```

Expected: **0 error, 0 warning**

---

## 自检清单

- [ ] `DWIN_HOST_STATUS_ICON_ADDR` = `0x1881`（紧接 128 控制器图标后的独立地址，不与 `0x1800-0x187F` 重叠）
- [ ] 每帧 32 图标 × 2 字节 = 64 字节 data_len，未超过 `DWIN_WriteVar` 的 64 字节限制
- [ ] 帧总长 70 字节，未超过 `DwinFrame_t.data[128]` 和 `DWIN_TX_FRAME_MAX` 128
- [ ] 系统状态 2 字节 data_len，在限制内
- [ ] 优先级 1 不抢占 Modbus 收发任务（优先级 2）
- [ ] 500ms 周期足够长，不会淹没 DwinTxQueue（队列深度 8）
