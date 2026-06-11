# Hostboard DWIN 图标更新任务设计

> 新增 TaskDwinIcons，每 500ms 批量更新 Hostboard 迪文屏上的控制器图标和系统状态图标。

## 任务架构

### TaskDwinIcons

| 属性 | 值 |
|------|-----|
| 函数 | `TaskDwinIcons` |
| 文件 | `dwin_tasks.c/h` |
| 优先级 | 1（与 DwinTx/DwinRx 同级） |
| 栈 | 128 words |
| 周期 | 500ms（`vTaskDelay(pdMS_TO_TICKS(500))`） |

- 每 500ms 组装 **5 帧** 图标数据，通过 `UART3_Send` → `xDwinTxQueue` → `TaskDwinTx` 发送
- 当前已有 DwinTxQueue 机制，直接复用

## 图标帧布局

### DWIN 变量地址

| 地址 | 用途 |
|------|------|
| `0x1800` ~ `0x187F` | 128 个控制器图标（每个 1 word，对应地址 1-128） |
| `0x1880` | 未使用（填充 0） |
| `0x1881` | 系统状态图标（1 word） |

新增常量（`dwin.h`）：

```c
#define DWIN_HOST_STATUS_ICON_ADDR  0x1881U   /* 系统状态图标地址 */
#define DWIN_ICONS_PER_FRAME        32U        /* 每帧图标数 */
#define DWIN_ICON_UPDATE_INTERVAL   500        /* 更新间隔 ms */
```

### 5 帧拆分

| 帧 | 起始地址 | 写入内容 | data_len |
|----|---------|---------|---------|
| 1 | `0x1800` | 控制器 1-32 图标 | 64 字节 |
| 2 | `0x1820` | 控制器 33-64 图标 | 64 字节 |
| 3 | `0x1840` | 控制器 65-96 图标 | 64 字节 |
| 4 | `0x1860` | 控制器 97-128 图标 | 64 字节 |
| 5 | `0x1881` | 系统状态图标（1 word） | 2 字节 |

## 图标值定义

### 控制器图标（复用现有 `DWIN_ICON_*`）

| 常量 | 值 | 条件 |
|------|----|------|
| `DWIN_ICON_OFFLINE` | `0x0000` | 控制器离线 |
| `DWIN_ICON_TROUBLE` | `0x0002` | 在线且零地址/重复地址标志为 1 |
| `DWIN_ICON_ALARM` | `0x0003` | 在线且全局报警位为 1 |
| `DWIN_ICON_NORMAL` | `0x0001` | 在线且无异常 |

### 系统状态图标（新定义，直接数值）

| 值 | 条件 |
|----|------|
| `3` | 有控制器地址重复（`HostReg_GetAddrConflict()`） |
| `2` | 存在零地址控制器（`HostReg_GetZeroAddrPresent()`） |
| `1` | 任一在线控制器有报警 |
| `0` | 正常 |

## 逻辑伪代码

```
TaskDwinIcons():
    for (;;):
        /* 构建控制器图标帧 */
        for frame = 0..3:
            base_addr = DWIN_CTRL_ICON_BASE_ADDR + frame * 32
            for i = 0..31:
                addr = frame * 32 + i + 1
                icon = calc_ctrl_icon(addr)
                buf[i*2]   = icon >> 8
                buf[i*2+1] = icon & 0xFF
            DWIN_WriteVar(base_addr, buf, 64)
        
        /* 构建系统状态图标帧 */
        status = calc_sys_icon()
        buf[0] = status >> 8
        buf[1] = status & 0xFF
        DWIN_WriteVar(DWIN_HOST_STATUS_ICON_ADDR, buf, 2)
        
        vTaskDelay(500ms)

calc_ctrl_icon(addr):
    if !HostReg_IsOnline(addr):
        return DWIN_ICON_OFFLINE
    if HostReg_GetCoilBit(addr, COIL_OFFSET_ZERO_ADDR)
       || HostReg_GetCoilBit(addr, COIL_OFFSET_ADDR_CONF):
        return DWIN_ICON_TROUBLE
    if HostReg_GetCoilBit(addr, COIL_GLOBAL_ALARM):
        return DWIN_ICON_ALARM
    return DWIN_ICON_NORMAL

calc_sys_icon():
    if HostReg_GetAddrConflict():
        return 3
    if HostReg_GetZeroAddrPresent():
        return 2
    for addr = 1..128:
        if HostReg_IsOnline(addr)
           && HostReg_GetCoilBit(addr, COIL_GLOBAL_ALARM):
            return 1
    return 0
```

## 需修改文件清单

| 文件 | 操作 | 改动 |
|------|------|------|
| `Core/Inc/dwin.h` | 修改 | 新增 `DWIN_HOST_STATUS_ICON_ADDR`、`DWIN_ICONS_PER_FRAME` |
| `Core/Inc/dwin_tasks.h` | 修改 | 声明 `TaskDwinIcons` |
| `Core/Src/dwin_tasks.c` | 修改 | 新增 `TaskDwinIcons` 函数实现 + `calc_ctrl_icon()`/`calc_sys_icon()` 辅助函数 |
| `Core/Src/main.c` | 修改 | 新增 `xTaskCreate(TaskDwinIcons, ...)` |

## 不需要改动的文件

- `dwin.c` — 帧缓冲大小不变、data_len 上限不变
- `hostboard_registers.c/h` — API 已经够用
- `CMakeLists.txt` — 无新文件
- `DwinFrame_t` — 每帧 70 字节，当前 128 字节缓冲足够
