# Hostboard 迪文屏详情界面设计文档

> **功能：** 在 Hostboard 迪文屏上，当用户选中一个控制器（selectAddr ≠ 0）时，切换显示该控制器的详情视图——包括窗状态、报警标志、传感器图标和传感器数据。

---

## 1. 概述

当前 Hostboard 迪文屏只有主界面：128 个控制器图标 + 1 个系统状态图标（`0x1800-0x1881`）。当用户在屏幕上点击某个控制器进入详情后，需要切换到一个类似 Controlboard 界面的详情视图，展示该控制器下传感器 1-63 的图标和实时数据。

### 双模式架构

```
selectAddr == 0  →  主界面模式（现有逻辑）
selectAddr != 0  →  详情界面模式（新增逻辑）
```

两种模式的 DWIN 更新地址完全不同，由 TaskDwinIcons 根据 `g_hostDwinStatus.selectAddr` 切换。

### 数据流向

```
选中控制器
  │
  ├── FC 0x02 (130 bits) → 线圈数据 → HostReg_StoreCoilData → 图标判定
  │
  ├── FC 0x03 (reg=n, count=1) → 传感器型号 → DetailView_SetType()
  │
  └── FC 0x03 (reg=64+(n-1)*7, count=7) → 传感器数值
        │
        ↓ 接收任务中转换格式
    DWIN_WriteVar(0x1900+偏移, data_buf, 32) → 迪文屏显示
```

---

## 2. 存储器模块（新建）

### 文件

- **新建** `Core/Src/detail_view_data.c`
- **新建** `Core/Inc/detail_view_data.h`

### 数据结构

```c
#define DETAIL_SENSOR_MAX   64   /* 传感器索引 1-63 */

static uint8_t  s_detail_types[DETAIL_SENSOR_MAX];  /* 传感器型号 */
static uint8_t  s_detail_coil_ready;                 /* 全量线圈已采集 */
static uint8_t  s_detail_select_addr;                /* 记录针对哪个控制器的数据 */
```

`s_detail_types[n]` 存储传感器 n 的型号（0x01-0x04），在收到 FC 0x03 reg=n 的响应时写入。

### API

```c
/* 选中控制器切换时调用：清空所有缓冲 */
void DetailView_Reset(void);

/* 存入传感器型号 */
void DetailView_SetType(uint8_t sensor_idx, uint8_t type);

/* 读取传感器型号 */
uint8_t DetailView_GetType(uint8_t sensor_idx);

/* 标记 / 查询线圈数据就绪状态 */
void DetailView_SetCoilReady(uint8_t addr);
uint8_t DetailView_IsCoilReady(uint8_t addr);
```

### 数据量

- `s_detail_types`: 64 × 1 = 64 字节
- `s_detail_coil_ready`: 1 字节
- `s_detail_select_addr`: 1 字节
- 总计约 **70 字节**（远小于 STM32F407 的 192KB RAM）

---

## 3. 轮询任务扩展

### 文件

- **修改** `Core/Src/modbus_polling.c`

### 3.1 新增内部变量

```c
/* 详情采集状态机 */
static uint8_t  s_detail_poll_interval;  /* 插队间隔计数器 */
static uint8_t  s_detail_state;          /* 采集阶段 (0-4) */
static uint8_t  s_detail_next;           /* 下一个要采集的传感器索引 (1-63) */
```

### 3.2 插队策略

在现有每轮 50ms 循环中，通过计数器控制插队频率：

```
常规轮询 × 2 次 → 详情采集 × 1 次 → 循环
```

即每 3 个周期中有 1 个周期执行详情采集，不执行常规轮询。这样常规轮询速度降为原来的 2/3，但不会完全被阻塞。

### 3.3 详情采集状态机

```c
typedef enum {
    DETAIL_IDLE       = 0,  /* selectAddr == 0，不执行 */
    DETAIL_NEED_COIL  = 1,  /* 需要读全量线圈 */
    DETAIL_NEXT_TYPE  = 2,  /* 读取下一个在线传感器的型号 */
    DETAIL_NEXT_DATA  = 3,  /* 读取下一个在线传感器的数据 */
    DETAIL_LOOP_BACK  = 4,  /* 一轮完成，准备下一轮 */
} DetailState_t;
```

**状态转移：**

```
selectAddr 从 0 → N
  │
  ▼
DETAIL_NEED_COIL
  │  发 FC 0x02 (addr=N, reg=0, count=130)
  │  等待 DetailView_IsCoilReady(N) == 1
  │
  ▼
DETAIL_NEXT_TYPE
  │  找下一个在线传感器 n（从 s_detail_next 开始）
  │  发 FC 0x03 (addr=N, reg=n, count=1)
  │  s_detail_next++ → DETAIL_NEXT_DATA
  │
  ▼
DETAIL_NEXT_DATA
  │  等待 DetailView_GetType(n) 非 0
  │  发 FC 0x03 (addr=N, reg=64+(n-1)*7, count=7)
  │  → DETAIL_NEXT_TYPE（继续下一个）
  │
  │  如果 s_detail_next > 63（全部在线传感器扫完）
  │  ▼
DETAIL_LOOP_BACK
  │  重置 s_detail_next = 1
  │  → DETAIL_NEXT_TYPE
```

### 3.4 伪代码

```c
void TaskModbusPoll(void *arg)
{
    // ... 开局加载备注（不变）...
    
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(50));

        uint8_t selectAddr = g_hostDwinStatus.selectAddr;

        /* ── 插队判定：每 3 次执行 1 次详情采集 ── */
        if (selectAddr != 0 && ++s_detail_poll_interval >= 3)
        {
            s_detail_poll_interval = 0;
            DetailCollect_Step(selectAddr);
            continue;
        }

        /* ── 常规轮询逻辑（不变）── */
        // ... 8+1 穿插逻辑 ...
    }
}
```

```c
static void DetailCollect_Step(uint8_t selectAddr)
{
    switch (s_detail_state)
    {
        case DETAIL_IDLE:
            if (selectAddr != 0)
            {
                s_detail_state = DETAIL_NEED_COIL;
                s_detail_next = 1;
                DetailView_Reset();
            }
            break;

        case DETAIL_NEED_COIL:
            if (!DetailView_IsCoilReady(selectAddr))
            {
                /* 发起 FC 0x02 读 130 bits */
                ModbusMasterRequest_t req = {
                    .slave_addr = selectAddr,
                    .func_code  = MODBUS_FUNC_READ_DISCRETE_INPUTS,
                    .reg_addr   = 0,
                    .reg_value  = 130
                };
                ModbusMaster_EnqueueRequest(&req);
            }
            else
            {
                s_detail_state = DETAIL_NEXT_TYPE;
            }
            break;

        case DETAIL_NEXT_TYPE:
        {
            /* 找下一个在线传感器 */
            while (s_detail_next <= 63)
            {
                if (HostReg_GetCoilBit(selectAddr, s_detail_next - 1))
                {
                    if (DetailView_GetType(s_detail_next) == 0)
                    {
                        /* 发 FC 0x03 读型号 */
                        ModbusMasterRequest_t req = {
                            .slave_addr = selectAddr,
                            .func_code  = MODBUS_FUNC_READ_HOLDING_REGISTERS,
                            .reg_addr   = s_detail_next,
                            .reg_value  = 1
                        };
                        ModbusMaster_EnqueueRequest(&req);
                        s_detail_state = DETAIL_NEXT_DATA;
                        return;
                    }
                }
                s_detail_next++;
            }
            /* 全部扫完 → 回绕 */
            s_detail_next = 1;
            s_detail_state = DETAIL_NEXT_TYPE;
            break;
        }

        case DETAIL_NEXT_DATA:
        {
            uint8_t n = s_detail_next;
            if (DetailView_GetType(n) != 0)
            {
                /* 发 FC 0x03 读 7 个数据寄存器 */
                ModbusMasterRequest_t req = {
                    .slave_addr = selectAddr,
                    .func_code  = MODBUS_FUNC_READ_HOLDING_REGISTERS,
                    .reg_addr   = 64 + (n - 1) * 7,
                    .reg_value  = 7
                };
                ModbusMaster_EnqueueRequest(&req);

                s_detail_next++;
                s_detail_state = DETAIL_NEXT_TYPE;
            }
            else
            {
                /* 型号尚未就绪 → 等下次 */
            }
            break;
        }
    }
}
```

---

## 4. 接收任务扩展

### 文件

- **修改** `Core/Src/modbus_master_tasks.c`

在 `TaskModbusRecv` 的 CRC 通过分支中新增 FC 0x03 处理：

```c
/* 在现有 FC 0x02 处理之后添加 */

else if (func_code == 0x03
         && slave_addr == g_hostDwinStatus.selectAddr
         && slave_addr == g_host_last_req.slave_addr)
{
    uint8_t byte_cnt  = raw.frame[2];
    uint8_t reg_count = byte_cnt / 2;
    uint16_t reg_addr = g_host_last_req.reg_addr;
    uint16_t regs[7];

    if (byte_cnt > 14)
        byte_cnt = 14;  /* 最多 7 个寄存器 */

    for (uint8_t i = 0; i < byte_cnt / 2; i++)
    {
        regs[i] = ((uint16_t)raw.frame[3 + i * 2] << 8)
                | raw.frame[3 + i * 2 + 1];
    }

    if (reg_addr >= 1 && reg_addr <= 63)
    {
        /* 传感器型号响应：Controlboard 将 uint8 改为 uint16，型号在低字节 */
        DetailView_SetType((uint8_t)reg_addr, (uint8_t)(regs[0] & 0xFF));
    }
    else if (reg_addr >= 64 && reg_addr <= 504)
    {
        /* 传感器数据响应 */
        uint8_t sensor_idx = (reg_addr - 64) / 7 + 1;
        uint8_t type = DetailView_GetType(sensor_idx);
        DetailPushToDwin(sensor_idx, type, regs);
    }
}
```

### 数据转换及推屏函数

```c
static void DetailPushToDwin(uint8_t sensor_idx, uint8_t type, const uint16_t *regs)
{
    uint8_t buf[32];
    memset(buf, 0, sizeof(buf));
    uint16_t addr = DWIN_DETAIL_SENSOR_DATA_ADDR + (uint16_t)(sensor_idx - 1) * 16;

    switch (type)
    {
        case 0x01:  /* CO 传感器 */
        {
            /* regs[0] ÷ 100 → float → buf[0..3] */
            float val = (float)(regs[0]) / 100.0f;
            uint32_t *p = (uint32_t *)&val;
            buf[0] = (uint8_t)(*p >> 24);
            buf[1] = (uint8_t)(*p >> 16);
            buf[2] = (uint8_t)(*p >> 8);
            buf[3] = (uint8_t)(*p);
            break;
        }
        case 0x02:  /* 风压传感器 */
        {
            /* regs[0] → float → buf[0..3] */
            float val = (float)(regs[0]);
            uint32_t *p = (uint32_t *)&val;
            buf[0] = (uint8_t)(*p >> 24);
            buf[1] = (uint8_t)(*p >> 16);
            buf[2] = (uint8_t)(*p >> 8);
            buf[3] = (uint8_t)(*p);
            break;
        }
        case 0x03:  /* 余压传感器 */
        {
            /* regs[0] → uint16 → buf[4..5] */
            buf[4] = (uint8_t)(regs[0] >> 8);
            buf[5] = (uint8_t)(regs[0] & 0xFF);
            break;
        }
        case 0x04:  /* 7 合 1 传感器 */
        {
            /* eCO₂   → buf[10..11] uint16 */
            buf[10] = (uint8_t)(regs[0] >> 8);
            buf[11] = (uint8_t)(regs[0] & 0xFF);
            /* eCH₂O  → buf[12..13] uint16 */
            buf[12] = (uint8_t)(regs[1] >> 8);
            buf[13] = (uint8_t)(regs[1] & 0xFF);
            /* TVOC   → buf[14..15] uint16 */
            buf[14] = (uint8_t)(regs[2] >> 8);
            buf[15] = (uint8_t)(regs[2] & 0xFF);
            /* PM2.5  → buf[16..19] float */
            {
                float v = (float)(regs[3]);
                uint32_t *p = (uint32_t *)&v;
                buf[16] = (uint8_t)(*p >> 24);
                buf[17] = (uint8_t)(*p >> 16);
                buf[18] = (uint8_t)(*p >> 8);
                buf[19] = (uint8_t)(*p);
            }
            /* PM10   → buf[20..23] float */
            {
                float v = (float)(regs[4]);
                uint32_t *p = (uint32_t *)&v;
                buf[20] = (uint8_t)(*p >> 24);
                buf[21] = (uint8_t)(*p >> 16);
                buf[22] = (uint8_t)(*p >> 8);
                buf[23] = (uint8_t)(*p);
            }
            /* 温度   → buf[24..27] float (regs[5] / 10) */
            {
                float v = (float)(regs[5]) / 10.0f;
                uint32_t *p = (uint32_t *)&v;
                buf[24] = (uint8_t)(*p >> 24);
                buf[25] = (uint8_t)(*p >> 16);
                buf[26] = (uint8_t)(*p >> 8);
                buf[27] = (uint8_t)(*p);
            }
            /* 湿度   → buf[28..31] float (regs[6] / 10) */
            {
                float v = (float)(regs[6]) / 10.0f;
                uint32_t *p = (uint32_t *)&v;
                buf[28] = (uint8_t)(*p >> 24);
                buf[29] = (uint8_t)(*p >> 16);
                buf[30] = (uint8_t)(*p >> 8);
                buf[31] = (uint8_t)(*p);
            }
            break;
        }
        default:
            return;  /* 未知型号，不发送 */
    }

    DWIN_WriteVar(addr, buf, 32);
}
```

> **注意：** float 到大端字节序的转换使用指针强转，需要确保 STM32F407 的 float 布局为 IEEE 754 标准。如果编译选项未启用硬浮点，`float` 操作可能通过软浮点库完成，代码体积会增加，但功能正常。

---

## 5. DWIN 图标任务修改

### 文件

- **修改** `Core/Src/dwin_tasks.c`

### 5.1 新增辅助函数

```c
/**
 * @brief  计算选中控制器的某个传感器图标值
 * @param  selectAddr  控制器地址
 * @param  sensor_idx  传感器索引 (1-63)
 * @return uint16_t 图标值
 */
static uint16_t DetailSensorIcon(uint8_t selectAddr, uint8_t sensor_idx)
{
    if (!HostReg_GetCoilBit(selectAddr, sensor_idx - 1))
        return DWIN_ICON_OFFLINE;

    if (HostReg_GetCoilBit(selectAddr, COIL_OFFSET_ALARM + sensor_idx - 1))
        return DWIN_ICON_ALARM;

    return DWIN_ICON_NORMAL;
}
```

### 5.2 TaskDwinIcons 双模式

```c
void TaskDwinIcons(void *arg)
{
    (void)arg;
    uint8_t buf[64];

    for (;;)
    {
        uint8_t selectAddr = g_hostDwinStatus.selectAddr;

        if (selectAddr == 0)
        {
            /* ===== 主界面模式（现有逻辑） ===== */
            for (uint8_t frame = 0; frame < 4; frame++)
            {
                uint16_t base  = DWIN_CTRL_ICON_BASE_ADDR + (uint16_t)frame * DWIN_ICONS_PER_FRAME;
                uint8_t  start = frame * DWIN_ICONS_PER_FRAME + 1;
                for (uint8_t i = 0; i < DWIN_ICONS_PER_FRAME; i++)
                {
                    uint16_t icon = HostCalcCtrlIcon(start + i);
                    buf[i * 2]     = (uint8_t)(icon >> 8);
                    buf[i * 2 + 1] = (uint8_t)(icon & 0xFF);
                }
                DWIN_WriteVar(base, buf, 64);
            }
            uint16_t sys = HostCalcSysIcon();
            buf[0] = (uint8_t)(sys >> 8);
            buf[1] = (uint8_t)(sys & 0xFF);
            DWIN_WriteVar(DWIN_HOST_STATUS_ICON_ADDR, buf, 2);
        }
        else
        {
            /* ===== 详情界面模式 ===== */

            /* 帧 1：窗状态 0x1882 */
            uint8_t win = HostReg_GetCoilBit(selectAddr, COIL_SMOKE_ALARM)
                       || HostReg_GetCoilBit(selectAddr, COIL_GLOBAL_ALARM);
            buf[0] = 0; buf[1] = win;
            DWIN_WriteVar(DWIN_DETAIL_WIN_ICON_ADDR, buf, 2);

            /* 帧 2：全局报警 0x1883 */
            buf[1] = HostReg_GetCoilBit(selectAddr, COIL_GLOBAL_ALARM);
            DWIN_WriteVar(DWIN_DETAIL_GLB_ALARM_ADDR, buf, 2);

            /* 帧 3：零地址 0x1884 */
            buf[1] = HostReg_GetCoilBit(selectAddr, COIL_OFFSET_ZERO_ADDR);
            DWIN_WriteVar(DWIN_DETAIL_ZERO_ADDR_ADDR, buf, 2);

            /* 帧 4：重复地址 0x1885 */
            buf[1] = HostReg_GetCoilBit(selectAddr, COIL_OFFSET_ADDR_CONF);
            DWIN_WriteVar(DWIN_DETAIL_CONF_ADDR_ADDR, buf, 2);

            /* 帧 5-7：传感器图标 0x1890-0x18CF（63 个图标）*/
            uint8_t *p = buf;
            uint8_t count = 0;
            uint16_t icon_addr = DWIN_DETAIL_SENSOR_ICON_ADDR;

            for (uint8_t i = 1; i <= 63; i++)
            {
                uint16_t icon = DetailSensorIcon(selectAddr, i);
                *p++ = (uint8_t)(icon >> 8);
                *p++ = (uint8_t)(icon & 0xFF);
                count++;

                if (count == 32 || i == 63)
                {
                    DWIN_WriteVar(icon_addr, buf, count * 2);
                    icon_addr += count;
                    count = 0;
                    p = buf;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
```

### 5.3 传感器图标分帧说明

63 个图标分为 3 帧发送：

| 帧 | 地址 | 图标数 | 数据长度 |
|---|------|--------|---------|
| 第1帧 | 0x1890 | 32 | 64 字节 |
| 第2帧 | 0x18B0 | 31 | 62 字节 |

---

## 6. 常量宏定义

### 文件

- **修改** `Core/Inc/dwin.h`（`USER CODE BEGIN EConst` 区域）

```c
/* 详情界面常量（在现有常量之后） */
#define DWIN_DETAIL_WIN_ICON_ADDR      0x1882U  /* 窗状态图标（选中控制器）*/
#define DWIN_DETAIL_GLB_ALARM_ADDR      0x1883U  /* 全局报警图标 */
#define DWIN_DETAIL_ZERO_ADDR_ADDR      0x1884U  /* 零地址图标 */
#define DWIN_DETAIL_CONF_ADDR_ADDR      0x1885U  /* 重复地址图标 */
#define DWIN_DETAIL_SENSOR_ICON_ADDR    0x1890U  /* 传感器图标起始地址 */
#define DWIN_DETAIL_SENSOR_DATA_ADDR    0x1900U  /* 传感器数据起始地址 */
```

---

## 7. 构建集成

### 文件

- **修改** `Hostboard/CMakeLists.txt` — 添加 `Core/Src/detail_view_data.c`

---

## 8. 时序估算

假设选中控制器有 10 个在线传感器：

| 阶段 | 请求数 | 耗时 |
|------|--------|------|
| FC 0x02 全量线圈 | 1 | 插队 1 次 = 50ms |
| FC 0x03 读 10 个型号 | 10 | 插队 10 次 = 10 × 50ms × 3 = 1.5s 墙钟 |
| FC 0x03 读 10 个数据 | 10 | 同上，穿插交错进行 |
| **首轮完整采集** | **约 20 次** | **约 1.5-2s 墙钟** |

> 因为型号和数据采集在同一个插队周期内连续执行（先发型号 → 下次插队发数据），实际上两个阶段是交错的。10 个在线传感器约 20 次插队 × 每 3 周期插 1 次 × 50ms = 3s 墙钟完成首轮。

后续持续刷新每轮约 1.5-3s，取决于在线传感器数量。

---

## 9. 与 Controlboard 的差异说明

此详情界面与 Controlboard 的本地 DWIN 界面在业务逻辑上相同，但有以下架构差异：

| 项目 | Controlboard | Hostboard 详情（本设计） |
|------|-------------|------------------------|
| 数据来源 | 传感器 Modbus 直连 | 通过 Controlboard FC 0x03 间接读取 |
| 数据更新 | 每次传感器响应后立即触发 | 采集到数据后由接收任务推屏 |
| 图标更新 | 接收任务中逐个更新 | TaskDwinIcons 每 500ms 批量更新 |
| 传感器范围 | 63 路（地址 1-63） | 63 路（仅选中控制器的传感器） |
| 烟雾报警器图标 | 有（0x1444） | 没有（用户指定） |

---

## 10. 边界情况处理

| 场景 | 处理 |
|------|------|
| 选中控制器的线圈数据尚不可用 | 详情图标显示 OFFLINE，等待采集完成 |
| 选中控制器离线 | DetailCollect_Step 检测 `!HostReg_IsOnline()` 跳过传感器采集，图标全部显示 OFFLINE |
| 选中控制器切换 | 调用 `DetailView_Reset()` 清空所有缓冲，状态机重置回 `DETAIL_NEED_COIL` |
| 返回到主界面 | `selectAddr == 0` 自动切回主界面图标更新，状态机回到 `DETAIL_IDLE` |
| 传感器型号尚未采集 | `DetailView_GetType()` 返回 0，`DetailPushToDwin()` 的 `default` 分支不推送 |
| 传感器数据尚未采集 | 不会触发推屏，图标显示 OFFLINE |
