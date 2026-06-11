/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : modbus_master_tasks.c
  * @brief          : Modbus RTU 主站发送/接收任务实现
  *
  * 发送任务 (TaskModbusSend)：
  *   1. 阻塞等待 xMasterSendQueue
  *   2. 构造 Modbus 帧（含 CRC），HAL_UART_Transmit 发送
  *   3. 打开接收窗口 → 等信号量（30ms 超时）→ 关闭接收窗口
  *   4. 3.5 字符帧间隔
  *
  * 接收任务 (TaskModbusRecv)：
  *   1. 阻塞等待 xMasterRawRxQueue（IDLE 中断填充）
  *   2. CRC 校验
  *   3. 解析帧（验证地址和功能码）
  *   4. 释放 xMasterRxSem 通知发送任务
  *
  * 参考 Controlboard/Core/Src/modbus_tasks.c 架构。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "modbus_master_tasks.h"
#include "uart1_modbus_master.h"
#include "hostboard_registers.h"
#include "usart.h"

/* USER CODE BEGIN 0 */

/* 3.5 字符时间 @ 9600 8N1 ≈ 3.65 ms → 4 ticks */
#define MODBUS_3_5_CHAR_TICKS  pdMS_TO_TICKS(4)

/* 等待响应超时 */
#define MODBUS_RESP_TIMEOUT    pdMS_TO_TICKS(30)

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==================== 发送任务 ==================== */

void TaskModbusSend(void *arg)
{
    (void)arg;
    ModbusMasterRequest_t req;
    uint8_t tx_buf[8];
    uint16_t crc;

    for (;;)
    {
        /* ---- 1. 等待发送队列有请求 ---- */
        if (xQueueReceive(xMasterSendQueue, &req, portMAX_DELAY) != pdPASS)
            continue;

        /* 清除可能残留的信号量（上次超时后接收任务可能仍会 Give） */
        xSemaphoreTake(xMasterRxSem, 0);

        /* 记录当前请求信息（供接收任务解析响应数据的位置） */
        g_host_last_req.slave_addr = req.slave_addr;
        g_host_last_req.func_code  = req.func_code;
        g_host_last_req.reg_addr   = req.reg_addr;
        g_host_last_req.reg_count  = req.reg_value;

        /* ---- 3. 构造 Modbus RTU 帧 (8 字节) ---- */
        tx_buf[0] = req.slave_addr;
        tx_buf[1] = req.func_code;
        tx_buf[2] = (uint8_t)(req.reg_addr >> 8);
        tx_buf[3] = (uint8_t)(req.reg_addr & 0xFF);
        tx_buf[4] = (uint8_t)(req.reg_value >> 8);
        tx_buf[5] = (uint8_t)(req.reg_value & 0xFF);

        crc = ModbusMaster_CRC16(tx_buf, 6);
        tx_buf[6] = (uint8_t)(crc & 0xFF);
        tx_buf[7] = (uint8_t)(crc >> 8);

        /* ---- 4. 中断发送（发送完成后再开接收窗，避免自回声触发假 IDLE） ---- */
        HAL_UART_Transmit_IT(&huart1, tx_buf, 8);
        xSemaphoreTake(xMasterTxCompleteSem, pdMS_TO_TICKS(50));

        /* ---- 5. 发送完成 → 开启接收窗口，等待响应 ---- */
        ModbusMaster_StartRx();
        xSemaphoreTake(xMasterRxSem, MODBUS_RESP_TIMEOUT);

        /* ---- 6. 关闭接收中断窗口 ---- */
        ModbusMaster_DisableRx();

        /* ---- 7. 帧间隔：3.5 字符时间 ---- */
        vTaskDelay(MODBUS_3_5_CHAR_TICKS);
    }
}

/* ==================== 接收任务 ==================== */

void TaskModbusRecv(void *arg)
{
    (void)arg;
    ModbusMasterFrame_t raw;

    for (;;)
    {
        /* ---- 1. 等待原始帧（IDLE 中断放入队列） ---- */
        if (ModbusMaster_DequeueRawFrame(&raw, portMAX_DELAY) != 0)
            continue;

        /* ---- 2. CRC 校验 ---- */
        uint8_t crc_ok = 0;
        if (raw.length >= 4)
        {
            uint16_t calc_crc = ModbusMaster_CRC16(raw.frame, raw.length - 2);
            uint16_t recv_crc = (uint16_t)raw.frame[raw.length - 2] |
                                ((uint16_t)raw.frame[raw.length - 1] << 8);
            crc_ok = (calc_crc == recv_crc);
        }

        if (crc_ok)
        {
            uint8_t  slave_addr = raw.frame[0];
            uint8_t  func_code  = raw.frame[1];
            uint8_t  byte_cnt   = raw.frame[2];

            /* 地址 0：任何 CRC 有效的响应均视为零地址存在 */
            if (slave_addr == 0)
            {
                HostReg_RecordZeroAddrResponse();
            }
            /* 地址 1-128：验证是 FC 0x02 响应且数据长度合理后存入寄存器 */
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
                                             g_host_last_req.reg_count,
                                             &raw.frame[3]);
                }
                /* else: 其他长度 → 忽略（不应发生） */
            }
            /* else: 功能码异常或格式错误 → 忽略不计 */

            /* ---- 3. 仅 CRC 通过时释放信号量：通知发送任务可以发下一帧 ---- */
            xSemaphoreGive(xMasterRxSem);
        }
        else
        {
            /* CRC 失败且收到过数据 → 计入重复地址检测 */
            if (raw.length > 0)
                HostReg_RecordError();
        }
    }
}

/* USER CODE END 0 */
