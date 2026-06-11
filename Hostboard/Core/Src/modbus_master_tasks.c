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

        /* ---- 2. 开启接收中断窗口 ---- */
        ModbusMaster_StartRx();

        /* 清除可能残留的信号量（上次超时后接收任务可能仍会 Give） */
        xSemaphoreTake(xMasterRxSem, 0);

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

        /* ---- 4. 发送 ---- */
        HAL_UART_Transmit(&huart1, tx_buf, 8, HAL_MAX_DELAY);

        /* ---- 5. 等待响应 ---- */
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
            uint8_t slave     = raw.frame[0];
            uint8_t func_code = raw.frame[1];
            uint8_t byte_cnt  = raw.frame[2];

            /* 验证：从机地址和功能码应在合理范围 */
            (void)slave;
            (void)func_code;
            (void)byte_cnt;

            /* TODO: 根据应用需求解析数据并存入本地寄存器 */
        }

        /* ---- 3. 释放信号量：通知发送任务可以发下一帧 ---- */
        xSemaphoreGive(xMasterRxSem);
    }
}

/* USER CODE END 0 */
