/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : dwin_tasks.c
  * @brief          : USART3 迪文屏发送/接收 FreeRTOS 任务实现
  *
  * 发送任务 (TaskDwinTx)：
  *   1. 阻塞等待 xDwinTxQueue
  *   2. HAL_UART_Transmit_IT 中断发送
  *   3. 等 xDwinTxCompleteSem（50ms 超时）
  *
  * 接收任务 (TaskDwinRx)：
  *   1. 阻塞等待 xDwinRxQueue（ISR 状态机填充）
  *   2. 将来：解析帧内容
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "dwin_tasks.h"
#include "dwin.h"
#include "usart.h"
#include <string.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ==================== USART3 发送任务 ==================== */

void TaskDwinTx(void *arg)
{
    (void)arg;
    DwinFrame_t frame;

    for (;;)
    {
        /* ---- 1. 等待发送队列 ---- */
        if (xQueueReceive(xDwinTxQueue, &frame, portMAX_DELAY) != pdPASS)
            continue;

        /* ---- 2. 中断发送 ---- */
        HAL_UART_Transmit_IT(&huart3, frame.data, frame.length);

        /* ---- 3. 等待发送完成（50ms 超时） ---- */
        xSemaphoreTake(xDwinTxCompleteSem, pdMS_TO_TICKS(50));
    }
}

/* ==================== USART3 接收任务 ==================== */

/* 全局 DWIN 解析状态 */
DWIN_HostStatus_t g_hostDwinStatus;

void TaskDwinRx(void *arg)
{
    (void)arg;
    DwinFrame_t frame;
    uint8_t cnt_tip = 0;    /* 开机恢复备注进度 */

    for (;;)
    {
        /* ---- 1. 等待完整帧（ISR 状态机组装后入队） ---- */
        if (xQueueReceive(xDwinRxQueue, &frame, portMAX_DELAY) != pdPASS)
            continue;

        uint8_t *buf = frame.data;
        uint16_t len = frame.length;

        /* ---- 2. 校验帧头 ---- */
        if (buf[0] != 0x5A || buf[1] != 0xA5)
            continue;

        /* 只处理 0x83 读变量应答帧 */
        if (len < 7 || buf[3] != 0x83)
            continue;

        uint16_t varAddr = ((uint16_t)buf[4] << 8) | buf[5];

        /* 备注变量区地址修正（迪文屏返回地址比实际少 1）*/
        if (varAddr >= (DWIN_TIP_VAR_ADDR - 1U) &&
            varAddr <  (DWIN_TIP_VAR_ADDR - 1U + DWIN_TIP_SLOT_COUNT * 0x10U))
        {
            varAddr += 1U;
        }

        /* ── 分支1：备注写入（屏幕下发备注内容）── */
        if (varAddr >= DWIN_TIP_VAR_ADDR &&
            varAddr <  DWIN_TIP_VAR_ADDR + DWIN_TIP_SLOT_COUNT * 0x10U)
        {
            if (len < 10U)
                continue;

            uint8_t content_len = buf[8];
            if (content_len == 0U || content_len > DWIN_TIP_CONTENT_MAX_LEN
                || len < (uint16_t)(9U + content_len))
            {
                continue;
            }

            uint16_t slot = (varAddr - DWIN_TIP_VAR_ADDR) / 0x10U;
            if (slot >= DWIN_TIP_SLOT_COUNT)
                continue;

            uint8_t store_buf[DWIN_TIP_BYTES_PER_SLOT];
            memset(store_buf, 0xFFU, sizeof(store_buf));
            store_buf[0] = content_len;
            memcpy(&store_buf[1], &buf[9], content_len);

            uint16_t crc = DWIN_CalcCRC16(store_buf, 1U + content_len);
            store_buf[1U + content_len]      = (uint8_t)(crc >> 8U);
            store_buf[1U + content_len + 1U] = (uint8_t)(crc & 0xFFU);

            uint8_t raw_bytes   = 1U + content_len + 2U;
            uint8_t align_bytes = (raw_bytes + 3U) & (uint8_t)~3U;
            if (align_bytes > DWIN_TIP_BYTES_PER_SLOT)
                align_bytes = DWIN_TIP_BYTES_PER_SLOT;

            DWIN_WriteVar(DWIN_TIP_WRITE_BUF_ADDR, store_buf, align_bytes);
            vTaskDelay(pdMS_TO_TICKS(20));

            uint32_t flash_addr = DWIN_TIP_FLASH_ADDR(slot);
            uint16_t word_len   = (uint16_t)(align_bytes / 2U);
            if (word_len & 1U) { word_len += 1U; }
            DWIN_NorFlashWrite(flash_addr, DWIN_TIP_WRITE_BUF_ADDR, word_len);
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        /* ── 分支2：开机恢复（从 NorFlash 读回的备注帧写入屏幕变量）── */
        else if (varAddr == DWIN_TIP_READ_BUF_ADDR)
        {
            if (len < 9U)
                continue;

            uint8_t content_len = buf[7];
            if (content_len == 0U || content_len > DWIN_TIP_CONTENT_MAX_LEN
                || len < (uint16_t)(7U + 1U + content_len + 2U))
            {
                continue;
            }

            uint16_t crc_calc   = DWIN_CalcCRC16(&buf[7], 1U + content_len);
            uint16_t crc_stored = ((uint16_t)buf[7U + 1U + content_len] << 8U)
                                 | buf[7U + 1U + content_len + 1U];
            if (crc_calc != crc_stored)
                continue;

            uint8_t write_buf[DWIN_TIP_CONTENT_MAX_LEN];
            uint8_t write_len = content_len + 2U;
            if (write_len & 1U) { write_len += 1U; }
            memset(write_buf, 0xFFU, sizeof(write_buf));
            memcpy(write_buf, &buf[8], content_len);

            DWIN_WriteVar((uint16_t)(DWIN_TIP_VAR_ADDR + (uint16_t)cnt_tip * 0x10U),
                          write_buf, write_len);
            cnt_tip++;
        }
        /* ── 分支3：屏幕选中控制器地址 ── */
        else if (varAddr == DWIN_SELECT_CTRL_ADDR)
        {
            /* 5A A5 06 83 addrH addrL 01 dataH dataL → 共 9 字节 */
            if (len >= 9U)
            {
                g_hostDwinStatus.selectAddr = (uint8_t)(((uint16_t)buf[7] << 8U) | buf[8]);
            }
        }
        /* ── 分支4：迪文屏 RTC 时间应答帧（地址 0x0010）── */
        else if (varAddr == 0x0010U && len >= 15U)
        {
            g_hostDwinStatus.rtcYear   = buf[7];
            g_hostDwinStatus.rtcMonth  = buf[8];
            g_hostDwinStatus.rtcDay    = buf[9];
            g_hostDwinStatus.rtcHour   = buf[11];
            g_hostDwinStatus.rtcMinute = buf[12];
            g_hostDwinStatus.rtcSecond = buf[13];
            g_hostDwinStatus.rtcReady  = 1U;
        }
    }
}

/* USER CODE END 0 */
