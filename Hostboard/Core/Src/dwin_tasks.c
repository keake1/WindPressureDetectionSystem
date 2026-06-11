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

void TaskDwinRx(void *arg)
{
    (void)arg;
    DwinFrame_t frame;

    for (;;)
    {
        /* ---- 1. 等待完整帧（ISR 状态机组装后入队） ---- */
        if (xQueueReceive(xDwinRxQueue, &frame, portMAX_DELAY) != pdPASS)
            continue;

        /* ---- 2. 将来：解析帧内容 ----
         * 根据 frame.data[3] (CMD) 分发：
         *   0x83 → 读应答（RTC 时间、传感器数据等）
         *   0x82 → 写应答确认
         */
    }
}

/* USER CODE END 0 */
