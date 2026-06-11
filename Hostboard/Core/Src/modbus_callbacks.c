/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : modbus_callbacks.c
  * @brief          : HAL UART TX/RX 完成回调 (Hostboard)
  *
  * USART1 TX 完成回调：释放 xMasterTxCompleteSem 信号量，
  * 通知发送任务可以继续执行。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "uart1_modbus_master.h"
#include "usart.h"

/* USER CODE BEGIN 0 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        BaseType_t xWoken = pdFALSE;
        xSemaphoreGiveFromISR(xMasterTxCompleteSem, &xWoken);
        portYIELD_FROM_ISR(xWoken);
    }
}

/* USER CODE END 0 */
