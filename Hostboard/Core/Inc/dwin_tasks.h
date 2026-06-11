/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : dwin_tasks.h
  * @brief          : USART3 迪文屏发送/接收 FreeRTOS 任务声明
  *
  * TaskDwinTx： 从发送队列取帧 → HAL_UART_Transmit_IT → 等 TX 完成信号量
  * TaskDwinRx： 从接收队列取帧 → 将来解析帧内容
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __DWIN_TASKS_H__
#define __DWIN_TASKS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported functions prototypes ---------------------------------------------*/

void TaskDwinTx(void *arg);
void TaskDwinRx(void *arg);
void TaskDwinIcons(void *arg);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __DWIN_TASKS_H__ */
