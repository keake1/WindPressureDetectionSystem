/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : modbus_polling.h
  * @brief          : Modbus 轮询任务 — 定时读取 Controlboard 数据
  *
  * 轮询内容：
  *   1. 读离散输入寄存器：获取 Controlboard 的传感器在线/报警状态
  *   2. 读保持寄存器：获取 Controlboard 的传感器类型和数据
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MODBUS_POLLING_H__
#define __MODBUS_POLLING_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported constants --------------------------------------------------------*/

/** @defgroup Poll_Config 轮询配置 */
/** @{ */
#define CONTROLBOARD_ADDR      1            /* Controlboard DIP 硬件地址（可调整） */
#define POLL_INTERVAL_MS       1000         /* 轮询间隔：每秒一次 */
/** @} */

/* USER CODE BEGIN EConst */

/* USER CODE END EConst */

/* Exported functions prototypes ---------------------------------------------*/

void TaskModbusPoll(void *arg);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_POLLING_H__ */
