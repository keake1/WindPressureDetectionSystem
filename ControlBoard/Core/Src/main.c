/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "controller_maintenance.h"
#include "dwin.h"
#include "global_status.h"
#include "modbus_rtu.h"
#include "modbus_sensor_host.h"
#include "modbus_slave.h"
#include "task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 任务心跳计数：各工作任务每轮循环递增自己的计数，喂狗任务据此判断任务是否仍在运行。 */
static volatile uint32_t g_slaveTaskHeartbeat;
static volatile uint32_t g_sensorTaskHeartbeat;

/* 任务心跳停滞超过该阈值视为任务挂死，停止喂狗等待 IWDG 复位。 */
#define IWDG_TASK_STALE_TIMEOUT_MS 800U
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void ControllerMaintenanceTask(void *argument);
static void ModbusSensorTask(void *argument);
static void ModbusSlaveTask(void *argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



static void ControllerMaintenanceTask(void *argument)
{
  /* maintenanceState 体积较大（~350B），且不可重入。放 static 可让 ctrl_maint 任务栈缩小。 */
  static ControllerMaintenanceState_t maintenanceState;
  uint32_t lastSlaveHeartbeat = 0U;
  uint32_t lastSensorHeartbeat = 0U;
  TickType_t lastSlaveProgressTick;
  TickType_t lastSensorProgressTick;

  (void)argument;
  ControllerMaintenance_Init(&maintenanceState);
  lastSlaveProgressTick = xTaskGetTickCount();
  lastSensorProgressTick = lastSlaveProgressTick;

  for (;;)
  {
    ControllerMaintenance_Process(&maintenanceState);

    /* 喂狗策略：只有当本任务、Modbus 从机任务、传感器任务三方心跳都健康时才喂狗。
     * 启动扫描期间传感器任务可能数秒不返回，此期间跳过其心跳检查。 */
    {
      TickType_t now = xTaskGetTickCount();
      uint32_t slaveHb = g_slaveTaskHeartbeat;
      uint32_t sensorHb = g_sensorTaskHeartbeat;
      uint8_t slaveAlive;
      uint8_t sensorAlive;

      if (slaveHb != lastSlaveHeartbeat)
      {
        lastSlaveHeartbeat = slaveHb;
        lastSlaveProgressTick = now;
      }
      if (sensorHb != lastSensorHeartbeat)
      {
        lastSensorHeartbeat = sensorHb;
        lastSensorProgressTick = now;
      }

      slaveAlive =
        ((now - lastSlaveProgressTick) <= pdMS_TO_TICKS(IWDG_TASK_STALE_TIMEOUT_MS)) ? 1U : 0U;
      sensorAlive = (g_controllerInfo.startupScanDone == 0U)
                      ? 1U
                      : (((now - lastSensorProgressTick) <= pdMS_TO_TICKS(IWDG_TASK_STALE_TIMEOUT_MS))
                           ? 1U
                           : 0U);

      if ((slaveAlive != 0U) && (sensorAlive != 0U))
      {
        (void)HAL_IWDG_Refresh(&hiwdg);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(CONTROLLER_MAINTENANCE_TASK_INTERVAL_MS));
  }
}

static void ModbusSensorTask(void *argument)
{
  uint8_t nextFullScanSensorIndex = 0U;
  uint8_t nextOnlineSensorIndex = 0U;
  uint8_t startupRound;

  (void)argument;

  /* 下行轮询任务：ControlBoard 通过 USART1 依次读取 1~64 号传感器。 */
  for (startupRound = 0U; startupRound < MODBUS_SENSOR_STARTUP_SCAN_ROUNDS; ++startupRound)
  {
    while ((ModbusSensorHost_Poll(MODBUS_SENSOR_POLL_ALL,
                                  &nextFullScanSensorIndex,
                                  MODBUS_SENSOR_COUNT,
                                  0U,
                                  MODBUS_SENSOR_DISCOVERY_RESPONSE_TIMEOUT_MS) &
            MODBUS_SENSOR_POLL_RESULT_ROUND_DONE) == 0U)
    {
    }
  }
  if (g_controllerInfo.errorResponseCount > GLOBAL_STATUS_DUPLICATE_ADDR_ERROR_THRESHOLD)
  {
    g_controllerInfo.hasDuplicateAddressSensor = 1U;
  }
  g_controllerInfo.startupScanDone = 1U;
  nextFullScanSensorIndex = 0U;

  for (;;)
  {
    (void)ModbusSensorHost_Poll(MODBUS_SENSOR_POLL_ALL,
                                &nextFullScanSensorIndex,
                                MODBUS_SENSOR_FULL_SCAN_BATCH_COUNT,
                                MODBUS_SENSOR_NORMAL_REQUEST_INTERVAL_MS,
                                MODBUS_SENSOR_DISCOVERY_RESPONSE_TIMEOUT_MS);
    (void)ModbusSensorHost_Poll(MODBUS_SENSOR_POLL_ONLINE,
                                &nextOnlineSensorIndex,
                                1U,
                                MODBUS_SENSOR_ONLINE_REQUEST_INTERVAL_MS,
                                MODBUS_SENSOR_ONLINE_RESPONSE_TIMEOUT_MS);
    g_sensorTaskHeartbeat++;
    vTaskDelay(pdMS_TO_TICKS(MODBUS_SENSOR_POLL_CYCLE_INTERVAL_MS));
  }
}

static void ModbusSlaveTask(void *argument)
{
  uint8_t request[MODBUS_SLAVE_MAX_ADU_SIZE];
  uint8_t response[MODBUS_SLAVE_MAX_ADU_SIZE];
  uint16_t requestLen;
  uint16_t responseLen;

  (void)argument;

  /* 上行从机任务：ControlBoard 通过 USART2 响应 HostBoard 的 Modbus 请求。 */
  for (;;)
  {
    if (ModbusRtu_PollFrameFrom(&huart2, request, sizeof(request), &requestLen) != 0U)
    {
      responseLen = ModbusSlave_HandleRequest(request, requestLen, response, sizeof(response));
      if (responseLen > 0U)
      {
        (void)ModbusRtu_SendTo(&huart2, response, responseLen, 100U);
      }
    }

    g_slaveTaskHeartbeat++;
    vTaskDelay(pdMS_TO_TICKS(1U));
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  /* USART2 是上行 Modbus 从机端口，USART1 是下行传感器 Modbus 主机端口。 */
  ModbusSlave_InitData();
  if (ModbusRtu_InitPort(&huart2) == 0U)
  {
    Error_Handler();
  }
  if (ModbusSensorHost_Init(&huart1) == 0U)
  {
    Error_Handler();
  }
  if (DwinSend_Init() == 0U)
  {
    Error_Handler();
  }
  if (xTaskCreate(ModbusSlaveTask, "mb_slave", 256U, NULL, tskIDLE_PRIORITY + 1U, NULL) != pdPASS)
  {
    Error_Handler();
  }
  if (xTaskCreate(ModbusSensorTask, "mb_sensors", 320U, NULL, tskIDLE_PRIORITY + 1U, NULL) != pdPASS)
  {
    Error_Handler();
  }
  if (xTaskCreate(ControllerMaintenanceTask, "ctrl_maint", 256U, NULL, tskIDLE_PRIORITY + 1U, NULL) != pdPASS)
  {
    Error_Handler();
  }
  if (xTaskCreate(DwinSendTask, "dwin_tx", 192U, NULL, tskIDLE_PRIORITY + 1U, NULL) != pdPASS)
  {
    Error_Handler();
  }
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
