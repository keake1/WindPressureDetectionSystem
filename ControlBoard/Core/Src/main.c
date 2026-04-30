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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
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
#define CONTROLLER_ALARM_UPDATE_INTERVAL_MS 500U
#define ALARM_SET_CO2_THRESHOLD             1000U
#define ALARM_SET_CH2O_THRESHOLD            80U
#define ALARM_SET_TVOC_THRESHOLD            600U
#define ALARM_SET_PM25_THRESHOLD            35U
#define ALARM_SET_PM10_THRESHOLD            50U
#define ALARM_SET_CO_X100_THRESHOLD         2500U
#define ALARM_CLEAR_CO2_THRESHOLD           800U
#define ALARM_CLEAR_CH2O_THRESHOLD          70U
#define ALARM_CLEAR_TVOC_THRESHOLD          500U
#define ALARM_CLEAR_PM25_THRESHOLD          25U
#define ALARM_CLEAR_PM10_THRESHOLD          40U
#define ALARM_CLEAR_CO_X100_THRESHOLD       2000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t ControllerAlarm_CalcSensorAlarm(const SensorInfo_t *sensor);
static void ControllerAlarmTask(void *argument);
static void ControllerStatusTask(void *argument);
static void ModbusSensorOnlineTask(void *argument);
static void ModbusSensorHostTask(void *argument);
static void ModbusSlaveTask(void *argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



static void ControllerAlarmTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    uint8_t i;
    uint8_t controllerAlarm = 0U;

    if (g_controllerInfo.startupScanDone == 0U)
    {
      vTaskDelay(pdMS_TO_TICKS(100U));
      continue;
    }

    for (i = 0U; i < GLOBAL_STATUS_SENSOR_COUNT; ++i)
    {
      uint8_t sensorAlarm = ControllerAlarm_CalcSensorAlarm(&g_sensorInfos[i]);
      *g_sensorInfos[i].alarm = sensorAlarm;
      if (sensorAlarm != 0U)
      {
        controllerAlarm = 1U;
      }
    }

    g_controllerInfo.alarm = controllerAlarm;
    vTaskDelay(pdMS_TO_TICKS(CONTROLLER_ALARM_UPDATE_INTERVAL_MS));
  }
}

static void ModbusSensorHostTask(void *argument)
{
  (void)argument;

  /* 下行轮询任务：ControlBoard 通过 USART1 依次读取 1~64 号传感器。 */
  ModbusSensorHost_PollAll(MODBUS_SENSOR_STARTUP_SCAN_ROUNDS,
                           MODBUS_SENSOR_FAST_REQUEST_INTERVAL_MS,
                           MODBUS_SENSOR_DISCOVERY_RESPONSE_TIMEOUT_MS);
  if (g_controllerInfo.errorResponseCount > GLOBAL_STATUS_DUPLICATE_ADDR_ERROR_THRESHOLD)
  {
    g_controllerInfo.hasDuplicateAddressSensor = 1U;
  }
  g_controllerInfo.startupScanDone = 1U;

  for (;;)
  {
    ModbusSensorHost_PollAll(1U,
                             MODBUS_SENSOR_NORMAL_REQUEST_INTERVAL_MS,
                             MODBUS_SENSOR_DISCOVERY_RESPONSE_TIMEOUT_MS);
  }
}

static void ControllerStatusTask(void *argument)
{
  uint32_t lastErrorResponseCount = 0U;
  uint8_t baselineReady = 0U;

  (void)argument;

  for (;;)
  {
    if (g_controllerInfo.startupScanDone == 0U)
    {
      vTaskDelay(pdMS_TO_TICKS(500U));
      continue;
    }

    if (baselineReady == 0U)
    {
      lastErrorResponseCount = g_controllerInfo.errorResponseCount;
      baselineReady = 1U;
    }

    g_controllerInfo.hasZeroAddressSensor =
      ModbusSensorHost_ProbeZeroAddress(MODBUS_SENSOR_ZERO_ADDR_RESPONSE_TIMEOUT_MS);

    {
      uint32_t currentErrorResponseCount = g_controllerInfo.errorResponseCount;
      uint32_t errorIncrease = currentErrorResponseCount - lastErrorResponseCount;
      g_controllerInfo.hasDuplicateAddressSensor =
        (errorIncrease > GLOBAL_STATUS_DUPLICATE_ADDR_ERROR_THRESHOLD) ? 1U : 0U;
      lastErrorResponseCount = currentErrorResponseCount;
    }

    vTaskDelay(pdMS_TO_TICKS(GLOBAL_STATUS_UPDATE_INTERVAL_MS));
  }
}

static void ModbusSensorOnlineTask(void *argument)
{
  (void)argument;

  /* 在线传感器补充轮询：在线数量少时，提高已有数据的刷新频率。 */
  for (;;)
  {
    if (ModbusSensorHost_PollOnline(MODBUS_SENSOR_ONLINE_REQUEST_INTERVAL_MS,
                                    MODBUS_SENSOR_ONLINE_RESPONSE_TIMEOUT_MS) == 0U)
    {
      vTaskDelay(pdMS_TO_TICKS(MODBUS_SENSOR_ONLINE_IDLE_INTERVAL_MS));
    }
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
  (void)xTaskCreate(ModbusSlaveTask, "mb_slave", 256U, NULL, tskIDLE_PRIORITY + 1U, NULL);
  (void)xTaskCreate(ModbusSensorHostTask, "mb_sensors", 384U, NULL, tskIDLE_PRIORITY + 1U, NULL);
  (void)xTaskCreate(ModbusSensorOnlineTask, "mb_online", 384U, NULL, tskIDLE_PRIORITY + 1U, NULL);
  (void)xTaskCreate(ControllerStatusTask, "ctrl_status", 256U, NULL, tskIDLE_PRIORITY + 1U, NULL);
  (void)xTaskCreate(ControllerAlarmTask, "ctrl_alarm", 256U, NULL, tskIDLE_PRIORITY + 1U, NULL);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  __disable_irq();
  while (1)
  {
  }
}

static uint8_t ControllerAlarm_CalcSensorAlarm(const SensorInfo_t *sensor)
{
  uint8_t nowAlarm;

  if ((sensor == NULL) ||
      (*sensor->online == 0U))
  {
    return 0U;
  }

  if ((*sensor->model != SENSOR_MODEL_CO) &&
      (*sensor->model != SENSOR_MODEL_SEVEN_IN_ONE))
  {
    return 0U;
  }

  nowAlarm = *sensor->alarm;
  if (nowAlarm == 0U)
  {
    /* 当前正常：CO 数据按实际 ppm * 100 保存，阈值也使用 x100 单位。 */
    if (*sensor->model == SENSOR_MODEL_CO)
    {
      if (sensor->data[GLOBAL_STATUS_DATA_CO] > ALARM_SET_CO_X100_THRESHOLD)
      {
        nowAlarm = 1U;
      }
    }
    else if (*sensor->model == SENSOR_MODEL_SEVEN_IN_ONE)
    {
      if ((sensor->data[GLOBAL_STATUS_DATA_CO2] > ALARM_SET_CO2_THRESHOLD) ||
          (sensor->data[GLOBAL_STATUS_DATA_CH2O] > ALARM_SET_CH2O_THRESHOLD) ||
          (sensor->data[GLOBAL_STATUS_DATA_TVOC] > ALARM_SET_TVOC_THRESHOLD) ||
          (sensor->data[GLOBAL_STATUS_DATA_PM25] > ALARM_SET_PM25_THRESHOLD) ||
          (sensor->data[GLOBAL_STATUS_DATA_PM10] > ALARM_SET_PM10_THRESHOLD))
      {
        nowAlarm = 1U;
      }
    }
  }
  else
  {
    /* 当前报警：CO 传感器按 x100 单位恢复，七合一传感器需所有参与项均低于恢复下限。 */
    if (*sensor->model == SENSOR_MODEL_CO)
    {
      if (sensor->data[GLOBAL_STATUS_DATA_CO] < ALARM_CLEAR_CO_X100_THRESHOLD)
      {
        nowAlarm = 0U;
      }
    }
    else if (*sensor->model == SENSOR_MODEL_SEVEN_IN_ONE)
    {
      if ((sensor->data[GLOBAL_STATUS_DATA_CO2] < ALARM_CLEAR_CO2_THRESHOLD) &&
          (sensor->data[GLOBAL_STATUS_DATA_CH2O] < ALARM_CLEAR_CH2O_THRESHOLD) &&
          (sensor->data[GLOBAL_STATUS_DATA_TVOC] < ALARM_CLEAR_TVOC_THRESHOLD) &&
          (sensor->data[GLOBAL_STATUS_DATA_PM25] < ALARM_CLEAR_PM25_THRESHOLD) &&
          (sensor->data[GLOBAL_STATUS_DATA_PM10] < ALARM_CLEAR_PM10_THRESHOLD))
      {
        nowAlarm = 0U;
      }
    }
  }

  return nowAlarm;
}
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
