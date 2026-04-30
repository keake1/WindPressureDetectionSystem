/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dwin.c
  * @brief   迪文屏通信接口实现
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "dwin.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "usart.h"
#include <string.h>

/* USER CODE BEGIN 0 */
/* 迪文屏 UART3 发送互斥锁：多个任务同时发送时按调用顺序串行发送。 */
static SemaphoreHandle_t g_uart3TxMutex;
/* UART3 中断发送完成信号量：发送完成或出错时由 HAL 回调释放。 */
static SemaphoreHandle_t g_uart3TxDoneSem;
/* UART3 当前是否有一帧正在发送。 */
static volatile uint8_t g_uart3TxActive;
/* UART3 最近一次中断发送结果。 */
static volatile HAL_StatusTypeDef g_uart3TxStatus;

uint8_t UART3_SendInit(void)
{
  if (g_uart3TxMutex == NULL)
  {
    g_uart3TxMutex = xSemaphoreCreateMutex();
  }

  if (g_uart3TxDoneSem == NULL)
  {
    g_uart3TxDoneSem = xSemaphoreCreateBinary();
  }

  g_uart3TxActive = 0U;
  g_uart3TxStatus = HAL_OK;
  return ((g_uart3TxMutex != NULL) && (g_uart3TxDoneSem != NULL)) ? 1U : 0U;
}

HAL_StatusTypeDef UART3_Send(const uint8_t *buf, uint16_t len)
{
  HAL_StatusTypeDef status;

  if ((buf == NULL) || (len == 0U) || (UART3_SendInit() == 0U))
  {
    return HAL_ERROR;
  }

  (void)xSemaphoreTake(g_uart3TxMutex, portMAX_DELAY);
  (void)xSemaphoreTake(g_uart3TxDoneSem, 0U);

  do
  {
    g_uart3TxStatus = HAL_BUSY;
    g_uart3TxActive = 1U;
    status = HAL_UART_Transmit_IT(&huart3, (uint8_t *)buf, len);
    if (status == HAL_BUSY)
    {
      g_uart3TxActive = 0U;
      vTaskDelay(pdMS_TO_TICKS(1U));
    }
  } while (status == HAL_BUSY);

  if (status == HAL_OK)
  {
    (void)xSemaphoreTake(g_uart3TxDoneSem, portMAX_DELAY);
    status = g_uart3TxStatus;
  }
  else
  {
    g_uart3TxActive = 0U;
  }

  if (status != HAL_OK)
  {
    (void)HAL_UART_AbortTransmit(&huart3);
  }

  (void)xSemaphoreGive(g_uart3TxMutex);
  return status;
}

void UART3_SendTxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if ((huart->Instance == USART3) && (g_uart3TxActive != 0U) && (g_uart3TxDoneSem != NULL))
  {
    g_uart3TxStatus = HAL_OK;
    g_uart3TxActive = 0U;
    (void)xSemaphoreGiveFromISR(g_uart3TxDoneSem, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
  }
}

void UART3_SendErrorCallback(UART_HandleTypeDef *huart)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if ((huart->Instance == USART3) && (g_uart3TxActive != 0U) && (g_uart3TxDoneSem != NULL))
  {
    g_uart3TxStatus = HAL_ERROR;
    g_uart3TxActive = 0U;
    (void)xSemaphoreGiveFromISR(g_uart3TxDoneSem, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
  }
}

/* USER CODE END 0 */

/**
 * @brief 向迪文屏指定变量地址写入任意长度数据（0x82 指令）
 *
 * 帧格式：
 *   5A A5 [LEN] 82 [AH] [AL] [D0] [D1] ... [Dn]
 *   LEN = 3 + data_len（指令码1 + 地址2 + 数据n）
 *
 * 示例：
 *   写 1 个 word(2字节) → 5A A5 05 82 10 00 00 02
 *   写 2 个 word(4字节) → 5A A5 07 82 10 01 00 0A 00 0B
 *
 * @param addr      目标变量起始地址
 * @param pData     待写入的原始字节数据（调用方负责大端序）
 * @param data_len  数据字节数（最大 0xF9，即 LEN 上限 0xFF - 3 固定字节）
 */
void DWIN_WriteVar(uint16_t addr, const uint8_t *pData, uint8_t data_len)
{
    /* 当前项目中最大实际发送长度极小，
     * 将其限制为 64 字节非常安全，并可节省约 200 字节的任务栈空间 */
    if (pData == NULL || data_len == 0 || data_len > 64)
    {
        return;
    }

    /* 总帧长 = 帧头(2) + LEN字段(1) + LEN内容(3+data_len) */
    uint8_t buf[3 + 1 + 2 + 64];          /* 优化：减小最大帧缓冲限制以防止栈溢出 */
    buf[0] = 0x5A;
    buf[1] = 0xA5;
    buf[2] = 3 + data_len;                  /* LEN */
    buf[3] = 0x82;                          /* 写变量指令 */
    buf[4] = (uint8_t)(addr >> 8);          /* 地址高字节 */
    buf[5] = (uint8_t)(addr & 0xFF);        /* 地址低字节 */
    memcpy(&buf[6], pData, data_len);       /* 数据段 */

    (void)UART3_Send(buf, 6U + data_len);
}

void DwinIcon_WriteValue(uint16_t addr, uint8_t value)
{
  uint8_t data[2];

  data[0] = 0U;
  data[1] = value;
  DWIN_WriteVar(addr, data, sizeof(data));
}

uint8_t DwinIcon_GetSensorStatusValue(uint8_t online, uint8_t alarm)
{
  if (online == 0U)
  {
    return 0U;
  }

  return (alarm != 0U) ? 1U : 2U;
}

void DwinValue_WriteU16(uint16_t addr, uint16_t value)
{
  uint8_t data[2];

  data[0] = (uint8_t)(value >> 8);
  data[1] = (uint8_t)value;
  DWIN_WriteVar(addr, data, sizeof(data));
}

void DwinValue_WriteFloat(uint16_t addr, float value)
{
  uint32_t raw;
  uint8_t data[4];

  memcpy(&raw, &value, sizeof(raw));
  data[0] = (uint8_t)(raw >> 24);
  data[1] = (uint8_t)(raw >> 16);
  data[2] = (uint8_t)(raw >> 8);
  data[3] = (uint8_t)raw;
  DWIN_WriteVar(addr, data, sizeof(data));
}

void DwinValue_UpdateSensor(uint8_t sensorIndex, const uint16_t *data, uint8_t forceUpdate)
{
  uint16_t baseAddr = (uint16_t)(DWIN_SENSOR_DATA_ADDR + ((uint16_t)sensorIndex * DWIN_SENSOR_DATA_STRIDE));

  if ((data[GLOBAL_STATUS_DATA_CO] != g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_CO]) || (forceUpdate != 0U))
  {
    float co = (float)g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_CO] / 100.0f;
    DwinValue_WriteFloat((uint16_t)(baseAddr + DWIN_SENSOR_DATA_CO_OFFSET), co);
  }

  if ((data[GLOBAL_STATUS_DATA_WIND_PRESSURE] != g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_WIND_PRESSURE]) || (forceUpdate != 0U))
  {
    float windPressure = (float)g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_WIND_PRESSURE];
    DwinValue_WriteFloat((uint16_t)(baseAddr + DWIN_SENSOR_DATA_WIND_OFFSET), windPressure);
  }

  if ((data[GLOBAL_STATUS_DATA_RESIDUAL_PRESSURE] != g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_RESIDUAL_PRESSURE]) || (forceUpdate != 0U))
  {
    DwinValue_WriteU16((uint16_t)(baseAddr + DWIN_SENSOR_DATA_RESIDUAL_OFFSET),
                       g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_RESIDUAL_PRESSURE]);
  }

  if ((data[GLOBAL_STATUS_DATA_CO2] != g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_CO2]) || (forceUpdate != 0U))
  {
    DwinValue_WriteU16((uint16_t)(baseAddr + DWIN_SENSOR_DATA_CO2_OFFSET),
                       g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_CO2]);
  }

  if ((data[GLOBAL_STATUS_DATA_CH2O] != g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_CH2O]) || (forceUpdate != 0U))
  {
    DwinValue_WriteU16((uint16_t)(baseAddr + DWIN_SENSOR_DATA_CH2O_OFFSET),
                       g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_CH2O]);
  }

  if ((data[GLOBAL_STATUS_DATA_TVOC] != g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_TVOC]) || (forceUpdate != 0U))
  {
    DwinValue_WriteU16((uint16_t)(baseAddr + DWIN_SENSOR_DATA_TVOC_OFFSET),
                       g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_TVOC]);
  }

  if ((data[GLOBAL_STATUS_DATA_PM25] != g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_PM25]) || (forceUpdate != 0U))
  {
    float pm25 = (float)g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_PM25];
    DwinValue_WriteFloat((uint16_t)(baseAddr + DWIN_SENSOR_DATA_PM25_OFFSET), pm25);
  }

  if ((data[GLOBAL_STATUS_DATA_PM10] != g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_PM10]) || (forceUpdate != 0U))
  {
    float pm10 = (float)g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_PM10];
    DwinValue_WriteFloat((uint16_t)(baseAddr + DWIN_SENSOR_DATA_PM10_OFFSET), pm10);
  }

  if ((data[GLOBAL_STATUS_DATA_TEMPERATURE] != g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_TEMPERATURE]) || (forceUpdate != 0U))
  {
    float temperature = (float)g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_TEMPERATURE] / 10.0f;
    DwinValue_WriteFloat((uint16_t)(baseAddr + DWIN_SENSOR_DATA_TEMP_OFFSET), temperature);
  }

  if ((data[GLOBAL_STATUS_DATA_HUMIDITY] != g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_HUMIDITY]) || (forceUpdate != 0U))
  {
    float humidity = (float)g_sensorInfos[sensorIndex].data[GLOBAL_STATUS_DATA_HUMIDITY] / 10.0f;
    DwinValue_WriteFloat((uint16_t)(baseAddr + DWIN_SENSOR_DATA_HUMI_OFFSET), humidity);
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
