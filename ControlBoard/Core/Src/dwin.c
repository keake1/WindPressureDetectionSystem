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
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "usart.h"
#include <string.h>

/* USER CODE BEGIN 0 */
#define DWIN_FRAME_MAX_DATA_LEN 8U  /* 当前最长一次写入 4 字节，留 8 字节余量。 */
#define DWIN_QUEUE_LENGTH       16U /* 队列槽数：覆盖一轮 UpdateSensor 的 10 帧 + 状态变化帧。 */
#define DWIN_TX_TIMEOUT_MS      100U

typedef struct
{
  uint16_t addr;
  uint8_t len;
  uint8_t data[DWIN_FRAME_MAX_DATA_LEN];
} DwinFrame_t;

/* DWIN 发送队列：业务任务入队，DwinSendTask 出队后串行发送。 */
static QueueHandle_t g_dwinQueue;
/* UART3 中断发送完成信号量：发送完成或出错时由 HAL 回调释放。 */
static SemaphoreHandle_t g_uart3TxDoneSem;
/* UART3 当前是否有一帧正在发送。 */
static volatile uint8_t g_uart3TxActive;
/* UART3 最近一次中断发送结果。 */
static volatile HAL_StatusTypeDef g_uart3TxStatus;
/* 队列入队失败计数：用于诊断业务侧入队压力是否过大。 */
static volatile uint32_t g_dwinQueueDropCount;

/* 内部串行发送 UART3 一帧；仅 DwinSendTask 调用，因此无需互斥锁。 */
static HAL_StatusTypeDef Dwin_TransmitFrame(const uint8_t *buf, uint16_t len)
{
  HAL_StatusTypeDef status;

  if ((buf == NULL) || (len == 0U) || (g_uart3TxDoneSem == NULL))
  {
    return HAL_ERROR;
  }

  (void)xSemaphoreTake(g_uart3TxDoneSem, 0U);
  g_uart3TxStatus = HAL_BUSY;
  g_uart3TxActive = 1U;
  status = HAL_UART_Transmit_IT(&huart3, (uint8_t *)buf, len);
  if (status != HAL_OK)
  {
    g_uart3TxActive = 0U;
    return status;
  }

  if (xSemaphoreTake(g_uart3TxDoneSem, pdMS_TO_TICKS(DWIN_TX_TIMEOUT_MS)) != pdTRUE)
  {
    g_uart3TxActive = 0U;
    /* 超时时强制结束硬件发送，避免下一帧叠加在未完成的传输上。 */
    (void)HAL_UART_AbortTransmit(&huart3);
    return HAL_TIMEOUT;
  }

  return g_uart3TxStatus;
}

uint8_t DwinSend_Init(void)
{
  if (g_uart3TxDoneSem == NULL)
  {
    g_uart3TxDoneSem = xSemaphoreCreateBinary();
  }

  if (g_dwinQueue == NULL)
  {
    g_dwinQueue = xQueueCreate(DWIN_QUEUE_LENGTH, sizeof(DwinFrame_t));
  }

  g_uart3TxActive = 0U;
  g_uart3TxStatus = HAL_OK;
  return ((g_uart3TxDoneSem != NULL) && (g_dwinQueue != NULL)) ? 1U : 0U;
}

uint32_t DwinSend_GetDropCount(void)
{
  return g_dwinQueueDropCount;
}

void DwinSendTask(void *argument)
{
  DwinFrame_t frame;
  uint8_t buf[3U + 1U + 2U + DWIN_FRAME_MAX_DATA_LEN]; /* 帧头2 + LEN1 + 指令1 + 地址2 + data */

  (void)argument;

  for (;;)
  {
    if (xQueueReceive(g_dwinQueue, &frame, portMAX_DELAY) != pdTRUE)
    {
      continue;
    }

    if ((frame.len == 0U) || (frame.len > DWIN_FRAME_MAX_DATA_LEN))
    {
      continue;
    }

    buf[0] = 0x5AU;
    buf[1] = 0xA5U;
    buf[2] = (uint8_t)(3U + frame.len);   /* LEN = 指令1 + 地址2 + 数据 */
    buf[3] = 0x82U;                        /* 写变量指令 */
    buf[4] = (uint8_t)(frame.addr >> 8);   /* 地址高字节 */
    buf[5] = (uint8_t)(frame.addr & 0xFFU);/* 地址低字节 */
    memcpy(&buf[6], frame.data, frame.len);

    (void)Dwin_TransmitFrame(buf, (uint16_t)(6U + frame.len));
  }
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
    DwinFrame_t frame;

    if ((pData == NULL) || (data_len == 0U) || (data_len > DWIN_FRAME_MAX_DATA_LEN) ||
        (g_dwinQueue == NULL))
    {
        return;
    }

    frame.addr = addr;
    frame.len = data_len;
    memcpy(frame.data, pData, data_len);

    /* 业务任务非阻塞入队；队列满则丢弃本帧，下一轮 maintenance 周期会重发。 */
    if (xQueueSend(g_dwinQueue, &frame, 0U) != pdTRUE)
    {
        g_dwinQueueDropCount++;
    }
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

void DwinValue_UpdateSensor(uint8_t sensorIndex)
{
  /* 按传感器型号只发送相关数据项，避免对所有屏幕变量都做无意义写入：
   *  - 风压传感器：仅风压 (float)
   *  - CO 传感器：仅 CO (float, ppm = 原始值/100)
   *  - 余压传感器：仅余压 (uint16)
   *  - 七合一传感器：CO2/CH2O/TVOC/PM2.5/PM10/温度/湿度
   * 这样 UART3 在屏幕刷新阶段的流量降到原先 1/3~1/10，屏幕响应也更快。 */
  uint16_t baseAddr;
  SensorModel_t model;

  if (sensorIndex >= GLOBAL_STATUS_SENSOR_COUNT)
  {
    return;
  }

  baseAddr = (uint16_t)(DWIN_SENSOR_DATA_ADDR + ((uint16_t)sensorIndex * DWIN_SENSOR_DATA_STRIDE));
  model = (SensorModel_t)SENSOR_MODEL(sensorIndex);

  switch (model)
  {
    case SENSOR_MODEL_WIND_PRESSURE:
      DwinValue_WriteFloat((uint16_t)(baseAddr + DWIN_SENSOR_DATA_WIND_OFFSET),
                           (float)SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_WIND_PRESSURE));
      break;

    case SENSOR_MODEL_CO:
      DwinValue_WriteFloat((uint16_t)(baseAddr + DWIN_SENSOR_DATA_CO_OFFSET),
                           (float)SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_CO) / 100.0f);
      break;

    case SENSOR_MODEL_RESIDUAL_PRESSURE:
      DwinValue_WriteU16((uint16_t)(baseAddr + DWIN_SENSOR_DATA_RESIDUAL_OFFSET),
                         SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_RESIDUAL_PRESSURE));
      break;

    case SENSOR_MODEL_SEVEN_IN_ONE:
      DwinValue_WriteU16((uint16_t)(baseAddr + DWIN_SENSOR_DATA_CO2_OFFSET),
                         SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_CO2));
      DwinValue_WriteU16((uint16_t)(baseAddr + DWIN_SENSOR_DATA_CH2O_OFFSET),
                         SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_CH2O));
      DwinValue_WriteU16((uint16_t)(baseAddr + DWIN_SENSOR_DATA_TVOC_OFFSET),
                         SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_TVOC));
      DwinValue_WriteFloat((uint16_t)(baseAddr + DWIN_SENSOR_DATA_PM25_OFFSET),
                           (float)SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_PM25));
      DwinValue_WriteFloat((uint16_t)(baseAddr + DWIN_SENSOR_DATA_PM10_OFFSET),
                           (float)SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_PM10));
      DwinValue_WriteFloat((uint16_t)(baseAddr + DWIN_SENSOR_DATA_TEMP_OFFSET),
                           (float)SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_TEMPERATURE) / 10.0f);
      DwinValue_WriteFloat((uint16_t)(baseAddr + DWIN_SENSOR_DATA_HUMI_OFFSET),
                           (float)SENSOR_DATA(sensorIndex, GLOBAL_STATUS_DATA_HUMIDITY) / 10.0f);
      break;

    default:
      /* 未知/未识别型号不刷屏，避免向屏幕写入垃圾数据。 */
      break;
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
