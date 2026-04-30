#include "modbus_rtu.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <string.h>

#define MODBUS_RTU_FRAME_GAP_MS 5U

static UART_HandleTypeDef *g_uart;
static SemaphoreHandle_t g_txMutex;
static SemaphoreHandle_t g_txDoneSem;
static uint8_t g_rxByte;
static volatile uint8_t g_receiving;
static volatile uint8_t g_overflow;
static volatile uint8_t g_txActive;
static volatile HAL_StatusTypeDef g_txStatus;
static volatile uint16_t g_rxLen;
static volatile uint32_t g_lastRxTick;
static volatile uint32_t g_lastTxDoneTick;
static uint8_t g_rxBuffer[MODBUS_RTU_MAX_FRAME_SIZE];

static TickType_t ModbusRtu_MsToTicksCeil(uint32_t ms)
{
  TickType_t ticks = pdMS_TO_TICKS(ms);
  return (ticks > 0U) ? ticks : 1U;
}

static void ModbusRtu_StartReceive(void)
{
  if (g_uart == NULL)
  {
    return;
  }

  (void)HAL_UART_Receive_IT(g_uart, &g_rxByte, 1U);
}

static void ModbusRtu_ResetRx(void)
{
  g_receiving = 0U;
  g_overflow = 0U;
  g_rxLen = 0U;
}

static uint8_t ModbusRtu_FrameReady(void)
{
  uint32_t elapsed;

  if (g_receiving == 0U)
  {
    return 0U;
  }

  elapsed = HAL_GetTick() - g_lastRxTick;
  return (elapsed >= MODBUS_RTU_FRAME_GAP_MS) ? 1U : 0U;
}

static void ModbusRtu_WaitTxFrameGap(void)
{
  while ((HAL_GetTick() - g_lastTxDoneTick) < MODBUS_RTU_FRAME_GAP_MS)
  {
    vTaskDelay(pdMS_TO_TICKS(1U));
  }
}

void ModbusRtu_Init(UART_HandleTypeDef *uart)
{
  g_uart = uart;
  g_txMutex = xSemaphoreCreateMutex();
  g_txDoneSem = xSemaphoreCreateBinary();
  g_txActive = 0U;
  g_txStatus = HAL_OK;
  g_lastTxDoneTick = HAL_GetTick() - MODBUS_RTU_FRAME_GAP_MS;
  ModbusRtu_ResetRx();
  ModbusRtu_StartReceive();
}

uint8_t ModbusRtu_PollFrame(uint8_t *frame, uint16_t frameMax, uint16_t *frameLen)
{
  uint16_t copyLen;
  uint8_t overflow;

  if ((frame == NULL) || (frameLen == NULL) || (ModbusRtu_FrameReady() == 0U))
  {
    return 0U;
  }

  taskENTER_CRITICAL();
  copyLen = g_rxLen;
  overflow = g_overflow;
  if (copyLen > frameMax)
  {
    copyLen = frameMax;
  }
  memcpy(frame, g_rxBuffer, copyLen);
  ModbusRtu_ResetRx();
  taskEXIT_CRITICAL();

  *frameLen = copyLen;
  return (overflow == 0U) ? 1U : 0U;
}

HAL_StatusTypeDef ModbusRtu_Send(const uint8_t *frame, uint16_t frameLen, uint32_t timeoutMs)
{
  HAL_StatusTypeDef status;

  if ((g_uart == NULL) || (g_txMutex == NULL) || (g_txDoneSem == NULL) || (frame == NULL) || (frameLen == 0U))
  {
    return HAL_ERROR;
  }

  (void)xSemaphoreTake(g_txMutex, portMAX_DELAY);
  (void)xSemaphoreTake(g_txDoneSem, 0U);

  ModbusRtu_WaitTxFrameGap();

  g_txStatus = HAL_BUSY;
  g_txActive = 1U;
  status = HAL_UART_Transmit_IT(g_uart, (uint8_t *)frame, frameLen);
  if (status != HAL_OK)
  {
    g_txActive = 0U;
    (void)xSemaphoreGive(g_txMutex);
    return status;
  }

  if (xSemaphoreTake(g_txDoneSem, ModbusRtu_MsToTicksCeil(timeoutMs)) != pdTRUE)
  {
    g_txActive = 0U;
    (void)HAL_UART_AbortTransmit(g_uart);
    (void)xSemaphoreGive(g_txMutex);
    return HAL_TIMEOUT;
  }

  status = g_txStatus;
  (void)xSemaphoreGive(g_txMutex);
  return status;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if ((g_uart != NULL) && (huart->Instance == g_uart->Instance) && (g_txActive != 0U))
  {
    g_lastTxDoneTick = HAL_GetTick();
    g_txStatus = HAL_OK;
    g_txActive = 0U;
    (void)xSemaphoreGiveFromISR(g_txDoneSem, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((g_uart != NULL) && (huart->Instance == g_uart->Instance))
  {
    if (g_rxLen < MODBUS_RTU_MAX_FRAME_SIZE)
    {
      g_rxBuffer[g_rxLen] = g_rxByte;
      g_rxLen++;
    }
    else
    {
      g_overflow = 1U;
    }

    g_receiving = 1U;
    g_lastRxTick = HAL_GetTick();
    ModbusRtu_StartReceive();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if ((g_uart != NULL) && (huart->Instance == g_uart->Instance))
  {
    ModbusRtu_ResetRx();
    ModbusRtu_StartReceive();

    if (g_txActive != 0U)
    {
      g_txStatus = HAL_ERROR;
      g_txActive = 0U;
      (void)xSemaphoreGiveFromISR(g_txDoneSem, &higherPriorityTaskWoken);
      portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
  }
}
