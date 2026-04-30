#include "modbus_rtu.h"

#include "FreeRTOS.h"
#include "dwin.h"
#include "semphr.h"
#include "task.h"
#include <string.h>

#define MODBUS_RTU_FRAME_GAP_MS 5U
#define MODBUS_RTU_PORT_COUNT   2U

typedef struct
{
  UART_HandleTypeDef *uart;
  SemaphoreHandle_t txMutex;   /* 串行化同一 UART 上的发送操作。 */
  SemaphoreHandle_t txDoneSem; /* 由发送完成/错误回调释放。 */
  uint8_t rxByte;
  volatile uint8_t receiving;
  volatile uint8_t overflow;
  volatile uint8_t txActive;
  volatile HAL_StatusTypeDef txStatus;
  volatile uint16_t rxLen;
  volatile uint32_t lastRxTick;
  volatile uint32_t lastTxDoneTick;
  uint8_t rxBuffer[MODBUS_RTU_MAX_FRAME_SIZE];
} ModbusRtu_Port_t;

/* RTU 端口状态表：分别保存 USART1 和 USART2 的收发缓存、锁和中断状态。 */
static ModbusRtu_Port_t g_ports[MODBUS_RTU_PORT_COUNT];

static TickType_t ModbusRtu_MsToTicksCeil(uint32_t ms)
{
  TickType_t ticks = pdMS_TO_TICKS(ms);
  return (ticks > 0U) ? ticks : 1U;
}

static ModbusRtu_Port_t *ModbusRtu_FindPort(UART_HandleTypeDef *uart)
{
  uint8_t i;

  if (uart == NULL)
  {
    return NULL;
  }

  for (i = 0U; i < MODBUS_RTU_PORT_COUNT; ++i)
  {
    if ((g_ports[i].uart != NULL) && (g_ports[i].uart->Instance == uart->Instance))
    {
      return &g_ports[i];
    }
  }

  return NULL;
}

static ModbusRtu_Port_t *ModbusRtu_AllocPort(UART_HandleTypeDef *uart)
{
  uint8_t i;
  ModbusRtu_Port_t *port = ModbusRtu_FindPort(uart);

  if (port != NULL)
  {
    return port;
  }

  for (i = 0U; i < MODBUS_RTU_PORT_COUNT; ++i)
  {
    if (g_ports[i].uart == NULL)
    {
      g_ports[i].uart = uart;
      return &g_ports[i];
    }
  }

  return NULL;
}

static void ModbusRtu_StartReceive(ModbusRtu_Port_t *port)
{
  if ((port == NULL) || (port->uart == NULL))
  {
    return;
  }

  (void)HAL_UART_Receive_IT(port->uart, &port->rxByte, 1U);
}

static void ModbusRtu_ResetRx(ModbusRtu_Port_t *port)
{
  port->receiving = 0U;
  port->overflow = 0U;
  port->rxLen = 0U;
}

static uint8_t ModbusRtu_FrameReady(ModbusRtu_Port_t *port)
{
  uint32_t elapsed;

  if ((port == NULL) || (port->receiving == 0U))
  {
    return 0U;
  }

  /* RTU 没有长度字段，这里用 5 ms 静默间隔判断一帧已经结束。 */
  elapsed = HAL_GetTick() - port->lastRxTick;
  return (elapsed >= MODBUS_RTU_FRAME_GAP_MS) ? 1U : 0U;
}

static void ModbusRtu_WaitTxFrameGap(ModbusRtu_Port_t *port)
{
  /* 发送新帧前也保留帧间隔，避免连续帧被对端合并解析。 */
  while ((HAL_GetTick() - port->lastTxDoneTick) < MODBUS_RTU_FRAME_GAP_MS)
  {
    vTaskDelay(pdMS_TO_TICKS(1U));
  }
}

uint8_t ModbusRtu_InitPort(UART_HandleTypeDef *uart)
{
  ModbusRtu_Port_t *port = ModbusRtu_AllocPort(uart);

  if (port == NULL)
  {
    return 0U;
  }

  if (port->txMutex == NULL)
  {
    port->txMutex = xSemaphoreCreateMutex();
  }

  if (port->txDoneSem == NULL)
  {
    port->txDoneSem = xSemaphoreCreateBinary();
  }

  if ((port->txMutex == NULL) || (port->txDoneSem == NULL))
  {
    return 0U;
  }

  port->txActive = 0U;
  port->txStatus = HAL_OK;
  port->lastTxDoneTick = HAL_GetTick() - MODBUS_RTU_FRAME_GAP_MS;
  ModbusRtu_ResetRx(port);
  ModbusRtu_StartReceive(port);
  return 1U;
}

void ModbusRtu_ClearPort(UART_HandleTypeDef *uart)
{
  ModbusRtu_Port_t *port = ModbusRtu_FindPort(uart);

  if (port == NULL)
  {
    return;
  }

  taskENTER_CRITICAL();
  ModbusRtu_ResetRx(port);
  taskEXIT_CRITICAL();
}

uint8_t ModbusRtu_PollFrameFrom(UART_HandleTypeDef *uart, uint8_t *frame, uint16_t frameMax, uint16_t *frameLen)
{
  uint16_t copyLen;
  uint8_t overflow;
  ModbusRtu_Port_t *port = ModbusRtu_FindPort(uart);

  if ((port == NULL) || (frame == NULL) || (frameLen == NULL) || (ModbusRtu_FrameReady(port) == 0U))
  {
    return 0U;
  }

  /* 接收中断仍可能写缓存，复制和清空时需要进入临界区。 */
  taskENTER_CRITICAL();
  copyLen = port->rxLen;
  overflow = port->overflow;
  if (copyLen > frameMax)
  {
    copyLen = frameMax;
  }
  memcpy(frame, port->rxBuffer, copyLen);
  ModbusRtu_ResetRx(port);
  taskEXIT_CRITICAL();

  *frameLen = copyLen;
  return (overflow == 0U) ? 1U : 0U;
}

HAL_StatusTypeDef ModbusRtu_SendTo(UART_HandleTypeDef *uart, const uint8_t *frame, uint16_t frameLen, uint32_t timeoutMs)
{
  HAL_StatusTypeDef status;
  ModbusRtu_Port_t *port = ModbusRtu_FindPort(uart);

  if ((port == NULL) || (port->txMutex == NULL) || (port->txDoneSem == NULL) || (frame == NULL) || (frameLen == 0U))
  {
    return HAL_ERROR;
  }

  (void)xSemaphoreTake(port->txMutex, portMAX_DELAY);
  (void)xSemaphoreTake(port->txDoneSem, 0U);

  ModbusRtu_WaitTxFrameGap(port);

  /* 使用中断发送降低任务阻塞时间，任务只等待完成信号或超时。 */
  port->txStatus = HAL_BUSY;
  port->txActive = 1U;
  status = HAL_UART_Transmit_IT(port->uart, (uint8_t *)frame, frameLen);
  if (status != HAL_OK)
  {
    port->txActive = 0U;
    (void)xSemaphoreGive(port->txMutex);
    return status;
  }

  if (xSemaphoreTake(port->txDoneSem, ModbusRtu_MsToTicksCeil(timeoutMs)) != pdTRUE)
  {
    port->txActive = 0U;
    /* 超时后终止尚未完成的硬件发送，防止下一帧接着未完成状态继续发送。 */
    (void)HAL_UART_AbortTransmit(port->uart);
    (void)xSemaphoreGive(port->txMutex);
    return HAL_TIMEOUT;
  }

  status = port->txStatus;
  (void)xSemaphoreGive(port->txMutex);
  return status;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  ModbusRtu_Port_t *port = ModbusRtu_FindPort(huart);

  if ((port != NULL) && (port->txActive != 0U) && (port->txDoneSem != NULL))
  {
    port->lastTxDoneTick = HAL_GetTick();
    port->txStatus = HAL_OK;
    port->txActive = 0U;
    (void)xSemaphoreGiveFromISR(port->txDoneSem, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
  }

  UART3_SendTxCpltCallback(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  ModbusRtu_Port_t *port = ModbusRtu_FindPort(huart);

  if (port != NULL)
  {
    if (port->rxLen < MODBUS_RTU_MAX_FRAME_SIZE)
    {
      port->rxBuffer[port->rxLen] = port->rxByte;
      port->rxLen++;
    }
    else
    {
      port->overflow = 1U;
    }

    port->receiving = 1U;
    port->lastRxTick = HAL_GetTick();
    ModbusRtu_StartReceive(port);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  ModbusRtu_Port_t *port = ModbusRtu_FindPort(huart);

  if (port != NULL)
  {
    ModbusRtu_ResetRx(port);
    ModbusRtu_StartReceive(port);

    if ((port->txActive != 0U) && (port->txDoneSem != NULL))
    {
      port->txStatus = HAL_ERROR;
      port->txActive = 0U;
      (void)xSemaphoreGiveFromISR(port->txDoneSem, &higherPriorityTaskWoken);
      portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
  }

  UART3_SendErrorCallback(huart);
}
