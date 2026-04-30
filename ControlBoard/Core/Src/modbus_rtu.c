#include "modbus_rtu.h"

#include "FreeRTOS.h"
#include "dwin.h"
#include "semphr.h"
#include "task.h"
#include <string.h>

#define MODBUS_RTU_FRAME_GAP_MS 5U
#define MODBUS_RTU_PORT_COUNT   2U

/* CRC-16 (Modbus) 查表：低字节先入，多项式 0xA001。
 * 256 项 × 2B = 512B Flash，换取每字节 1~2 周期 vs 软计算的约 30 周期。 */
static const uint16_t kModbusCrc16Table[256] = {
  0x0000U, 0xC0C1U, 0xC181U, 0x0140U, 0xC301U, 0x03C0U, 0x0280U, 0xC241U,
  0xC601U, 0x06C0U, 0x0780U, 0xC741U, 0x0500U, 0xC5C1U, 0xC481U, 0x0440U,
  0xCC01U, 0x0CC0U, 0x0D80U, 0xCD41U, 0x0F00U, 0xCFC1U, 0xCE81U, 0x0E40U,
  0x0A00U, 0xCAC1U, 0xCB81U, 0x0B40U, 0xC901U, 0x09C0U, 0x0880U, 0xC841U,
  0xD801U, 0x18C0U, 0x1980U, 0xD941U, 0x1B00U, 0xDBC1U, 0xDA81U, 0x1A40U,
  0x1E00U, 0xDEC1U, 0xDF81U, 0x1F40U, 0xDD01U, 0x1DC0U, 0x1C80U, 0xDC41U,
  0x1400U, 0xD4C1U, 0xD581U, 0x1540U, 0xD701U, 0x17C0U, 0x1680U, 0xD641U,
  0xD201U, 0x12C0U, 0x1380U, 0xD341U, 0x1100U, 0xD1C1U, 0xD081U, 0x1040U,
  0xF001U, 0x30C0U, 0x3180U, 0xF141U, 0x3300U, 0xF3C1U, 0xF281U, 0x3240U,
  0x3600U, 0xF6C1U, 0xF781U, 0x3740U, 0xF501U, 0x35C0U, 0x3480U, 0xF441U,
  0x3C00U, 0xFCC1U, 0xFD81U, 0x3D40U, 0xFF01U, 0x3FC0U, 0x3E80U, 0xFE41U,
  0xFA01U, 0x3AC0U, 0x3B80U, 0xFB41U, 0x3900U, 0xF9C1U, 0xF881U, 0x3840U,
  0x2800U, 0xE8C1U, 0xE981U, 0x2940U, 0xEB01U, 0x2BC0U, 0x2A80U, 0xEA41U,
  0xEE01U, 0x2EC0U, 0x2F80U, 0xEF41U, 0x2D00U, 0xEDC1U, 0xEC81U, 0x2C40U,
  0xE401U, 0x24C0U, 0x2580U, 0xE541U, 0x2700U, 0xE7C1U, 0xE681U, 0x2640U,
  0x2200U, 0xE2C1U, 0xE381U, 0x2340U, 0xE101U, 0x21C0U, 0x2080U, 0xE041U,
  0xA001U, 0x60C0U, 0x6180U, 0xA141U, 0x6300U, 0xA3C1U, 0xA281U, 0x6240U,
  0x6600U, 0xA6C1U, 0xA781U, 0x6740U, 0xA501U, 0x65C0U, 0x6480U, 0xA441U,
  0x6C00U, 0xACC1U, 0xAD81U, 0x6D40U, 0xAF01U, 0x6FC0U, 0x6E80U, 0xAE41U,
  0xAA01U, 0x6AC0U, 0x6B80U, 0xAB41U, 0x6900U, 0xA9C1U, 0xA881U, 0x6840U,
  0x7800U, 0xB8C1U, 0xB981U, 0x7940U, 0xBB01U, 0x7BC0U, 0x7A80U, 0xBA41U,
  0xBE01U, 0x7EC0U, 0x7F80U, 0xBF41U, 0x7D00U, 0xBDC1U, 0xBC81U, 0x7C40U,
  0xB401U, 0x74C0U, 0x7580U, 0xB541U, 0x7700U, 0xB7C1U, 0xB681U, 0x7640U,
  0x7200U, 0xB2C1U, 0xB381U, 0x7340U, 0xB101U, 0x71C0U, 0x7080U, 0xB041U,
  0x5000U, 0x90C1U, 0x9181U, 0x5140U, 0x9301U, 0x53C0U, 0x5280U, 0x9241U,
  0x9601U, 0x56C0U, 0x5780U, 0x9741U, 0x5500U, 0x95C1U, 0x9481U, 0x5440U,
  0x9C01U, 0x5CC0U, 0x5D80U, 0x9D41U, 0x5F00U, 0x9FC1U, 0x9E81U, 0x5E40U,
  0x5A00U, 0x9AC1U, 0x9B81U, 0x5B40U, 0x9901U, 0x59C0U, 0x5880U, 0x9841U,
  0x8801U, 0x48C0U, 0x4980U, 0x8941U, 0x4B00U, 0x8BC1U, 0x8A81U, 0x4A40U,
  0x4E00U, 0x8EC1U, 0x8F81U, 0x4F40U, 0x8D01U, 0x4DC0U, 0x4C80U, 0x8C41U,
  0x4400U, 0x84C1U, 0x8581U, 0x4540U, 0x8701U, 0x47C0U, 0x4680U, 0x8641U,
  0x8201U, 0x42C0U, 0x4380U, 0x8341U, 0x4100U, 0x81C1U, 0x8081U, 0x4040U
};

uint16_t ModbusRtu_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;
  uint16_t pos;

  if (data == NULL)
  {
    return crc;
  }

  for (pos = 0U; pos < length; ++pos)
  {
    uint8_t idx = (uint8_t)(crc ^ (uint16_t)data[pos]);
    crc = (uint16_t)((crc >> 8) ^ kModbusCrc16Table[idx]);
  }

  return crc;
}

uint16_t ModbusRtu_ReadU16BE(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

void ModbusRtu_WriteU16BE(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value >> 8);
  data[1] = (uint8_t)value;
}

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
