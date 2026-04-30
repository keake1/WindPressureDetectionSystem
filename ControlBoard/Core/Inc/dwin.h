/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    dwin.h
  * @brief   迪文屏通信接口声明
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DWIN_H__
#define __DWIN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "global_status.h"
#include <stdint.h>
#include <string.h>


#define DWIN_SENSOR_DATA_ADDR             0x1000U  /* 传感器数据区（屏幕读回后存放处） */
#define DWIN_SENSOR_STATUS_ICON_ADDR      0x1400U  /* 传感器状态图标区（屏幕读回后存放处） */
#define DWIN_WINDOWS_STATUS_ICON_ADDR     0x1440U  /* 窗状态图标区（屏幕读回后存放处） */
#define DWIN_GLOBAL_ALARM_ICON_ADDR       0x1441U  /* 全局报警状态图标（屏幕读回后存放处） */
#define DWIN_ZERO_ADDR_ICON_ADDR          0x1442U  /* 零地址状态图标（屏幕读回后存放处） */
#define DWIN_DUP_ADDR_ICON_ADDR           0x1443U  /* 重复地址状态图标（屏幕读回后存放处） */
#define DWIN_CONTROL_BORAD_ADDR_VAR       0x1500U  /* 控制板地址变量（屏幕写入后存放处） */
#define DWIN_SENSOR_DATA_STRIDE           16U      /* 每个传感器数据区占 16 个变量地址。 */
#define DWIN_SENSOR_DATA_CO_OFFSET        0U       /* CO，float，占 2 个地址。 */
#define DWIN_SENSOR_DATA_WIND_OFFSET      2U       /* 风压，float，占 2 个地址。 */
#define DWIN_SENSOR_DATA_RESIDUAL_OFFSET  4U       /* 余压，uint16，占 1 个地址。 */
#define DWIN_SENSOR_DATA_CO2_OFFSET       5U       /* CO2，uint16，占 1 个地址。 */
#define DWIN_SENSOR_DATA_CH2O_OFFSET      6U       /* CH2O，uint16，占 1 个地址。 */
#define DWIN_SENSOR_DATA_TVOC_OFFSET      7U       /* TVOC，uint16，占 1 个地址。 */
#define DWIN_SENSOR_DATA_PM25_OFFSET      8U       /* PM2.5，float，占 2 个地址。 */
#define DWIN_SENSOR_DATA_PM10_OFFSET      10U      /* PM10，float，占 2 个地址。 */
#define DWIN_SENSOR_DATA_TEMP_OFFSET      12U      /* 温度，float，占 2 个地址。 */
#define DWIN_SENSOR_DATA_HUMI_OFFSET      14U      /* 湿度，float，占 2 个地址。 */
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/**
 * @brief 向迪文屏指定变量地址写入任意长度数据（0x82 指令）
 *
 * 帧格式：
 *   5A A5 [LEN] 82 [AH] [AL] [D0] [D1] ... [Dn]
 *   LEN = 3 + data_len（指令码1 + 地址2 + 数据n）
 *
 * @param addr      目标变量起始地址
 * @param pData     待写入的原始字节数据（调用方负责大端序）
 * @param data_len  数据字节数（最大 0xF9，即 LEN 上限 0xFF - 3 固定字节）
 */
void DWIN_WriteVar(uint16_t addr, const uint8_t *pData, uint8_t data_len);
void DwinIcon_WriteValue(uint16_t addr, uint8_t value);
uint8_t DwinIcon_GetSensorStatusValue(uint8_t online, uint8_t alarm);
void DwinValue_WriteU16(uint16_t addr, uint16_t value);
void DwinValue_WriteFloat(uint16_t addr, float value);
void DwinValue_UpdateSensor(uint8_t sensorIndex, const uint16_t *data, uint8_t forceUpdate);
uint8_t UART3_SendInit(void);
HAL_StatusTypeDef UART3_Send(const uint8_t *buf, uint16_t len);
void UART3_SendTxCpltCallback(UART_HandleTypeDef *huart);
void UART3_SendErrorCallback(UART_HandleTypeDef *huart);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __DWIN_H__ */
