/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : ADDR8_Pin ADDR7_Pin ADDR4_Pin ADDR3_Pin
                           ADDR2_Pin ADDR1_Pin */
  GPIO_InitStruct.Pin = ADDR8_Pin|ADDR7_Pin|ADDR4_Pin|ADDR3_Pin
                          |ADDR2_Pin|ADDR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ADDR6_Pin ADDR5_Pin */
  GPIO_InitStruct.Pin = ADDR6_Pin|ADDR5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
uint8_t GPIO_ReadDeviceAddr(void)
{
  uint8_t addr = 0U;

  /* ADDR1 对应最高位 bit7，拨码/短接到低电平时该位为 1。 */
  if (HAL_GPIO_ReadPin(ADDR1_GPIO_Port, ADDR1_Pin) == GPIO_PIN_RESET)
  {
    addr |= (uint8_t)(1U << 7);
  }

  /* ADDR2 对应 bit6。 */
  if (HAL_GPIO_ReadPin(ADDR2_GPIO_Port, ADDR2_Pin) == GPIO_PIN_RESET)
  {
    addr |= (uint8_t)(1U << 6);
  }

  /* ADDR3 对应 bit5。 */
  if (HAL_GPIO_ReadPin(ADDR3_GPIO_Port, ADDR3_Pin) == GPIO_PIN_RESET)
  {
    addr |= (uint8_t)(1U << 5);
  }

  /* ADDR4 对应 bit4。 */
  if (HAL_GPIO_ReadPin(ADDR4_GPIO_Port, ADDR4_Pin) == GPIO_PIN_RESET)
  {
    addr |= (uint8_t)(1U << 4);
  }

  /* ADDR5 对应 bit3。 */
  if (HAL_GPIO_ReadPin(ADDR5_GPIO_Port, ADDR5_Pin) == GPIO_PIN_RESET)
  {
    addr |= (uint8_t)(1U << 3);
  }

  /* ADDR6 对应 bit2。 */
  if (HAL_GPIO_ReadPin(ADDR6_GPIO_Port, ADDR6_Pin) == GPIO_PIN_RESET)
  {
    addr |= (uint8_t)(1U << 2);
  }

  /* ADDR7 对应 bit1。 */
  if (HAL_GPIO_ReadPin(ADDR7_GPIO_Port, ADDR7_Pin) == GPIO_PIN_RESET)
  {
    addr |= (uint8_t)(1U << 1);
  }

  /* ADDR8 对应最低位 bit0。 */
  if (HAL_GPIO_ReadPin(ADDR8_GPIO_Port, ADDR8_Pin) == GPIO_PIN_RESET)
  {
    addr |= (uint8_t)(1U << 0);
  }

  return addr;
}

/* USER CODE END 2 */
