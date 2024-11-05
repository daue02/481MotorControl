/**
 ******************************************************************************
 * @file    UART/UART_Printf/Src/stm32f4xx_hal_msp.c
 * @author  MCD Application Team
 * @brief   HAL MSP module.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F4xx_HAL_Examples
 * @{
 */

/** @defgroup HAL_MSP
 * @brief HAL MSP module.
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Private_Functions
 * @{
 */

/**
 * @brief UART MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  if (huart->Instance == USARTx) // Serial monitor configuration
  {
    /* Enable GPIO TX/RX clock */
    USARTx_TX_GPIO_CLK_ENABLE();
    USARTx_RX_GPIO_CLK_ENABLE();

    /* Enable USARTx clock */
    USARTx_CLK_ENABLE();

    /* UART TX GPIO pin configuration */
    GPIO_InitStruct.Pin = USARTx_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = USARTx_TX_AF;
    HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX GPIO pin configuration */
    GPIO_InitStruct.Pin = USARTx_RX_PIN;
    GPIO_InitStruct.Alternate = USARTx_RX_AF;
    HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
  }
  else if (huart->Instance == USART6) // Secondary UART configuration for USART6
  {
    /* Enable GPIO clock for USART6 TX/RX pins */
    USART6_TX_GPIO_CLK_ENABLE();
    USART6_RX_GPIO_CLK_ENABLE();

    /* Enable USART6 clock */
    USART6_CLK_ENABLE();

    /* UART6 TX GPIO pin configuration (PC6) */
    GPIO_InitStruct.Pin = USART6_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = USART6_TX_AF;
    HAL_GPIO_Init(USART6_TX_GPIO_PORT, &GPIO_InitStruct);

    /* UART6 RX GPIO pin configuration (PC7) */
    GPIO_InitStruct.Pin = USART6_RX_PIN;
    GPIO_InitStruct.Alternate = USART6_RX_AF;
    HAL_GPIO_Init(USART6_RX_GPIO_PORT, &GPIO_InitStruct);
  }
}

/**
 * @brief UART MSP De-Initialization
 *        This function frees the hardware resources used in this example:
 *          - Disable the Peripheral's clock
 *          - Revert GPIO and NVIC configuration to their default state
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) // De-initialize USART2
  {
    /*##-1- Reset peripherals for USART2 ##################################*/
    USARTx_FORCE_RESET();
    USARTx_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks ###########################*/
    /* Configure USART2 Tx as alternate function */
    HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
    /* Configure USART2 Rx as alternate function */
    HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);
  }
  else if (huart->Instance == USART6) // De-initialize USART6
  {
    /*##-1- Reset peripherals for USART6 ##################################*/
    USART6_FORCE_RESET();
    USART6_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks ###########################*/
    /* Configure USART6 Tx as alternate function */
    HAL_GPIO_DeInit(USART6_TX_GPIO_PORT, USART6_TX_PIN);
    /* Configure USART6 Rx as alternate function */
    HAL_GPIO_DeInit(USART6_RX_GPIO_PORT, USART6_RX_PIN);
  }
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
