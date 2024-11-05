/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "stdio.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

void ErrorHandler(void);

/* Definition for USARTx clock resources */
#define USARTx USART2
#define USARTx_CLK_ENABLE() __HAL_RCC_USART2_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET() __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET() __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN GPIO_PIN_2
#define USARTx_TX_GPIO_PORT GPIOA
#define USARTx_TX_AF GPIO_AF7_USART2
#define USARTx_RX_PIN GPIO_PIN_3
#define USARTx_RX_GPIO_PORT GPIOA
#define USARTx_RX_AF GPIO_AF7_USART2

/* Definition for USART1 clock resources */
#define USART6_CLK_ENABLE() __HAL_RCC_USART6_CLK_ENABLE()
#define USART6_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
#define USART6_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

#define USART6_FORCE_RESET() __HAL_RCC_USART6_FORCE_RESET()
#define USART6_RELEASE_RESET() __HAL_RCC_USART6_RELEASE_RESET()

/* Definition for USART6 Pins */
#define USART6_TX_PIN GPIO_PIN_6
#define USART6_TX_GPIO_PORT GPIOC
#define USART6_TX_AF GPIO_AF8_USART6
#define USART6_RX_PIN GPIO_PIN_7
#define USART6_RX_GPIO_PORT GPIOC
#define USART6_RX_AF GPIO_AF8_USART6

#endif /* __MAIN_H */