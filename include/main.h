/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "stdio.h"
#include "log.h"
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

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

/* Definition for USART5 clock resources */
#define UART5_CLK_ENABLE() __HAL_RCC_UART5_CLK_ENABLE()
#define UART5_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOD_CLK_ENABLE()
#define UART5_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()

#define UART5_FORCE_RESET() __HAL_RCC_UART5_FORCE_RESET()
#define UART5_RELEASE_RESET() __HAL_RCC_UART5_RELEASE_RESET()

/* Definition for UART5 Pins */
#define UART5_TX_PIN GPIO_PIN_12
#define UART5_TX_GPIO_PORT GPIOC
#define UART5_TX_AF GPIO_AF8_UART5
#define UART5_RX_PIN GPIO_PIN_2
#define UART5_RX_GPIO_PORT GPIOD
#define UART5_RX_AF GPIO_AF8_UART5

#endif /* __MAIN_H */