#include "encoder_hal.h"
#include "uart.h"

// Timer handles for encoder timers and TIM10 for periodic transmission
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;

void MX_TIM1_Init(void) // Encoder 1
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;   // No prescaler for full resolution
    htim1.Init.Period = 0xFFFF; // Max value for 16-bit counter
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef sConfig = {0};
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12; // Both channels for quadrature encoding

    // Channel 1 Configuration
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0x0F; // Add a filter to suppress noise

    // Channel 2 Configuration
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0x0F; // Add a filter to suppress noise

    HAL_TIM_Encoder_Init(&htim1, &sConfig);
    HAL_TIM_Base_Start(&htim1);
}

void MX_TIM8_Init(void) // Encoder 2
{
    __HAL_RCC_TIM8_CLK_ENABLE();

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;   // No prescaler for full resolution
    htim8.Init.Period = 0xFFFF; // Max value for 16-bit counter
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    TIM_Encoder_InitTypeDef sConfig = {0};
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12; // Both channels for quadrature encoding

    // Channel 1 Configuration
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0x0F; // Add a filter to suppress noise

    // Channel 2 Configuration
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0x0F; // Add a filter to suppress noise

    HAL_TIM_Encoder_Init(&htim8, &sConfig);
    HAL_TIM_Base_Start(&htim8);
}

/**
 * @brief  Initializes TIM10 to generate an interrupt every 50ms.
 *
 * TIM10 is configured with a prescaler and period that yield a 50ms overflow.
 * Here we assume the timer clock is 180MHz (APB2 clock with no divider).
 * With a prescaler of 1799, the counter clock becomes 100kHz, so a period
 * of 4999 gives 5000 ticks = 50ms.
 */
void MX_TIM10_Init(void)
{
    __HAL_RCC_TIM10_CLK_ENABLE();

    htim10.Instance = TIM10;
    htim10.Init.Prescaler = 1799; // (180MHz / 1800) = 100kHz
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = 4999; // 100kHz * 0.05s = 5000 counts (Period = 5000-1)
    htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
    {
        LOG_ERROR("TIM10 initialization failed");
    }

    // Set up NVIC for TIM10 (TIM10 shares IRQ with TIM1 Update)
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
}

/**
 * @brief  Initializes the GPIO pins and timers for the encoders.
 *
 * This function configures the GPIO pins for TIM1 and TIM8 (for the encoders)
 * and then initializes TIM10 for periodic tick transmission.
 */
void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure PA8 (TIM1_CH1) and PA9 (TIM1_CH2) for first encoder
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure PC6 (TIM8_CH1) and PC7 (TIM8_CH2) for second encoder
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Initialize encoder timers
    MX_TIM1_Init();
    MX_TIM8_Init();

    // Initialize TIM10 for periodic UART transmission and start its interrupt
    MX_TIM10_Init();
    HAL_TIM_Base_Start_IT(&htim10);
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim10, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim10, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim10, TIM_IT_UPDATE);

            int16_t ticks1, ticks2;
            getTicks(&ticks1, &ticks2);
            sendTicks(ticks1, ticks2);
        }
    }
}

/**
 * @brief  Retrieves the current tick counts from the encoders.
 *
 * @param  ticks1: Pointer to an integer where the tick count of the first encoder (TIM1) will be stored.
 * @param  ticks2: Pointer to an integer where the tick count of the second encoder (TIM8) will be stored.
 *
 * This function reads the current counter values of TIM1 and TIM8 and stores them in the provided
 * integer pointers. These values represent the tick counts of the respective encoders.
 */
void getTicks(uint16_t *ticks1, uint16_t *ticks2)
{
    *ticks1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
    *ticks2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);
}

void stopTicksTimer(void)
{
    HAL_TIM_Base_Stop_IT(&htim10);
}

void startTicksTimer(void)
{
    HAL_TIM_Base_Start_IT(&htim10);
}
