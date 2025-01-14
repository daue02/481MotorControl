#include "encoder_hal.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

void MX_TIM1_Init(void) // Encoder 1
{
    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0; // No prescaler for full resolution
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
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

    // Initialize TIM1 in encoder mode
    HAL_TIM_Encoder_Init(&htim1, &sConfig);
    HAL_TIM_Base_Start(&htim1);
}

void MX_TIM8_Init(void) // Encoder 2
{
    __HAL_RCC_TIM8_CLK_ENABLE();

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0; // No prescaler for full resolution
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
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

    // Initialize TIM8 in encoder mode
    HAL_TIM_Encoder_Init(&htim8, &sConfig);
    HAL_TIM_Base_Start(&htim8);
}

/**
 * @brief  Initializes the GPIO pins and timers for the encoders.
 *
 * This function configures the GPIO pins PA8, PA9 for TIM1 and PC6, PC7 for TIM8
 * to be used as encoder inputs. It sets the pins to alternate function push-pull mode,
 * with pull-up resistors and high speed. It also initializes the TIM1 and TIM8 timers.
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

    // Initialize TIM1 and TIM8
    MX_TIM1_Init();
    MX_TIM8_Init();
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
void getTicks(int32_t *ticks1, int32_t *ticks2)
{
    *ticks1 = (int32_t)__HAL_TIM_GET_COUNTER(&htim1);
    *ticks2 = (int32_t)__HAL_TIM_GET_COUNTER(&htim8);
}