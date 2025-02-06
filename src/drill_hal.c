#include "drill_hal.h"
#include "controls.h"

TIM_HandleTypeDef htim2;

static void MX_TIM2_Init(void);

Drill motorDrill = {
    .name = "motorDrill",
    .pwmPort = GPIOA,
    .pwmPin = GPIO_PIN_5,
};

/**
 * @brief Initializes the timer for drill PWM.
 *
 */
void Drill_Init(void)
{
    motorDrill.currentPower = 0;
    motorDrill.targetPower = 0;
    motorDrill.accel = 5; // % per sec
    MX_TIM2_Init();
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (double)htim2.Init.Period / 100 * motorDrill.currentPower);
}

/**
 * @brief Accelerate/Decelerate the drill to a new power
 *
 * @param power Power expressed as %
 */
void setDrillPower(int power)
{
    motorDrill.targetPower = power;
    while (motorDrill.targetPower != motorDrill.currentPower)
    {
        if (motorDrill.targetPower > motorDrill.currentPower)
        {
            motorDrill.currentPower = motorDrill.currentPower + 1;
        }
        else if (motorDrill.targetPower < motorDrill.currentPower)
        {
            motorDrill.currentPower = motorDrill.currentPower - 1;
        }

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (double)htim2.Init.Period / 100 * motorDrill.currentPower);
        HAL_Delay(1 / (motorDrill.accel / 1000));
        LOG_INFO("Power level: %d", motorDrill.currentPower); // Use the logging macro
    }
}

static void MX_TIM2_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = (uint32_t)((SystemCoreClock / 10000000) - 1);
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 335; // Approx 15kHz
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2); // Initialize TIM2 in PWM mode

    // Configure the PWM channel
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

    // Configure GPIO Pin PA5 for Alternate Function (TIM2_CH2)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = motorDrill.pwmPin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(motorDrill.pwmPort, &GPIO_InitStruct);

    // Start PWM on TIM2 Channel 2
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}
