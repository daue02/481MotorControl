#include "drill_hal.h"
#include "controls.h"

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim9; // New timer for acceleration control

static void MX_TIM2_Init(void);
static void MX_TIM9_Init(void);
static void Drill_TimerCallback(void);

Drill motorDrill = {
    .name = "motorDrill",
    .pwmPort = GPIOA,
    .pwmPin = GPIO_PIN_5,
};

void Drill_Init(void)
{
    motorDrill.currentPower = 0;
    motorDrill.targetPower = 0;
    motorDrill.accel = 25; // % per sec
    MX_TIM2_Init();
    MX_TIM9_Init();
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (double)htim2.Init.Period / 100 * motorDrill.currentPower);
}

void setDrillPower(int power)
{
    motorDrill.targetPower = power;
    LOG_INFO("Setting drill to %d%% power", power);

    if (motorDrill.currentPower != motorDrill.targetPower)
    {
        HAL_TIM_Base_Start_IT(&htim9); // Start acceleration timer
    }
}

void Drill_TimerCallback(void)
{
    if (motorDrill.currentPower == motorDrill.targetPower)
    {
        HAL_TIM_Base_Stop_IT(&htim9); // Stop timer when target power is reached
        return;
    }

    if (motorDrill.targetPower > motorDrill.currentPower)
    {
        motorDrill.currentPower++;
    }
    else
    {
        motorDrill.currentPower--;
    }

    uint32_t pwmValue = (uint32_t)(((double)htim2.Init.Period / 100.0) * motorDrill.currentPower);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwmValue);
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim9);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM9)
    {
        Drill_TimerCallback();
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
    HAL_TIM_PWM_Init(&htim2);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = motorDrill.pwmPin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(motorDrill.pwmPort, &GPIO_InitStruct);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

static void MX_TIM9_Init(void)
{
    __HAL_RCC_TIM9_CLK_ENABLE();
    htim9.Instance = TIM9;
    htim9.Init.Prescaler = (uint32_t)((SystemCoreClock / 10000) - 1);
    htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim9.Init.Period = 1000 / motorDrill.accel; // Adjusted based on acceleration
    htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim9);
    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
    __HAL_TIM_CLEAR_FLAG(&htim9, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim9, TIM_IT_UPDATE);
}