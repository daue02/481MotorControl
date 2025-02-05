#include "stm32f4xx_hal.h"

ADC_HandleTypeDef hadc1;

const float R1 = 485000.0; // 470K, measured with multimeter
const float R2 = 38100.0;  // 3x 100K parallel + 5.1k series, measured with multimeter
const float V_REF = 3.3;

void ADC_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE(); // Enable ADC1 clock

    // Pin Config
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // ADC Config
    hadc1.Instance = ADC1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;

    HAL_ADC_Init(&hadc1);
}

float readBatteryVoltage()
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_11; // PC1
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    float scaleFactor = (R1 + R2) / R2;
    float vOut = (adcValue / 4095.0) * V_REF;
    float batVoltage = vOut * scaleFactor;
    return adcValue;
}