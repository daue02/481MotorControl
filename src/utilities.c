#include "stm32f4xx_hal.h"
#include "controls.h"
#include "limit_switch_hal.h"
#include "utilities.h"

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim5;

void ErrorHandler(void);
static void TIM5_Init(void);
void TIM5_IRQHandler(void);

Battery bat =
    {
        .name = "Battery",
        .adcPort = GPIOC,
        .adcPin = GPIO_PIN_1,
        .adcChannel = ADC_CHANNEL_11,
        .R1 = 493000.0, // 470K, measured with multimeter
        .R2 = 38100.0,  // 3x 100K parallel + 5.1k series, measured with multimeter
        .V_REF = 3.3,
        .V_MIN = 32.0, // Min safe range is 30-32V. Absolute minimum is 25V
};

LED greenLED =
    {
        .name = "greenLED",
        .port = GPIOA,
        .pin = GPIO_PIN_1,
        .mode = GPIO_MODE_OUTPUT_PP,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_FREQ_LOW,
};

LED redLED =
    {
        .name = "redLED",
        .port = GPIOC,
        .pin = GPIO_PIN_0,
        .mode = GPIO_MODE_OUTPUT_PP,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_FREQ_LOW,
};

// For tim5 interrupt
LED activeLED =
    {
        .mode = GPIO_MODE_OUTPUT_PP,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_FREQ_LOW,
};

/**
 * @brief Initializes battery pins and ADC
 *
 */
void Battery_Health_Init(void)
{
    // Pin Config
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = bat.adcPin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(bat.adcPort, &GPIO_InitStruct);

    // ADC Config
    __HAL_RCC_ADC1_CLK_ENABLE(); // Enable ADC1 clock
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        LOG_ERROR("Battery ADC init failed");
        ErrorHandler();
    }

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = bat.adcChannel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        LOG_ERROR("Battery ADC init failed");
        ErrorHandler();
    }
}

/**
 * @brief Initializes the pins and state of a single LED
 *
 * @param led LED object
 */
void LED_Init(LED *led)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = led->pin;
    GPIO_InitStruct.Mode = led->mode;
    GPIO_InitStruct.Pull = led->pull;
    GPIO_InitStruct.Speed = led->speed;
    HAL_GPIO_Init(led->port, &GPIO_InitStruct);

    led->pin_state = HAL_GPIO_ReadPin(led->port, led->pin);
}

/**
 * @brief Initializes the LEDs and battery, to be called in main
 *
 */
void Utilities_Init(void)
{
    TIM5_Init();

    Battery_Health_Init();
    LED_Init(&greenLED);
    LED_Init(&redLED);
}

/**
 * @brief Returns current battery voltage. Runs at start of code, and everytime the state is changed
 *
 * @param bat Battery object
 * @return Battery voltage as a float
 */
float readBatteryVoltage(Battery *bat)
{
    // Poll multiple times and average
    int numSamples = 10;
    int totalAdcValue = 0;

    for (int i = 0; i < numSamples; i++)
    {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        {
            totalAdcValue += HAL_ADC_GetValue(&hadc1);
        }
        // HAL_Delay(1); # This delay locks up when called from an isr, seems to work without
    }

    float adcValue = totalAdcValue / (float)numSamples;
    float scaleFactor = (bat->R1 + bat->R2) / bat->R2;
    float vOut = (adcValue / 4095.0) * bat->V_REF;
    float batVoltage = vOut * scaleFactor;

    return batVoltage;
}

/**
 * @brief Converts battery voltage to a percentage. Only valid under low current.
 *
 * @param v Battery charge [V]
 * @return Battery charge [%]
 */
double batVoltagetoPercent(double v)
{
    double SoC[12] = {0, 5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    double Voltage[12] = {30.0, 34.5, 36.8, 37.4, 37.7, 37.9, 38.2, 38.7, 39.2, 39.8, 40.6, 42.0};

    // Check if voltage is out of range
    if (v < Voltage[0] || v > Voltage[11])
    {
        return -1; // Out of range
    }

    // If voltage exactly matches a table value, return corresponding SoC
    for (int i = 0; i < 12; i++)
    {
        if (v == Voltage[i])
        {
            return SoC[i];
        }
    }

    // If not, linearly interpolate
    for (int i = 0; i < 11; i++)
    {
        if (v > Voltage[i] && v < Voltage[i + 1])
        {
            return SoC[i] + (v - Voltage[i]) * (SoC[i + 1] - SoC[i]) / (Voltage[i + 1] - Voltage[i]);
        }
    }

    return -1; // Should never reach here - just to avoid caution
}

/**
 * @brief Check limit switch continuity and battery health
 *
 */
void SystemHealthCheck(void)
{
    // Check and display battery voltage; Fault out if too low
    float voltage = readBatteryVoltage(&bat);
    double percent = batVoltagetoPercent(voltage);
    if (voltage < bat.V_MIN)
    {
        LOG_ERROR("Battery Voltage LOW: %d.%02dV (%d%%)", (int)voltage, (int)(voltage * 100) % 100, (int)percent);
        ErrorHandler();
    }
    LOG_INFO("Battery Voltage: %d.%02dV (%d%%)", (int)voltage, (int)(voltage * 100) % 100, (int)percent);

    // Check that all limit switches are closed (NC switched).
    if (ySW_pos.Pin_state)
    {
        LOG_ERROR("Check Y+ sw");
    }
    else if (ySW_neg.Pin_state)
    {
        LOG_ERROR("Check Y- sw");
    }
    else if (zSW_pos.Pin_state)
    {
        LOG_ERROR("Check Z+ sw");
    }
    else if (zSW_neg.Pin_state)
    {
        LOG_ERROR("Check Z- sw");
    }
    else
    {
        return;
    }
    ErrorHandler();
}

/**
 * @brief Will hold the device in an infinte loop on error.
 *
 */
void ErrorHandler(void)
{
    updateStateMachine("Faulted");
    while (1)
    {
    }
}

/**
 * @brief Changes overall LED state
 *
 * @param led LED object
 * @param ledMode Slow, Fast, Solid
 */
void changeLEDState(LED led, const char *ledMode)
{
    // Shut off the other LED
    if (strcmp(led.name,"greenLED") == 0)
    {
        HAL_TIM_Base_Stop_IT(&htim5);
        HAL_GPIO_WritePin(redLED.port, redLED.pin, GPIO_PIN_RESET);        
    }
    else
    {
        HAL_TIM_Base_Stop_IT(&htim5);
        HAL_GPIO_WritePin(greenLED.port, greenLED.pin, GPIO_PIN_RESET);  
    }

    // Set this LED to slow, fast, or solid
    if (strcmp(ledMode, "Slow") == 0 || strcmp(ledMode, "Fast") == 0)
    {
        double flashSpeed = 0;
        if (strcmp(ledMode, "Slow") == 0)
        {
            flashSpeed = 0.5; // Slow interval [s]
        }
        else
        {
            flashSpeed = 0.1; // Fast interval [s]
        }
        double timerPeriod = 1000000 * flashSpeed; // 1MHz clock * Speed
        activeLED.port = led.port;
        activeLED.pin = led.pin;
        __HAL_TIM_SetCounter(&htim5, 0);
        __HAL_TIM_SET_AUTORELOAD(&htim5, timerPeriod);
        HAL_TIM_Base_Start_IT(&htim5);
    }
    else if (strcmp(ledMode, "Solid") == 0)
    {
        HAL_TIM_Base_Stop_IT(&htim5);
        HAL_GPIO_WritePin(led.port, led.pin, GPIO_PIN_SET);
    }
    else
    {
        LOG_ERROR("Invalid LED type commanded");
        ErrorHandler();
    }
}

static void TIM5_Init(void)
{
    __HAL_RCC_TIM5_CLK_ENABLE();

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1); // 1 MHz clock
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFF; // 0.5s / inv(1MHz)
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim5);

    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

void TIM5_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim5, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
            HAL_GPIO_TogglePin(activeLED.port, activeLED.pin);
        }
    }
}