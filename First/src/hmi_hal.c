#include "hmi_hal.h"
#include "motor_hal.h"
#include "controls.h"

TIM_HandleTypeDef htim5;
void HMI_Init(void);
static void TIM5_Init(void);
void TIM5_IRQHandler(void);

buttonLED greenLED =
    {
        .name = "greenLED",
        .port = GPIOA,
        .pin = GPIO_PIN_1,
        .mode = GPIO_MODE_OUTPUT_PP,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_FREQ_LOW,
};

buttonLED redLED =
    {
        .name = "redLED",
        .port = GPIOC,
        .pin = GPIO_PIN_0,
        .mode = GPIO_MODE_OUTPUT_PP,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_FREQ_LOW,
};

// For tim5 interrupt
buttonLED activeLED =
    {
        .mode = GPIO_MODE_OUTPUT_PP,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_FREQ_LOW,
};

buttonLED homeButton =
    {
        .name = "homeButton",
        .port = GPIOB,
        .pin = GPIO_PIN_0,
        .mode = GPIO_MODE_INPUT,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_FREQ_LOW,
};

buttonLED auxButton =
    {
        .name = "auxButton",
        .port = GPIOA,
        .pin = GPIO_PIN_4,
        .mode = GPIO_MODE_INPUT,
        .pull = GPIO_NOPULL,
        .speed = GPIO_SPEED_FREQ_LOW,
};

/**
 * @brief Initializes the pins and state of a single button or LED
 *
 * @param butLED Button/LED object
 */
void buttonLED_Init(buttonLED *butLED)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Pin configuration
    GPIO_InitStruct.Pin = butLED->pin;
    GPIO_InitStruct.Mode = butLED->mode;
    GPIO_InitStruct.Pull = butLED->pull;
    GPIO_InitStruct.Speed = butLED->speed;
    HAL_GPIO_Init(butLED->port, &GPIO_InitStruct);

    butLED->pin_state = HAL_GPIO_ReadPin(butLED->port, butLED->pin);
}

/**
 * @brief Initializes all buttons/LEDs, to be used in main
 *
 */
void HMI_Init(void)
{
    TIM5_Init();

    buttonLED_Init(&greenLED);
    buttonLED_Init(&redLED);
    buttonLED_Init(&homeButton);
    buttonLED_Init(&auxButton);
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

/**
 * @brief Flashes an LED at a slow (0.5s) or fast (0.1s) pace
 *
 * @param butLED LED to flash
 * @param speed period in [s]
 */
void flashLED(buttonLED butLED, double speed)
{
    double timerPeriod = 1000000 * speed; // 1MHz clock * Speed
    activeLED.port = butLED.port;
    activeLED.pin = butLED.pin;
    __HAL_TIM_SetCounter(&htim5, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim5, timerPeriod);
    HAL_TIM_Base_Start_IT(&htim5);
}

/**
 * @brief Turns an LED on
 *
 * @param butLED LED to turn on
 */
void solidLED(buttonLED butLED)
{
    HAL_TIM_Base_Stop_IT(&htim5);
    HAL_GPIO_WritePin(butLED.port, butLED.pin, GPIO_PIN_SET);
}

/**
 * @brief Turns an LED off
 *
 * @param butLED LED to turn off
 */
void stopLED(buttonLED butLED)
{
    HAL_TIM_Base_Stop_IT(&htim5);
    HAL_GPIO_WritePin(butLED.port, butLED.pin, GPIO_PIN_RESET);
}

/**
 * @brief Changes state of LEDs
 *
 * @param butLED Button/LED object
 * @param ledMode Slow, Fast, or Solid
 */
void changeLEDState(buttonLED butLED, const char *ledMode)
{
    if (strcmp(butLED.name, "greenLED") == 0)
    {
        if (strcmp(ledMode, "Slow") == 0)
        {
            stopLED(redLED);
            flashLED(greenLED, 0.5);
        }
        else if (strcmp(ledMode, "Fast") == 0)
        {
            stopLED(redLED);
            flashLED(greenLED, 0.1);
        }
        else
        {
            stopLED(redLED);
            solidLED(greenLED);
        }
    }
    else
    {
        if (strcmp(ledMode, "Slow") == 0)
        {
            stopLED(greenLED);
            flashLED(redLED, 0.5);
        }
        else if (strcmp(ledMode, "Fast") == 0)
        {
            stopLED(greenLED);
            flashLED(redLED, 0.1);
        }
        else
        {
            stopLED(greenLED);
            solidLED(redLED);
        }
    }
}

/**
 * @brief Reads the digital state of a button
 *
 * @param butLED Button to read
 */
void readDigitalPinState(buttonLED butLED)
{
    GPIO_PinState button_state = HAL_GPIO_ReadPin(butLED.port, butLED.pin);
    const char *button_name = butLED.name;

    if (button_state == GPIO_PIN_SET)
    {
        LOG_INFO("%s INACTIVE", button_name);
    }
    else if (button_state == GPIO_PIN_RESET)
    {
        LOG_INFO("%s ACTIVE", button_name);
    }
}

/**
 * @brief Debug to test all lights and buttons of the HMI
 *
 */
void hmiTesting(void)
{
    // LOG_INFO("HMI Testing Beginning in 3s");
    // HAL_Delay(3000);

    // LOG_INFO("Green Solid");
    // changeLEDState(greenLED, "Solid");
    // HAL_Delay(3000);

    // LOG_INFO("Green Slow");
    // changeLEDState(greenLED, "Slow");
    // HAL_Delay(3000);

    // LOG_INFO("Green Fast");
    // changeLEDState(greenLED, "Fast");
    // HAL_Delay(3000);

    // LOG_INFO("Red Solid");
    // changeLEDState(redLED, "Solid");
    // HAL_Delay(3000);

    // LOG_INFO("Red Slow");
    // changeLEDState(redLED, "Slow");
    // HAL_Delay(3000);

    // LOG_INFO("Red Fast");
    // changeLEDState(redLED, "Fast");
    // HAL_Delay(3000);

    int i = 0;
    for (i = 0; i < 100; i++)
    {
        if (HAL_GPIO_ReadPin(homeButton.port, homeButton.pin) == GPIO_PIN_SET)
        {
            LOG_INFO("Home button ON");
        }
        else
        {
            LOG_INFO("Home button OFF");
        }

        if (HAL_GPIO_ReadPin(auxButton.port, auxButton.pin) == GPIO_PIN_SET)
        {
            LOG_INFO("Aux button ON");
        }
        else
        {
            LOG_INFO("Aux button OFF");
        }
        LOG_INFO("");
        HAL_Delay(1000);
    }
    LOG_INFO("Test complete, please try the RESET button");
}