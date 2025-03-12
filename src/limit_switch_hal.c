#include "limit_switch_hal.h"
#include "motor_hal.h"
#include "main.h"
#include "controls.h"
#include "drill_hal.h"

void EXTI9_5_IRQHandler(void);
void Switch_Init(InterruptSwitch *interruptSW);

#define LIMIT_SWITCH_DEBOUNCE_MS 10  // Software debounce time in milliseconds
#define DEBOUNCE_OK(last_time, now) ((now - last_time) > LIMIT_SWITCH_DEBOUNCE_MS)

InterruptSwitch ySW_pos =
    {
        .name = "ySwitchPos",
        .port = GPIOB,
        .pin = GPIO_PIN_9,
        .lastDebounceTime = 0
};

InterruptSwitch ySW_neg =
    {
        .name = "ySwitchNeg",
        .port = GPIOB,
        .pin = GPIO_PIN_8,
        .lastDebounceTime = 0
};

InterruptSwitch zSW_pos =
    {
        .name = "zSwitchPos",
        .port = GPIOB,
        .pin = GPIO_PIN_6,
        .lastDebounceTime = 0
};

InterruptSwitch zSW_neg =
    {
        .name = "zSwitchNeg",
        .port = GPIOB,
        .pin = GPIO_PIN_5,
        .lastDebounceTime = 0
};

/**
 * @brief Initializes the pins and state of the limit switch / pi interrupt.
 *
 * @param InterruptSwitch Interrupt switch object.
 */
void Switch_Init(InterruptSwitch *interruptSW)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = interruptSW->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(interruptSW->port, &GPIO_InitStruct);

    interruptSW->Pin_state = HAL_GPIO_ReadPin(interruptSW->port, interruptSW->pin);
}

/**
 * @brief Initializes all the limit switches, called in main.
 *
 */
void Limit_Switch_Init(void)
{
    Switch_Init(&ySW_pos);
    Switch_Init(&ySW_neg);
    Switch_Init(&zSW_pos);
    Switch_Init(&zSW_neg);

    // Enable and set EXTI line Interrupt to the given priority
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
 * @brief Pins 5-9 interrupt handler, currently used for limit switches.
 *
 */
void EXTI9_5_IRQHandler(void)
{
    if (!isDrillPWMDisabled())
    {
        HAL_GPIO_EXTI_IRQHandler(ySW_pos.pin);
        HAL_GPIO_EXTI_IRQHandler(ySW_neg.pin);
        HAL_GPIO_EXTI_IRQHandler(zSW_pos.pin);
        HAL_GPIO_EXTI_IRQHandler(zSW_neg.pin);
        return;
    }

    uint32_t now = HAL_GetTick();  // Get current system time

    GPIO_PinState yPin_p_state = HAL_GPIO_ReadPin(ySW_pos.port, ySW_pos.pin);
    GPIO_PinState yPin_n_state = HAL_GPIO_ReadPin(ySW_neg.port, ySW_neg.pin);
    GPIO_PinState zPin_p_state = HAL_GPIO_ReadPin(zSW_pos.port, zSW_pos.pin);
    GPIO_PinState zPin_n_state = HAL_GPIO_ReadPin(zSW_neg.port, zSW_neg.pin);

    if (ySW_pos.Pin_state != yPin_p_state && DEBOUNCE_OK(ySW_pos.lastDebounceTime, now))
    {
        ySW_pos.lastDebounceTime = now;
        if (yPin_p_state)
        {
            motorY.isMoving = 0;
            LOG_INFO("Y+ Engaged");
        }
        else
        {
            LOG_INFO("Y+ Disengaged");
        }
        HAL_GPIO_EXTI_IRQHandler(ySW_pos.pin);
        ySW_pos.Pin_state = yPin_p_state;
    }

    if (ySW_neg.Pin_state != yPin_n_state && DEBOUNCE_OK(ySW_neg.lastDebounceTime, now))
    {
        ySW_neg.lastDebounceTime = now;
        if (yPin_n_state)
        {
            motorY.isMoving = 0;
            LOG_INFO("Y- Engaged");
        }
        else
        {
            LOG_INFO("Y- Disengaged");
        }
        HAL_GPIO_EXTI_IRQHandler(ySW_neg.pin);
        ySW_neg.Pin_state = yPin_n_state;
    }

    if (zSW_pos.Pin_state != zPin_p_state && DEBOUNCE_OK(zSW_pos.lastDebounceTime, now))
    {
        zSW_pos.lastDebounceTime = now;
        if (zPin_p_state)
        {
            motorZ.isMoving = 0;
            LOG_INFO("Z+ Engaged");
        }
        else
        {
            LOG_INFO("Z+ Disengaged");
        }
        HAL_GPIO_EXTI_IRQHandler(zSW_pos.pin);
        zSW_pos.Pin_state = zPin_p_state;
    }

    if (zSW_neg.Pin_state != zPin_n_state && DEBOUNCE_OK(zSW_neg.lastDebounceTime, now))
    {
        zSW_neg.lastDebounceTime = now;
        if (zPin_n_state)
        {
            motorZ.isMoving = 0;
            LOG_INFO("Z- Engaged");
        }
        else
        {
            LOG_INFO("Z- Disengaged");
        }
        HAL_GPIO_EXTI_IRQHandler(zSW_neg.pin);
        zSW_neg.Pin_state = zPin_n_state;
    }
}
