#include "limit_switch_hal.h"
#include "motor_hal.h"
#include "main.h"
#include "controls.h"
#include "drill_hal.h"

void EXTI9_5_IRQHandler(void);
void Switch_Init(LimitSwitch *limitSW);

#define LIMIT_SWITCH_DEBOUNCE_MS 20

LimitSwitch ySW_pos =
    {
        .name = "ySwitchPos",
        .port = GPIOB,
        .pin = GPIO_PIN_9,
        .lastDebounceTime = 0};

LimitSwitch ySW_neg =
    {
        .name = "ySwitchNeg",
        .port = GPIOB,
        .pin = GPIO_PIN_8,
        .lastDebounceTime = 0};

LimitSwitch zSW_pos =
    {
        .name = "zSwitchPos",
        .port = GPIOB,
        .pin = GPIO_PIN_6,
        .lastDebounceTime = 0};

LimitSwitch zSW_neg =
    {
        .name = "zSwitchNeg",
        .port = GPIOB,
        .pin = GPIO_PIN_5,
        .lastDebounceTime = 0};

/**
 * @brief Initializes the pins and state of the limit switch
 *
 * @param limitSW Interrupt switch object.
 */
void Switch_Init(LimitSwitch *limitSW)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = limitSW->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(limitSW->port, &GPIO_InitStruct);

    limitSW->Pin_state = HAL_GPIO_ReadPin(limitSW->port, limitSW->pin);
}

/**
 * @brief Initializes all the limit switches, called in main.
 */
void Limit_Switch_Init(void)
{
    Switch_Init(&ySW_pos);
    Switch_Init(&ySW_neg);
    Switch_Init(&zSW_pos);
    Switch_Init(&zSW_neg);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/**
 * @brief Pins 5-9 interrupt handler, currently used for limit switches.
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

    uint32_t now = HAL_GetTick();

    // Y+
    if ((now - ySW_pos.lastDebounceTime) > LIMIT_SWITCH_DEBOUNCE_MS)
    {
        ySW_pos.lastDebounceTime = now;
        GPIO_PinState new_state = HAL_GPIO_ReadPin(ySW_pos.port, ySW_pos.pin);
        if (ySW_pos.Pin_state != new_state)
        {
            ySW_pos.Pin_state = new_state;
            if (new_state)
            {
                motorY.isMoving = 0;
                LOG_INFO("Y+ Engaged");
            }
            else
            {
                LOG_INFO("Y+ Disengaged");
            }
        }
    }
    HAL_GPIO_EXTI_IRQHandler(ySW_pos.pin);

    // Y-
    if ((now - ySW_neg.lastDebounceTime) > LIMIT_SWITCH_DEBOUNCE_MS)
    {
        ySW_neg.lastDebounceTime = now;
        GPIO_PinState new_state = HAL_GPIO_ReadPin(ySW_neg.port, ySW_neg.pin);
        if (ySW_neg.Pin_state != new_state)
        {
            ySW_neg.Pin_state = new_state;
            if (new_state)
            {
                motorY.isMoving = 0;
                LOG_INFO("Y- Engaged");
            }
            else
            {
                LOG_INFO("Y- Disengaged");
            }
        }
    }
    HAL_GPIO_EXTI_IRQHandler(ySW_neg.pin);

    // Z+
    if ((now - zSW_pos.lastDebounceTime) > LIMIT_SWITCH_DEBOUNCE_MS)
    {
        zSW_pos.lastDebounceTime = now;
        GPIO_PinState new_state = HAL_GPIO_ReadPin(zSW_pos.port, zSW_pos.pin);
        if (zSW_pos.Pin_state != new_state)
        {
            zSW_pos.Pin_state = new_state;
            if (new_state)
            {
                motorZ.isMoving = 0;
                LOG_INFO("Z+ Engaged");
            }
            else
            {
                LOG_INFO("Z+ Disengaged");
            }
        }
    }
    HAL_GPIO_EXTI_IRQHandler(zSW_pos.pin);

    // Z-
    if ((now - zSW_neg.lastDebounceTime) > LIMIT_SWITCH_DEBOUNCE_MS)
    {
        zSW_neg.lastDebounceTime = now;
        GPIO_PinState new_state = HAL_GPIO_ReadPin(zSW_neg.port, zSW_neg.pin);
        if (zSW_neg.Pin_state != new_state)
        {
            zSW_neg.Pin_state = new_state;
            if (new_state)
            {
                motorZ.isMoving = 0;
                LOG_INFO("Z- Engaged");
            }
            else
            {
                LOG_INFO("Z- Disengaged");
            }
        }
    }
    HAL_GPIO_EXTI_IRQHandler(zSW_neg.pin);
}
