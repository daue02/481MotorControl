#include "limit_switch_hal.h"
#include "motor_hal.h"
#include "main.h"
#include "controls.h"

void EXTI9_5_IRQHandler(void);
void Switch_Init(InterruptSwitch *interruptSW);

InterruptSwitch ySW_pos =
    {
        .name = "ySwitchPos",
        .port = GPIOB,
        .pin = GPIO_PIN_8,
};

InterruptSwitch ySW_neg =
    {
        .name = "ySwitchNeg",
        .port = GPIOB,
        .pin = GPIO_PIN_9,
};

InterruptSwitch zSW_pos =
    {
        .name = "zSwitchPos",
        .port = GPIOB,
        .pin = GPIO_PIN_5,
};

InterruptSwitch zSW_neg =
    {
        .name = "zSwitchNeg",
        .port = GPIOB,
        .pin = GPIO_PIN_6,
};

/*
InterruptSwitch piSW =
    {
        .name = "piSwitch",
        .port = GPIOB,
        .pin = GPIO_PIN_7,
};
*/

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
    /*
    if (interruptSW->name == piSW.name)
    {
        GPIO_InitStruct.Pull = GPIO_PULLDOWN; // Not on a debounce circuit like the other switches
    }
    else
    {
        GPIO_InitStruct.Pull = GPIO_NOPULL;
    }
    */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(interruptSW->port, &GPIO_InitStruct);

    interruptSW->Pin_state = HAL_GPIO_ReadPin(interruptSW->port, interruptSW->pin);
}

/**
 * @brief Initializes all the limit switched, called in main.
 *
 */
void Limit_Switch_Init(void)
{
    Switch_Init(&ySW_pos);
    Switch_Init(&ySW_neg);
    Switch_Init(&zSW_pos);
    Switch_Init(&zSW_neg);
    // Switch_Init(&piSW);

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
    GPIO_PinState yPin_p_state = HAL_GPIO_ReadPin(ySW_pos.port, ySW_pos.pin);
    GPIO_PinState yPin_n_state = HAL_GPIO_ReadPin(ySW_neg.port, ySW_neg.pin);

    GPIO_PinState zPin_p_state = HAL_GPIO_ReadPin(zSW_pos.port, zSW_pos.pin);
    GPIO_PinState zPin_n_state = HAL_GPIO_ReadPin(zSW_neg.port, zSW_neg.pin);

    // GPIO_PinState piSW_state = HAL_GPIO_ReadPin(piSW.port, piSW.pin);

    if (ySW_pos.Pin_state != yPin_p_state)
    {
        // Switch is open and limit switch is being engaged
        if (yPin_p_state)
        {
            motorY.isMoving = 0;
            LOG_INFO("Y+ Engaged");
        }
        // Switch is closed and limit switch is not engaged
        else
        {
            LOG_INFO("Y+ Disengaged");
        }
        HAL_GPIO_EXTI_IRQHandler(ySW_pos.pin);
        ySW_pos.Pin_state = yPin_p_state;
    }
    else if (ySW_neg.Pin_state != yPin_n_state)
    {
        // Switch is open and limit switch is being engaged
        if (yPin_n_state)
        {
            motorY.isMoving = 0;
            LOG_INFO("Y- Engaged");
        }
        // Switch is closed and limit switch is not engaged
        else
        {
            LOG_INFO("Y- Disengaged");
        }
        HAL_GPIO_EXTI_IRQHandler(ySW_neg.pin);
        ySW_neg.Pin_state = yPin_n_state;
    }

    if (zSW_pos.Pin_state != zPin_p_state)
    {
        // Switch is open and limit switch is being engaged
        if (zPin_p_state)
        {
            motorZ.isMoving = 0;
            LOG_INFO("Z+ Engaged");
        }
        // Switch is closed and limit switch is not engaged
        else
        {
            LOG_INFO("Z+ Disengaged");
        }
        HAL_GPIO_EXTI_IRQHandler(zSW_pos.pin);
        zSW_pos.Pin_state = zPin_p_state;
    }
    else if (zSW_neg.Pin_state != zPin_n_state)
    {
        // Switch is open and limit switch is being engaged
        if (zPin_n_state)
        {
            motorZ.isMoving = 0;
            LOG_INFO("Z- Engaged");
        }
        // Switch is closed and limit switch is not engaged
        else
        {
            LOG_INFO("Z- Disengaged");
        }
        HAL_GPIO_EXTI_IRQHandler(zSW_neg.pin);
        zSW_neg.Pin_state = zPin_n_state;
    }

    /*
    if (piSW.Pin_state != piSW_state)
    {
        // Pi starts sending interrupt
        if (piSW_state)
        {
            ErrorHandler();
            LOG_ERROR("Interrupt received from Pi");
        }
        // Pi stops sending interrupt
        else
        {
            LOG_INFO("Interrupt from Pi cancelled");
        }
        HAL_GPIO_EXTI_IRQHandler(piSW.pin);
        piSW.Pin_state = piSW_state;
    }
    */
}