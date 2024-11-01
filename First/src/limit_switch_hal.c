#include "limit_switch_hal.h"
#include "motor_hal.h"
#include "main.h"
#include "controls.h"

void EXTI9_5_IRQHandler(void);
void Switch_Init(LimitSwitch *limitSW);

LimitSwitch ySW =
    {
        .name = "ySwitch",
        .port = GPIOB,
        .Pin_p = GPIO_PIN_8,
        .Pin_n = GPIO_PIN_9,
};

LimitSwitch zSW =
    {
        .name = "zSwitch",
        .port = GPIOB,
        .Pin_p = GPIO_PIN_5,
        .Pin_n = GPIO_PIN_6,
};

/**
 * @brief Initializes the pins and state of the limit switch.
 *
 * @param limitSW limit switch object.
 */
void Switch_Init(LimitSwitch *limitSW)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // External interrupt pin configuration
    GPIO_InitStruct.Pin = limitSW->Pin_p | limitSW->Pin_n;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(limitSW->port, &GPIO_InitStruct);

    limitSW->Pin_p_state = HAL_GPIO_ReadPin(limitSW->port, limitSW->Pin_p);
    limitSW->Pin_n_state = HAL_GPIO_ReadPin(limitSW->port, limitSW->Pin_n);
}

/**
 * @brief Initializes all the limit switched, called in main.
 *
 */
void Limit_Switch_Init(void)
{
    Switch_Init(&ySW);
    Switch_Init(&zSW);

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
    GPIO_PinState yPin_p_state = HAL_GPIO_ReadPin(ySW.port, ySW.Pin_p);
    GPIO_PinState yPin_n_state = HAL_GPIO_ReadPin(ySW.port, ySW.Pin_n);

    GPIO_PinState zPin_p_state = HAL_GPIO_ReadPin(zSW.port, zSW.Pin_p);
    GPIO_PinState zPin_n_state = HAL_GPIO_ReadPin(zSW.port, zSW.Pin_n);

    if (ySW.Pin_p_state != yPin_p_state)
    {
        // Switch is open and limit switch is being engaged
        if (yPin_p_state)
        {
            motorY.isMoving = 0;
            printf("Y+ Engaged\n");
        }
        // Switch is closed and limit switch is not engaged
        else
        {
            printf("Y+ Disengaged\n");
        }
        HAL_GPIO_EXTI_IRQHandler(ySW.Pin_p);
        ySW.Pin_p_state = yPin_p_state;
    }
    else if (ySW.Pin_n_state != yPin_n_state)
    {
        // Switch is open and limit switch is being engaged
        if (yPin_n_state)
        {
            motorY.isMoving = 0;
            printf("Y- Engaged\n");
        }
        // Switch is closed and limit switch is not engaged
        else
        {
            printf("Y- Disengaged\n");
        }
        HAL_GPIO_EXTI_IRQHandler(ySW.Pin_n);
        ySW.Pin_n_state = yPin_n_state;
    }

    if (zSW.Pin_p_state != zPin_p_state)
    {
        // Switch is open and limit switch is being engaged
        if (zPin_p_state)
        {
            motorZ.isMoving = 0;
            printf("Z+ Engaged\n");
        }
        // Switch is closed and limit switch is not engaged
        else
        {
            printf("Z+ Disengaged\n");
        }
        HAL_GPIO_EXTI_IRQHandler(zSW.Pin_p);
        zSW.Pin_p_state = zPin_p_state;
    }
    else if (zSW.Pin_n_state != zPin_n_state)
    {
        // Switch is open and limit switch is being engaged
        if (zPin_n_state)
        {
            motorZ.isMoving = 0;
            printf("Z- Engaged\n");
        }
        // Switch is closed and limit switch is not engaged
        else
        {
            printf("Z- Disengaged\n");
        }
        HAL_GPIO_EXTI_IRQHandler(zSW.Pin_n);
        zSW.Pin_n_state = zPin_n_state;
    }
}