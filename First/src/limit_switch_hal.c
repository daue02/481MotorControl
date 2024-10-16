#include "limit_switch_hal.h"
#include "motor_hal.h"
#include "main.h"
#include "controls.h"

void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void Switch_Init(LimitSwitch *limitSW);

LimitSwitch theta1SW =
    {
        .name = "theta1SW",
        .port = GPIOC,
        .Pin_p = GPIO_PIN_5,
        .Pin_n = GPIO_PIN_6,
};

LimitSwitch theta2SW =
    {
        .name = "theta2SW",
        .port = GPIOC,
        .Pin_p = GPIO_PIN_7,
        .Pin_n = GPIO_PIN_8,
};

LimitSwitch thetazSW =
    {
        .name = "theta3SW",
        .port = GPIOB,
        .Pin_p = GPIO_PIN_13,
        .Pin_n = GPIO_PIN_14,
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
    GPIO_InitStruct.Pull = GPIO_NOPULL;
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
    Switch_Init(&theta1SW);
    Switch_Init(&theta2SW);
    Switch_Init(&thetazSW);

    // Enable and set EXTI line Interrupt to the given priority
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief Pins 5-9 interrupt handler, currently used for limit switches.
 *
 */
void EXTI9_5_IRQHandler(void)
{

    GPIO_PinState theta1Pin_p_state = HAL_GPIO_ReadPin(theta1SW.port, theta1SW.Pin_p);
    GPIO_PinState theta1Pin_n_state = HAL_GPIO_ReadPin(theta1SW.port, theta1SW.Pin_n);

    GPIO_PinState theta2Pin_p_state = HAL_GPIO_ReadPin(theta2SW.port, theta2SW.Pin_p);
    GPIO_PinState theta2Pin_n_state = HAL_GPIO_ReadPin(theta2SW.port, theta2SW.Pin_n);

    if (theta1SW.Pin_p_state != theta1Pin_p_state)
    {
        // Switch is open and limit switch is being engaged
        if (theta1Pin_p_state)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
            motor1.isMoving = 0;
            printf("Theta1+ Engaged\n\r");
        }
        // Switch is closed and limit switch is not engaged
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
            printf("Theta1+ Disengaged\n\r");
        }
        theta1SW.Pin_p_state = theta1Pin_p_state;
        HAL_GPIO_EXTI_IRQHandler(theta1SW.Pin_p);
    }
    else if (theta1SW.Pin_n_state != theta1Pin_n_state)
    {
        // Switch is open and limit switch is being engaged
        if (theta1Pin_n_state)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
            motor1.isMoving = 0;
            printf("Theta1- Engaged\n\r");
        }
        // Switch is closed and limit switch is not engaged
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
            printf("Theta1- Disengaged\n\r");
        }
        theta1SW.Pin_n_state = theta1Pin_n_state;
        HAL_GPIO_EXTI_IRQHandler(theta1SW.Pin_n);
    }

    if (theta2SW.Pin_p_state != theta2Pin_p_state)
    {
        // Switch is open and limit switch is being engaged
        if (theta2Pin_p_state)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
            motor2.isMoving = 0;
            printf("Theta2+ Engaged\n\r");
        }
        // Switch is closed and limit switch is not engaged
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
            printf("Theta2+ Disengaged\n\r");
        }
        theta2SW.Pin_p_state = theta2Pin_p_state;
        HAL_GPIO_EXTI_IRQHandler(theta2SW.Pin_p);
    }
    else if (theta2SW.Pin_n_state != theta2Pin_n_state)
    {
        // Switch is open and limit switch is being engaged
        if (theta2Pin_n_state)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
            motor2.isMoving = 0;
            printf("Theta2- Engaged\n\r");
        }
        // Switch is closed and limit switch is not engaged
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
            printf("Theta2- Disengaged\n\r");
        }
        theta2SW.Pin_n_state = theta2Pin_n_state;
        HAL_GPIO_EXTI_IRQHandler(theta2SW.Pin_n);
    }
}

/**
 * @brief Pins 10-15 interrupt handler, currenlty used for limit switches
 *
 */
void EXTI15_10_IRQHandler(void)
{
    GPIO_PinState thetazPin_p_state = HAL_GPIO_ReadPin(thetazSW.port, thetazSW.Pin_p);
    GPIO_PinState thetazPin_n_state = HAL_GPIO_ReadPin(thetazSW.port, thetazSW.Pin_n);

    if (thetazSW.Pin_p_state != thetazPin_p_state)
    {
        // Switch is open and limit switch is being engaged
        if (thetazPin_p_state)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
            motorz.isMoving = 0;
            printf("Thetaz+ Engaged\n\r");
        }
        // Switch is closed and limit switch is not engaged
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
            printf("Thetaz+ Disengaged\n\r");
        }
        thetazSW.Pin_p_state = thetazPin_p_state;
        HAL_GPIO_EXTI_IRQHandler(thetazSW.Pin_n);
    }
    else if (thetazSW.Pin_n_state != thetazPin_n_state)
    {
        // Switch is open and limit switch is being engaged
        if (thetazPin_n_state)
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
            motorz.isMoving = 0;
            printf("Thetaz- Engaged\n\r");
        }
        // Switch is closed and limit switch is not engaged
        else
        {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
            printf("Thetaz- Disengaged\n\r");
        }
        thetazSW.Pin_n_state = thetazPin_n_state;
        HAL_GPIO_EXTI_IRQHandler(thetazSW.Pin_n);
    }
}