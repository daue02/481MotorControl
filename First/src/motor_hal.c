#include "motor_hal.h"
#include "controls.h"
#include "limit_switch_hal.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

void Motor_Init(Motor motor);
void StepMotor(Motor *motor);
static void TIM3_Init(void);
static void TIM4_Init(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
uint32_t CalculateMotorSpeed(Motor *motor);

// Motor Objects
Motor motorY = {
    .name = "motorY",
    .stepPort = GPIOB,
    .stepPin = GPIO_PIN_4,
    .dirPort = GPIOB,
    .dirPin = GPIO_PIN_5,
    .sleepPort = GPIOA,
    .sleepPin = GPIO_PIN_8,
    .dir = CCW,
    .stepsPerRev = 3200, // 200PPR * sixteenth-stepping
    .lead = 1,           // NEED TO UPDATE
    .posMin = 0,
    .posMax = 400,
    .isMoving = 0,
};

Motor motorZ = {
    .name = "motorZ",
    .stepPort = GPIOB,
    .stepPin = GPIO_PIN_3,
    .dirPort = GPIOA,
    .dirPin = GPIO_PIN_10,
    .sleepPort = GPIOB,
    .sleepPin = GPIO_PIN_10,
    .dir = CCW,
    .stepsPerRev = 3200, // 200PPR * sixteenth-stepping
    .lead = 1,           // Actual: 5mm
    .posMin = -200,
    .posMax = 100,
    .isMoving = 0,
};

/**
 * @brief Takes in a motor struct and initialized the associated pins.
 *
 * @param motor Motor struct
 */
void Motor_Init(Motor motor)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Initialize Step Pin
    GPIO_InitStruct.Pin = motor.stepPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(motor.stepPort, &GPIO_InitStruct);

    // Initialize Direction Pin
    GPIO_InitStruct.Pin = motor.dirPin;
    HAL_GPIO_Init(motor.dirPort, &GPIO_InitStruct);

    // Initialize sleep pin
    GPIO_InitStruct.Pin = motor.sleepPin;
    HAL_GPIO_Init(motor.sleepPort, &GPIO_InitStruct);
}

/**
 * @brief Initializes both motors and start the timers.
 *
 */
void Motors_Init(void)
{
    Motor_Init(motorY);
    Motor_Init(motorZ);
    TIM3_Init();
    TIM4_Init();

    motorY.limitSwitch = ySW;
    motorZ.limitSwitch = zSW;
}

/**
 * @brief Move the specified motor.
 *
 * @param motor Motor to move
 * @param dist Requested move [mm]
 * @param speedRPM Speed in RPM
 * @return Actual move [mm]
 */
double MoveByDist(Motor *motor, double dist, double speedRPM)
{
    HAL_GPIO_WritePin(motor->sleepPort, motor->sleepPin, 1);

    if (dist > 0)
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CCW);
        motor->dir = CCW;
    }
    else
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CW);
        motor->dir = CW;
        dist = dist * -1;
    }

    motor->stepsToComplete = (uint32_t)(dist / motor->lead * motor->stepsPerRev * 2); // Divide by 2, since each interrupt is a toggle
    motor->accelStep = motor->stepsToComplete * 3 / 4;                                // Spend the first 1/4 of time accelerating
    motor->decelStep = motor->stepsToComplete / 4;                                    // Spend the last 1/4 of time decelerating
    motor->targetRPM = speedRPM;
    uint32_t timerPeriod = CalculateMotorSpeed(motor);
    motor->isMoving = 1;

    double distToComplete = motor->stepsToComplete / motor->stepsPerRev * motor->lead / 2;
    if (motor->dir == CW)
    {
        distToComplete = distToComplete * -1;
    }

    if (motor->name == motorY.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim3, timerPeriod);
        HAL_TIM_Base_Start_IT(&htim3);
        // while (HAL_TIM_Base_GetState(&htim3) != HAL_TIM_STATE_READY)
        // {
        //     // Wait for the timer to be fully initialized. Fixed 2024-10-23
        // }
    }
    else if (motor->name == motorZ.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim4, timerPeriod);
        HAL_TIM_Base_Start_IT(&htim4);
        // while (HAL_TIM_Base_GetState(&htim4) != HAL_TIM_STATE_READY)
        // {
        //     // Wait for the timer to be fully initialized. Fixed 2024-10-23
        // }
    }

    while (HAL_TIM_Base_GetState(&htim3) != HAL_TIM_STATE_READY && HAL_TIM_Base_GetState(&htim4) != HAL_TIM_STATE_READY)
    {
        // Wait for both timers to be fully initialized. Fixed 2024-10-23
    }

    return distToComplete;
}

void StepMotor(Motor *motor)
{
    if (!motor->stepsToComplete || !motor->isMoving)
    {
        if (motor->name == motorY.name)
        {
            HAL_TIM_Base_Stop_IT(&htim3);
            
        }
        else if (motor->name == motorZ.name)
        {
            HAL_TIM_Base_Stop_IT(&htim4);
        }
        motor->isMoving = 0;
        HAL_GPIO_WritePin(motor->sleepPort, motor->sleepPin, 0);
    }
    HAL_GPIO_TogglePin(motor->stepPort, motor->stepPin);
    motor->stepsToComplete--;
}

static void TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1); // 1 MHz clock
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF; // Max value, update frequency will be set in stepMotor()
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim3);

    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
            StepMotor(&motorY);
            uint32_t timerPeriod = CalculateMotorSpeed(&motorY);
            __HAL_TIM_SET_AUTORELOAD(&htim3, timerPeriod);
        }
    }
}

static void TIM4_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = (uint32_t)((SystemCoreClock / 1000000) - 1); // 1 MHz clock
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xFFFF; // Max value, update frequency will be set in stepMotor()
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim4);

    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
            StepMotor(&motorZ);
            uint32_t timerPeriod = CalculateMotorSpeed(&motorZ);
            __HAL_TIM_SET_AUTORELOAD(&htim4, timerPeriod);
        }
    }
}

/**
 * @brief Motor acceleration / deceleration copntrol
 *
 * @param motor Motor to move
 * @return current timer period to achieve accel/decel/RPM
 */
uint32_t CalculateMotorSpeed(Motor *motor)
{
    if (motor->stepsToComplete > motor->accelStep)
    {
        double slope = 1.0 - ((double)(motor->stepsToComplete - motor->accelStep) / (motor->accelStep / 3.0));
        motor->currentRPM = MIN_RPM + slope * (motor->targetRPM - MIN_RPM);
        // printf("Slope: %d || RPM: %d\n",(int)slope,(int)motor->currentRPM);
    }
    else if (motor->stepsToComplete < motor->decelStep)
    {
        double slope = ((double)motor->decelStep - motor->stepsToComplete) / motor->decelStep;
        motor->currentRPM = motor->targetRPM - slope * (motor->targetRPM - MIN_RPM);
        // printf("Slope: %d || RPM: %d\n",(int)slope,(int)motor->currentRPM);
    }

    float timePerStep = 60.0 / (motor->currentRPM * motor->stepsPerRev); // Time per step in seconds
    uint32_t timerPeriod = (uint32_t)((timePerStep * 1000000) / 2) - 1;  // Time per toggle, in microseconds

    return timerPeriod;
}

/**
 * @brief Will stop all motors immediately.
 *
 */
void StopMotors(void)
{
    HAL_TIM_Base_Stop_IT(&htim3);
    HAL_TIM_Base_Stop_IT(&htim4);
}

/**
 * @brief Homes the motors.
 *
 */
void HomeMotors(void)
{
    printf("Homing...\n");
    updateStateMachine("Homing");

    // Move full left/up until LS contact
    MoveTo(motorY.posMin, motorZ.posMin, 120, 120);
    while (motorY.isMoving || motorZ.isMoving)
    {
        HAL_Delay(1);
    }
    HAL_Delay(1000);

    // Move right/down by 5mm
    MoveBy(-1 * (motorY.posMax - motorY.posMin), -1 * (motorZ.posMax - motorZ.posMin), 250, 250);
    while (motorY.isMoving || motorZ.isMoving)
    {
        HAL_Delay(1);
    }

    // Update the state machine
    updateStateMachine("Idle");
    state.y = motorY.posMin + 5;
    state.z = motorZ.posMin + 5;
    PrintState(true);
}