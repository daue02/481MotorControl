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
    .stepPin = GPIO_PIN_3,
    .dirPort = GPIOA,
    .dirPin = GPIO_PIN_10,
    .sleepPort = GPIOB,
    .sleepPin = GPIO_PIN_4,
    .dir = CCW,
    .stepsPerRev = 200, // 200PPR * no microstep
    .lead = 8,
    .posMin = 0,
    .posMax = 219.62, // See 2025-02-11 Electrical OneNote (ED)
    .isMoving = 0,
};

Motor motorZ = {
    .name = "motorZ",
    .stepPort = GPIOA,
    .stepPin = GPIO_PIN_7,
    .dirPort = GPIOB,
    .dirPin = GPIO_PIN_10,
    .sleepPort = GPIOA,
    .sleepPin = GPIO_PIN_6,
    .dir = CCW,
    .stepsPerRev = 200, // 200PPR
    .lead = 5,
    .posMin = -93.2, // See 2025-02-11 Electrical OneNote (ED)
    .posMax = 113.1, // See 2025-02-11 Electrical OneNote (ED)
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

    // Step Pin
    GPIO_InitStruct.Pin = motor.stepPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(motor.stepPort, &GPIO_InitStruct);

    // Direction Pin
    GPIO_InitStruct.Pin = motor.dirPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(motor.dirPort, &GPIO_InitStruct);

    // Sleep pin
    GPIO_InitStruct.Pin = motor.sleepPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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

    motorY.posLS = ySW_pos;
    motorY.negLS = ySW_neg;
    motorZ.posLS = zSW_pos;
    motorZ.negLS = zSW_neg;
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
    motor->stepsToCompleteOrig = motor->stepsToComplete;
    double accelTime = 0.25;                                                                    // Time to accelerate/decelerate in seconds - 21NOV - OFF BY A FACTOR OF 4
    double nominalTime = (double)motor->stepsToComplete / motor->stepsPerRev / speedRPM * 60.0; // Time to move at cnst speed

    if (accelTime * 2 > nominalTime)
    {
        accelTime = nominalTime / 2;
    }
    motor->accelStep = motor->stepsToComplete - speedRPM * motor->stepsPerRev / 60 * accelTime; // Remaining steps when acceleration is complete
    motor->decelStep = speedRPM * motor->stepsPerRev / 60 * accelTime;                          // Remaining steps when deceleration begins

    // if (motor->name == motorY.name)
    // {
    //     LOG_INFO("Steps to Complete: %d", motor->stepsToComplete);
    //     LOG_INFO("stepsPerRev: %d", (int)motor->stepsPerRev);
    //     LOG_INFO("speedRPM: %d", (int)speedRPM);
    //     LOG_INFO("Nominal Time: %d", (int)nominalTime);
    //     LOG_INFO("Accel Step: %d", (int)motor->accelStep);
    //     LOG_INFO("Decel Step: %d", (int)motor->decelStep);
    // }

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
        motor->stepsToCompleteOrig = 0;
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
        double slope = 1.0 - ((double)(motor->stepsToComplete - motor->accelStep) / (motor->stepsToCompleteOrig - motor->accelStep));
        motor->currentRPM = MIN_RPM + slope * (motor->targetRPM - MIN_RPM);
    }
    else if (motor->stepsToComplete < motor->decelStep)
    {
        double slope = 1.0 - ((double)motor->decelStep - motor->stepsToComplete) / motor->decelStep;
        motor->currentRPM = MIN_RPM + slope * (motor->targetRPM - MIN_RPM);
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
 * @brief Checks if either motor is moving
 * @return True if either is moving
 *
 */
bool motorsMoving(void)
{
    if (motorY.isMoving || motorZ.isMoving)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Homes the motors.
 *
 */
void HomeMotors(void)
{
    LOG_INFO("Homing...");
    updateStateMachine("Homing");

    // Use these lines to set current limits on initial bootup
    // HAL_GPIO_WritePin(motorY.sleepPort, motorY.sleepPin, 1);
    // HAL_GPIO_WritePin(motorZ.sleepPort, motorZ.sleepPin, 1);

    // Set positions to max so motor is allowed to move in min direction
    state.y = motorY.posMax;
    state.z = motorZ.posMax;

    // Move full left/up until LS contact
    MoveTo(motorY.posMin, motorZ.posMin);
    while (motorsMoving())
    {
        HAL_Delay(1);
    }
    HAL_Delay(1000);

    // Move right/down by 5mm
    MoveBy(5, 5);
    while (motorsMoving())
    {
        HAL_Delay(1);
    }

    // Update min position to avoid limit switch contact
    motorY.posMin += 6.21; // As measured for 5mm command 12-FEB-2025 ED
    motorZ.posMin += 7.21; // As measured for 5mm command 12-FEB-2025 ED

    state.y = motorY.posMin;
    state.z = motorZ.posMin;

    updateStateMachine("Waiting");
}

/**
 * @brief Moves the motor at a given speed indefinitely until stopped.
 *
 * @param motor Motor to move
 * @param speedRPM Speed in RPM
 */
void MoveBySpeed(Motor *motor, double speedRPM)
{
    HAL_GPIO_WritePin(motor->sleepPort, motor->sleepPin, 1);

    if (speedRPM > 0)
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CCW);
        motor->dir = CCW;
    }
    else
    {
        HAL_GPIO_WritePin(motor->dirPort, motor->dirPin, CW);
        motor->dir = CW;
        speedRPM = -speedRPM; // Ensure speed is positive for calculations
    }

    motor->targetRPM = speedRPM;
    uint32_t timerPeriod = CalculateMotorSpeed(motor);
    motor->isMoving = 1;
    motor->stepsToComplete = UINT32_MAX; // Run indefinitely
    motor->accelStep = motor->stepsToComplete - speedRPM * motor->stepsPerRev / 60 / 20;

    if (motor->name == motorY.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim3, timerPeriod);
        HAL_TIM_Base_Start_IT(&htim3);
    }
    else if (motor->name == motorZ.name)
    {
        __HAL_TIM_SET_AUTORELOAD(&htim4, timerPeriod);
        HAL_TIM_Base_Start_IT(&htim4);
    }
}