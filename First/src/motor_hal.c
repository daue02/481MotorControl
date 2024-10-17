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

// Motor Objects
Motor motorY = {
    .name = "motorY",
    .stepPort = GPIOA,      // D2-PA10
    .stepPin = GPIO_PIN_10, // D2-PA10
    .dirPort = GPIOB,       // D5-PB4
    .dirPin = GPIO_PIN_4,   // D5-PB4
    .dir = CCW,
    .stepsPerRev = 3200, // 200PPR * sixteenth-stepping
    .lead = 1,           // NEED TO UPDATE
    .posMin = 0,
    .posMax = 400,
    .isMoving = 0,
};

Motor motorZ = {
    .name = "motorZ",
    .stepPort = GPIOB,     // D3-PB3
    .stepPin = GPIO_PIN_3, // D3-PB3
    .dirPort = GPIOB,      // D6-PB10
    .dirPin = GPIO_PIN_10, // D6-PB10
    .dir = CCW,
    .stepsPerRev = 3200, // 200PPR * sixteenth-stepping
    .lead = 1,           // NEED TO UPDATE
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
    motor->isMoving = 1;
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

    // Gain scheduling setup
    motor->stepsToComplete = (uint32_t)(dist / motor->lead * motor->stepsPerRev);
    // Speed up for first 1/4 steps
    motor->stepsToSpeedUp = 3.0 / 4.0 * motor->stepsToComplete;
    // Slow down for last 1/4 steps
    motor->stepsToSlowDown = 1.0 / 4.0 * motor->stepsToComplete;
    // RPM delta per step
    motor->slope = (speedRPM - MIN_RPM) / (motor->stepsToSlowDown);
    // Start at the min rpm
    motor->currentRPM = MIN_RPM;

    float timePerStep = 60.0 / (motor->currentRPM * motor->stepsPerRev); // Time per step in seconds
    uint32_t timerPeriod = (uint32_t)((timePerStep * 1000000) / 2) - 1;  // Time per toggle, in microseconds

    double distToComplete = motor->stepsToComplete / motor->stepsPerRev * motor->lead;
    if (motor->dir == CW)
    {
        distToComplete = distToComplete * -1;
    }

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

    return distToComplete;
}

void StepMotor(Motor *motor)
{
    // IsMoving will be set to 0 if a limit switch is engaged
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
        }
    }
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
/*
void HomeMotors(void)
{
    printf("Homing...\n\r");
    updateStateMachine("Homing");

    MoveByAngle(&motor1, 8.0, 5.0);
    MoveByAngle(&motor2, 8.0, 5.0);

    while (motor1.isMoving || motor2.isMoving)
    {
        HAL_Delay(1);
    }
    HAL_Delay(1000);

    // Move back 6 degrees
    double theta1 = MoveByAngle(&motor1, -6.0 / 180.0 * M_PI, 1.0);
    double theta2 = MoveByAngle(&motor2, -6.0 / 180.0 * M_PI, 1.0);

    while (motor1.isMoving || motor2.isMoving)
    {
        HAL_Delay(1);
    }

    // Update the state machine
    updateStateMachine("Auto Wait");
    state.theta1 = motor1.thetaMax + theta1;
    state.theta2 = motor2.thetaMax + theta2;
    CalculateCartesianCoords(state.theta1, state.theta2, &state.x, &state.y);
    printf("Current Coords in x-y:");
    PrintCaresianCoords(state.x, state.y);
*/