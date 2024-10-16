#include <Arduino.h>
#include <accelstepper.h>
#include "motor_hal.h"
#include "controls.h"

// Create my motor objects
StepperMotor motorY(3, 6, 200, 16, 1, 50, 50); // 40 32
StepperMotor motorZ(4, 7, 200, 16, 1, 50, 50); // 50 50

// Create accelStepper motor objects
AccelStepper stepperY(AccelStepper::DRIVER, motorY.stepPin, motorY.dirPin);
AccelStepper stepperZ(AccelStepper::DRIVER, motorZ.stepPin, motorZ.dirPin);

void Motor_Init(StepperMotor motor, AccelStepper &stepper)
{
    int maxStepSpeed = motor.maxLinSpeed / motor.lead * motor.stepPerRev * motor.microstep;
    int maxStepAccel = motor.maxLinAccel / motor.lead * motor.stepPerRev * motor.microstep;

    stepper.setMaxSpeed(maxStepSpeed);
    stepper.setAcceleration(maxStepAccel);
    stepper.setSpeed(maxStepSpeed);
}

/**
 * @brief Initialize Y & Z steppers
 */
void Motors_Init(void)
{
    Motor_Init(motorY, stepperY);
    Motor_Init(motorZ, stepperZ);
}

/**
 * @brief Move by a number of steps
 *
 * @param y Y steps to move (+ = CW)
 * @param z Z steps to move (+ = CW)
 */
void moveMotors(long y, long z)
{
    stepperY.move(y);
    stepperY.runToPosition();
    stepperZ.move(z);
    stepperZ.runToPosition();
}
