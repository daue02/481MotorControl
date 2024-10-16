#include <Arduino.h>
#include <accelstepper.h>
#include "main.h"
#include "motor_hal.h"
#include "controls.h"

// Create state machine
stateMachine state(false, 0, 0);

double mm2steps(double mm, StepperMotor motor)
{
    double steps = 0;
    steps = mm / motor.lead * motor.stepPerRev * motor.microstep;
    return steps;
}

double steps2mm(double steps, StepperMotor motor)
{
    double mm = 0;
    mm = steps / motor.microstep / motor.stepPerRev * motor.lead;
    return mm;
}

double calculateRequiredSteps(double mm, StepperMotor motor)
{
    double desiredSteps = mm2steps(mm, motor);
    long actualSteps = static_cast<int>(desiredSteps + (desiredSteps > 0 ? 0.5 : -0.5));
    return actualSteps;
}

/**
 * @brief Move to a (Y,Z) position [mm]
 *
 * @param y Y distance to move to
 * @param z Z distance to move to
 */
void moveTo(double y, double z)
{
    long yStep = calculateRequiredSteps(y-state.yPos, motorY);
    long zStep = calculateRequiredSteps(z-state.zPos, motorZ);

    moveMotors(yStep, zStep);

    y = steps2mm(yStep, motorY);
    z = steps2mm(zStep, motorZ);

    state.yPos += y;
    state.zPos += z;
}

void printState()
{
    Serial.println("");

    Serial.print("Homed?: ");
    Serial.println(state.homed);

    Serial.print("Y Pos [mm]: ");
    Serial.println(state.yPos);

    Serial.print("Y Pos [mm]: ");
    Serial.println(state.zPos);

    Serial.println("");
}