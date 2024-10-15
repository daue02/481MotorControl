#include <Arduino.h>
#include <accelstepper.h>

// Define pin connections & motor's steps per revolution
const int stepYPin = 3;
const int dirYPin = 6;

const int stepZPin = 4;
const int dirZPin = 7;

// Define drive ratios
const int stepPerRevY = 200;
const int microstepY = 16;
const int leadY = 2; // in mm

const int stepPerRevZ = 200;
const int microstepZ = 16;
const int leadZ = 2; // in mm

// Define performance criteria
const int maxLinSpeedY = 40; // mm/s
const int maxLinAccelY = 50; // mm/s^2

const int maxLinSpeedZ = 40; // mm/s
const int maxLinAccelZ = 50; // mm/s^2

// Define max speeds
const int maxStepSpeedY = maxLinSpeedY / leadY * stepPerRevY * microstepY; // step/s
const int maxStepAccelY = maxLinAccelY / leadY * stepPerRevY * microstepY; // step/s^2

const int maxStepSpeedZ = maxLinSpeedZ / leadZ * stepPerRevZ * microstepZ; // step/s
const int maxStepAccelZ = maxLinAccelZ / leadZ * stepPerRevZ * microstepZ; // step/s^2

// Create motor objects
AccelStepper stepperY(AccelStepper::DRIVER, stepYPin, dirYPin);
AccelStepper stepperZ(AccelStepper::DRIVER, stepZPin, dirZPin);

void setup()
{
  stepperY.setMaxSpeed(maxStepSpeedY);
  stepperY.setAcceleration(maxStepAccelY); // May not need here - not a max

  stepperZ.setMaxSpeed(maxStepSpeedZ);
  stepperZ.setAcceleration(maxStepAccelZ); // May not need here - not a max

  stepperY.move(-3200); // Equivalent to 1 rotation CCW
  stepperZ.move(3200);  // Equivalent to 1 rotation CW
}

void loop()
{
  if (stepperY.distanceToGo() != 0)
  {
    stepperY.run(); // Step the motor one step
  }

  if (stepperZ.distanceToGo() != 0)
  {
    stepperZ.run(); // Step the motor one step
  }
}