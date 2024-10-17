// Define to prevent recursive inclusion
#ifndef __MOTOR_HAL_H
#define __MOTOR_HAL_H

#include "main.h"
#include "limit_switch_hal.h"
#include "stm32f4xx_hal_tim.h"

#define STEPS_PER_REV 6400.0
#define MIN_RPM 5.0
#define MAX_RPM 100.0

#define CW 0
#define CCW 1

typedef struct
{
    const char *name;
    GPIO_TypeDef *stepPort;   // Port of the motor
    uint16_t stepPin;         // Pin for stepping
    GPIO_TypeDef *dirPort;    // Port for the direction
    uint16_t dirPin;          // Pin to set direction
    bool dir;                 // Motor direction
    double stepsPerRev;       // Motor Resolution * Microstep
    double lead;              // Leadscrew lead [mm/rev]
    double posMax;            // Positive limit switch position
    double posMin;            // Negative limit switch position
    uint32_t stepsToComplete; // Number of steps the motor has left to complete
    uint32_t stepsToSpeedUp;  // How many steps the motor has to ramp up speed
    uint32_t stepsToSlowDown; // How many steps the motor has to ramp down speed
    double slope;             // The slope betweent the min and target speed
    double currentRPM;        // The motors current rpm
    bool isMoving;            // Is the motor moving?
    LimitSwitch limitSwitch;  // Limit switch associated with the motor
} Motor;

extern Motor motorY;
extern Motor motorZ;

void Motors_Init(void);
double MoveByDist(Motor *motor, double dist, double speedRPM);
void HomeMotors(void);
void StopMotors(void);

#endif