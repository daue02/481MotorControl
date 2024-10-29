// Define to prevent recursive inclusion
#ifndef __MOTOR_HAL_H
#define __MOTOR_HAL_H

#include "main.h"
#include "limit_switch_hal.h"
#include "stm32f4xx_hal_tim.h"

#define MIN_RPM 5.0

#define CW 0
#define CCW 1

typedef struct
{
    const char *name;
    GPIO_TypeDef *stepPort;   // Port of the motor
    uint16_t stepPin;         // Pin for stepping
    GPIO_TypeDef *dirPort;    // Port for the direction
    uint16_t dirPin;          // Pin to set direction
    GPIO_TypeDef *sleepPort;  // Port for sleep mode
    uint16_t sleepPin;        // Pin or sleep mode
    bool dir;                 // Motor direction
    double stepsPerRev;       // Motor Resolution * Microstep
    double lead;              // Leadscrew lead [mm/rev]
    double posMax;            // Positive limit switch position
    double posMin;            // Negative limit switch position
    uint32_t stepsToComplete; // Number of steps the motor has left to complete
    uint32_t accelStep;       // stepsToComplete when motor finishes accelerating
    uint32_t decelStep;       // stepsToComplete when motor begins decelerating
    double slope;             // The slope betweent the min and target speed
    double currentRPM;        // The motors current rpm
    double targetRPM;         // The steady-state target rpm
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