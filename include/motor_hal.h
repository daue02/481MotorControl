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
    GPIO_TypeDef *stepPort;       // Port of the motor
    uint16_t stepPin;             // Pin for stepping
    GPIO_TypeDef *dirPort;        // Port for the direction
    uint16_t dirPin;              // Pin to set direction
    GPIO_TypeDef *sleepPort;      // Port for sleep mode
    uint16_t sleepPin;            // Pin or sleep mode
    bool dir;                     // Motor direction
    double stepsPerRev;           // Motor Resolution * Microstep
    double lead;                  // Leadscrew lead [mm/rev]
    double posMax;                // Positive limit switch position
    double posMin;                // Negative limit switch position
    double speed;                 // Steady-state speed [mm/s]
    uint32_t stepsToComplete;     // Number of steps the motor has left to complete
    uint32_t stepsToCompleteOrig; // Number of steps the motor originally had to complete
    uint32_t accelStep;           // stepsToComplete when motor finishes accelerating
    uint32_t decelStep;           // stepsToComplete when motor begins decelerating
    double slope;                 // The slope betweent the min and target speed
    double currentRPM;            // RPM motor is currently running at
    double targetRPM;             // Steady-state RPM for current move
    bool isMoving;                // Is the motor moving?
    InterruptSwitch posLS;        // Positive limit switch associated with the motor
    InterruptSwitch negLS;        // Positive limit switch associated with the motor
} Motor;

extern Motor motorY;
extern Motor motorZ;

void Motors_Init(void);
double MoveByDist(Motor *motor, double dist, double speedRPM);
void StopMotors(void);
bool motorsMoving(void);
void HomeMotors(void);
void MoveBySpeed(Motor *motor, double speedRPM);

#endif