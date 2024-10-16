// Define to prevent recursive inclusion
#ifndef __MOTOR_HAL_H
#define __MOTOR_HAL_H

#include "main.h"
#include "limit_switch_hal.h"
#include "stm32f4xx_hal_tim.h"

#define STEPS_PER_REV 6400.0
#define MIN_RPM 5.0
#define MAX_RPM 20.0
#define Z_STEPS_PER_REV 4096.0 // Full stepping
#define Z_MM_PER_REV (15.0 * M_PI)

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
    double reduction;            // Gear reduction of the motor
    double thetaMax;          // Positive limit switch position
    double thetaMin;          // Negative limit switch position
    uint32_t stepsToComplete; // Number of steps the motor has left to complete
    uint32_t stepsToSpeedUp;  // How many steps the motor has to ramp up speed
    uint32_t stepsToSlowDown; // How many steps the motor has to ramp down speed
    double slope;             // The slope betweent the min and target speed
    double currentRPM;        // The motors current rpm
    bool isMoving;            // Is the motor moving?
    LimitSwitch limitSwitch;  // Limit switch associated with the motor

} Motor;

typedef struct
{
    GPIO_TypeDef *pwmPort;
    uint16_t pwmPin;
    float position;       // current position
    float closedPosition; // limit when gripper is closing
    float openPosition;   // limit when gripper is open
    int currentDraw;      // current drawn by servo
    bool isOpen;          // Flag to indicate if serco is open or closed
} ServoMotor;

extern Motor motor1;
extern Motor motor2;
extern Motor motorz;
extern ServoMotor gripper;

void Motors_Init(void);
double MoveByAngle(Motor *motor, double angle, double speedRPM);
double MoveByDist(Motor *motor, double dist, double speedRPM);
void HomeMotors(void);
void StopMotors(void);
void gripperClose(ServoMotor *gripper);
void gripperOpen(ServoMotor *gripper);

#endif