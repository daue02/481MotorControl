// Define to prevent recursive inclusion
#ifndef __MOTOR_HAL_H
#define __MOTOR_HAL_H

struct StepperMotor
{
    int stepPin;
    int dirPin;
    int stepPerRev;
    int microstep;
    int lead;
    int maxLinSpeed;
    int maxLinAccel;

    // Constructor to initialize motor attributes
    StepperMotor(int sPin, int dPin, int sPerRev, int mStep, int ld, int mLinSpeed, int mLinAccel)
        : stepPin(sPin), dirPin(dPin), stepPerRev(sPerRev), microstep(mStep), lead(ld), maxLinSpeed(mLinSpeed), maxLinAccel(mLinAccel) {}
};

extern StepperMotor motorY;
extern StepperMotor motorZ;

void Motors_Init(void);
void moveMotors(long y, long z);

#endif