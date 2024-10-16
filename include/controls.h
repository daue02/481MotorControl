// Define to prevent recursive inclusion
#ifndef CONTROLS_H
#define CONTROLS_H

#include "motor_hal.h"
#include "main.h"

struct stateMachine
{
    bool homed;

    double yPos;
    double zPos;

    // Constructor to initialize state machine attributes
    stateMachine(bool home, double y, double z)
        : homed(home), yPos(y), zPos(z) {}
};

extern stateMachine state;

void moveTo(double y, double z);
void printState();

#endif