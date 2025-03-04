// Define to prevent recursive inclusion
#ifndef __CONTROLS_H
#define __CONTROLS_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include "main.h"
#include "motor_hal.h"

/**
 * @brief Robot state machine
 *
 */
struct stateMachine
{
    bool faulted;     // Failed health check                     - RED SLOW
    bool unhomed;     // Waiting to home                         - RED SOLID
    bool homing;      // Currently homing                        - RED FAST
    bool waiting;     // Homed and ready for command             - GREEN SOLID
    bool positioning; // Moving above the ground                 - GREEN SLOW
    bool manual;      // Moving w/ controller                    - GREEN STROBE
    bool drilling;    // Moving below the ground or cleaning bit - GREEN FAST

    double y; // Y position of end effector
    double z; // Z position of end effector
};

extern struct stateMachine state;

void locateWeed(double y);
void removeWeed(double y, int drillPower);
void MoveTo(double y, double z);
void MoveBy(double rel_y, double rel_z);
void PrintCartesianCoords(void);
void updateStateMachine(const char *toState);

#endif