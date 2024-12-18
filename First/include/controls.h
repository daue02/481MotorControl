// Define to prevent recursive inclusion
#ifndef __CONTROLS_H
#define __CONTROLS_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include "motor_hal.h"
#include "main.h"

#define LINK_1 85.0  // mm
#define LINK_2 190.0 // mm
#define SAFETY_MARGIN (3.0 / 180.0 * M_PI)
#define Z_SAFETY_MARGIN 5.0

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
    bool drilling;    // Moving below the ground or cleaning bit - GREEN FAST

    double y; // Y position of end effector
    double z; // Z position of end effector
};

extern struct stateMachine state;

void PrintState(bool posOnly);
void MoveTo(double y, double z);
void MoveBy(double rel_y, double rel_z);
void PrintCartesianCoords(double y, double z);
void updateStateMachine(const char *toState);

#endif