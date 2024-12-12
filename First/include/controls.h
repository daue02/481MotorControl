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
    bool faulted;     //  Is the robot stuck in a fault state (Ex. Failed Health Check)
    bool homed;       // Has the robot been homed
    bool homing;      // Is the robot currently homing
    bool positioning; // Are the YZ motors running
    bool drilling;    // Is the drill motor running

    double y; // Y position of end effector
    double z; // Z position of end effector
};

extern struct stateMachine state;

void PrintState(bool posOnly);
void MoveTo(double y, double z, double yRPM, double zRPM);
void MoveBy(double rel_y, double rel_z, double yRPM, double zRPM);
void PrintCartesianCoords(double y, double z);
void updateStateMachine(const char *toState);

#endif