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
    bool manual;      // Is the robot currently in manual mode (As opposed to automatic)
    bool testRunning; // Is the robot running the test (in auto)
    bool testHasRun;

    bool grasping;   // Is the robot grasping and object
    double theta1;   // Link 1 angle
    double theta2;   // Link 2 angle
    double currentZ; // Current z position of end effector
    double x;        // X position of end effector
    double y;        // Y position of end effector
    double z;        // Z position of end effector
};

extern struct stateMachine state;

void updateStateMachine(const char *toState);
void MoveTo(double x, double y, double rpm);
void MoveBy(double rel_x, double rel_y, double rpm);
void MoveToZ(double z, double rpm);
void PrintAnglesInDegrees(double theta1, double theta2);
void PrintCaresianCoords(double x, double y);
void PrintState(void);
void CalculateCartesianCoords(double theta1, double theta2, double *x, double *y);

void dummy(void);

#endif