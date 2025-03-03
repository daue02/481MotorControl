#include "controls.h"
#include "drill_hal.h"
#include "limit_switch_hal.h"
#include "motor_hal.h"
#include "uart.h"
#include "utilities.h"

// Define common intermediate locations
#define Z_SAFE -25.0

void MoveTo(double y, double z);
void checkMoveIsValid(double y, double z);

/**
 * @brief Moves the y axis into position over weed, lowers drill to near-ground
 *
 * @param y coordinate where weed is located
 */
void locateWeed(double y)
{
    updateStateMachine("Moving");
    MoveTo(y, Z_SAFE);
}

/**
 * @brief Drill out weed then retract z axis
 *
 * @param y coordinate where weed is located
 * @param drillPower % power (0-100) for drill
 */
void removeWeed(double y, int drillPower)
{
    updateStateMachine("Drilling");
    setDrillPower(drillPower);
    MoveTo(y, motorZ.posMax);
    setDrillPower(0);
    updateStateMachine("Moving");
    MoveTo(y, motorZ.posMin);
    motorOperationCompleteCallback();
    updateStateMachine("Waiting");
}

/**
 * @brief Move the robot to the x and y coordinates.
 *
 * @param y coordniate.
 * @param z coordinate.
 */
void MoveTo(double y, double z)
{
    checkMoveIsValid(y, z);

    double deltaY = y - state.y;
    double deltaZ = z - state.z;
    double yRPM = motorY.speed / motorY.lead * 60; // Rev/min = mm/s * rev/mm * 60s/min
    double zRPM = motorZ.speed / motorZ.lead * 60; // Rev/min = mm/s * rev/mm * 60s/min
    deltaY = MoveByDist(&motorY, deltaY, yRPM);
    deltaZ = MoveByDist(&motorZ, deltaZ, zRPM);
    state.y += deltaY;
    state.z += deltaZ;

    // Wait for movement before accepting more
    while (motorsMoving())
    {
        HAL_Delay(1);
    }

    PrintCartesianCoords();
}

/**
 * @brief Increment the robots current position.
 *
 * @param rel_y y increment.
 * @param rel_z z increment
 */
void MoveBy(double rel_y, double rel_z)
{
    double new_y = state.y + rel_y;
    double new_z = state.z + rel_z;

    MoveTo(new_y, new_z);
}

/**
 * @brief Checks the requested move is valid/safe
 *
 * @param y coordniate.
 * @param z coordinate.
 */
void checkMoveIsValid(double y, double z)
{
    if (y > motorY.posMax)
    {
        LOG_ERROR("Y movement exceeds maximum range");
    }
    else if (y < motorY.posMin)
    {
        LOG_ERROR("Y movement exceeds minimum range");
    }
    else if (z > motorZ.posMax)
    {
        LOG_ERROR("Z movement exceeds maximum range");
    }
    else if (z < motorZ.posMin)
    {
        LOG_ERROR("Z movement exceeds minimum range");
    }
    else if (y != state.y && z > Z_SAFE)
    {
        LOG_ERROR("Y cannot be moved when Z is near or below ground");
    }
    else
    {
        return;
    }
    ErrorHandler();
}

/**
 * @brief Prints cartesian coordinates to the serial monitor.
 *
 */
void PrintCartesianCoords(void)
{
    double y = state.y;
    double z = state.z;

    int int_part = (int)y;
    int decimal_part = abs((int)((y - int_part) * 1000)); // 3 decimal places
    int int_part2 = (int)z;
    int decimal_part2 = abs((int)((z - int_part2) * 1000)); // 3 decimal places

    LOG_INFO("");
    LOG_INFO("Current YZ Pos [mm]: ");
    LOG_INFO("(%d.%d, %d.%d)", int_part, decimal_part, int_part2, decimal_part2);
    LOG_INFO("");
}

/**
 *
 * @param toState to: Faulted, Unhomed, Homing, Waiting, Moving, Drilling
 */
void updateStateMachine(const char *toState)
{
    if (strcmp(toState, "Faulted") == 0)
    {
        motorY.speed = 0; // Motion not permitted when faulted
        motorZ.speed = 0; // Motion not permitted when faulted
        changeLEDState(redLED, "Slow");
    }
    else if (strcmp(toState, "Unhomed") == 0)
    {
        motorY.speed = 0; // Motion not permitted when unhomed
        motorZ.speed = 0; // Motion not permitted when unhomed
        changeLEDState(redLED, "Solid");
    }
    else if (strcmp(toState, "Homing") == 0)
    {
        motorY.speed = 35;
        motorZ.speed = 35;
        changeLEDState(redLED, "Fast");
    }
    else if (strcmp(toState, "Waiting") == 0)
    {
        motorY.speed = 0;
        motorZ.speed = 0;
        changeLEDState(greenLED, "Solid");
    }
    else if (strcmp(toState, "Positioning") == 0)
    {
        motorY.speed = 100;
        motorZ.speed = 100;
        changeLEDState(greenLED, "Slow");
    }
    else if (strcmp(toState, "Manual") == 0)
    {
        motorY.speed = 100;
        motorZ.speed = 100;
        changeLEDState(greenLED, "Strobe");
    }
    else if (strcmp(toState, "Drilling") == 0)
    {
        motorY.speed = 0; // Y motion not allowed when below ground
        motorZ.speed = 10;
        changeLEDState(greenLED, "Fast");
    }
    else
    {
        LOG_ERROR("Invalid State Commanded");
        ErrorHandler();
    }
}