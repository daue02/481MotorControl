#include "controls.h"
#include "drill_hal.h"
#include "limit_switch_hal.h"
#include "motor_hal.h"
#include "uart.h"
#include "utilities.h"

void MoveTo(double y, double z);
void checkMoveIsValid(double y, double z);

/**
 * @brief Moves the y axis into position over weed, lowers drill to near-ground
 *
 * @param y coordinate where weed is located
 */
void locateWeed(double y)
{
    LOG_INFO("Positining for Removal");
    updateStateMachine("Positioning");
    MoveTo(y, 25);
}

/**
 * @brief Drill out weed then retract z axis
 *
 * @param y coordinate where weed is located
 * @param mode 'drill', 'spin', or 'fake'
 */
void removeWeed(double y, const char *mode)
{
    double drillPower = 0;
    double z = 0;

    if (strcmp(mode, "drill") == 0)
    {
        drillPower = 25;
        z = motorZ.posMin;
    }
    else if (strcmp(mode, "spin") == 0)
    {
        drillPower = 25;
        z = -10;
    }
    else if (strcmp(mode, "fake") == 0)
    {
        drillPower = 5;
        z = 0;
    }
    else
    {
        LOG_ERROR("Invalid weed removal mode entered");
        ErrorHandler();
    }

    LOG_INFO("Removing Weed");
    updateStateMachine("Drilling");
    setDrillPower(drillPower, DRILLCCW);
    MoveTo(y, z);
    setDrillPower(0, DRILLCW);
    updateStateMachine("Positioning");
    MoveTo(y, 85);

    // hack
    motorOperationCompleteCallback();
    HAL_Delay(100);
    motorOperationCompleteCallback();
    HAL_Delay(100);
    motorOperationCompleteCallback();

    updateStateMachine("Waiting");
}

/**
 * @brief Move the robot to absoulte (x,y) coordinates
 *
 * @param y coordniate.
 * @param z coordinate.
 */
void MoveTo(double y, double z)
{
    checkMoveIsValid(y, z);

    double deltaY = y - state.y;
    double deltaZ = z - state.z;
    double yRPM = motorY.speed / motorY.lead * 60 * 2; // Rev/min = mm/s * rev/mm * 60s/min. Need to mult by 2 for some reason
    double zRPM = motorZ.speed / motorZ.lead * 60 * 2; // Rev/min = mm/s * rev/mm * 60s/min. Need to mult by 2 for some reason
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
 * @brief Move the robot by relative (x,y) coordinates
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
 * @brief Checks the requested move is within YZ bounds
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

    LOG_INFO("Current YZ Pos [mm]: (%d.%d, %d.%d)", int_part, decimal_part, int_part2, decimal_part2);
}

/**
 *
 * @param toState to: Faulted, Unhomed, Homing, Waiting, Positining, Manual, Drilling
 */
void updateStateMachine(const char *toState)
{
    if (strcmp(toState, "Faulted") == 0)
    {
        motorY.speed = 0; // Motion not permitted when faulted
        motorZ.speed = 0; // Motion not permitted when faulted
        changeLEDState(redLED, "Slow");

        state = (struct stateMachine){.faulted = true, .y = state.y, .z = state.z};
    }
    else if (strcmp(toState, "Unhomed") == 0)
    {
        motorY.speed = 0; // Motion not permitted when unhomed
        motorZ.speed = 0; // Motion not permitted when unhomed
        changeLEDState(redLED, "Solid");

        state = (struct stateMachine){.unhomed = true, .y = state.y, .z = state.z};
    }
    else if (strcmp(toState, "Homing") == 0)
    {
        motorY.speed = 17.5;
        motorZ.speed = 17.5;
        changeLEDState(redLED, "Fast");

        state = (struct stateMachine){.homing = true, .y = state.y, .z = state.z};
    }
    else if (strcmp(toState, "Waiting") == 0)
    {
        motorY.speed = 0;
        motorZ.speed = 0;
        changeLEDState(greenLED, "Solid");

        state = (struct stateMachine){.waiting = true, .y = state.y, .z = state.z};
    }
    else if (strcmp(toState, "Positioning") == 0)
    {
        motorY.speed = 50;
        motorZ.speed = 50;
        changeLEDState(greenLED, "Slow");

        state = (struct stateMachine){.positioning = true, .y = state.y, .z = state.z};
    }
    else if (strcmp(toState, "Manual") == 0)
    {
        motorY.speed = 50;
        motorZ.speed = 50;
        changeLEDState(greenLED, "Strobe");

        state = (struct stateMachine){.manual = true, .y = state.y, .z = state.z};
    }
    else if (strcmp(toState, "Drilling") == 0)
    {
        motorY.speed = 0; // Y motion not allowed when below ground
        motorZ.speed = 5;
        changeLEDState(greenLED, "Fast");

        state = (struct stateMachine){.drilling = true, .y = state.y, .z = state.z};
    }
    else
    {
        LOG_ERROR("Invalid State Commanded");
        ErrorHandler();
    }
}