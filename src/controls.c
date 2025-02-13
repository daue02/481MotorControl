#include "controls.h"
#include "hmi_hal.h"
#include "limit_switch_hal.h"
#include "motor_hal.h"
#include "utilities.h"

double calculateRPM(Motor *motor);
void PrintState(bool posOnly);

/**
 * @brief Move the robot to the x and y coordinates.
 *
 * @param y coordniate.
 * @param z coordinate.
 */
void MoveTo(double y, double z)
{
    double deltaY = y - state.y;
    double deltaZ = z - state.z;

    if (y > motorY.posMax || y < motorY.posMin || z > motorZ.posMax || z < motorZ.posMin)
    {
        LOG_ERROR("Move is outside of range!");
        ErrorHandler();
    }
    else
    {
        double yRPM = calculateRPM(&motorY);
        double zRPM = calculateRPM(&motorZ);
        deltaY = MoveByDist(&motorY, deltaY, yRPM);
        deltaZ = MoveByDist(&motorZ, deltaZ, zRPM);
        state.y += deltaY;
        state.z += deltaZ;
    }

    // Wait for movement before accepting more
    while (motorsMoving())
    {
        HAL_Delay(1);
    }
    PrintState(true);
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
 * @brief Calculates RPM based on robot state and move distance
 *
 * @param Motor motor to find RPM of
 * @return Resultant RPM
 */
double calculateRPM(Motor *motor)
{
    double linSpeed = 0;

    if (state.drilling)
    {
        linSpeed = motor->drillingSpeed;
    }
    else if (state.homing)
    {
        linSpeed = motor->homingSpeed;
    }
    else if (state.positioning)
    {
        linSpeed = motor->positioningSpeed;
    }
    else
    {
        LOG_ERROR("Move command issued outside of expected state");
        ErrorHandler(); // Robot should only receive move commands in 'positioning', 'homing', or 'drilling' states
    }

    return linSpeed / motor->lead * 60; // Rev/min = mm/s * rev/mm * 60s/min
}

/**
 * @brief Prints state machine vrbs.
 *
 * @param posOnly True if you only want YZ pos
 */
void PrintState(bool posOnly)
{
    LOG_INFO("");
    if (!posOnly)
    {
        LOG_INFO("Robot State:");
        LOG_INFO("Faulted: %s", state.faulted ? "Yes" : "No");
        LOG_INFO("Unhomed: %s", state.homing ? "Yes" : "No");
        LOG_INFO("Homing: %s", state.homing ? "Yes" : "No");
        LOG_INFO("Waiting: %s", state.waiting ? "Yes" : "No");
        LOG_INFO("Positioning: %s", state.positioning ? "Yes" : "No");
        LOG_INFO("Drilling: %s", state.drilling ? "Yes" : "No");
    }
    LOG_INFO("Current YZ Pos [mm]: ");
    PrintCartesianCoords(state.y, state.z);
    LOG_INFO("");
}

/**
 * @brief Prints cartesian coordinates to the serial monitor.
 *
 * @param y coordinate.
 * @param z coordinate.
 */
void PrintCartesianCoords(double y, double z)
{
    int int_part = (int)y;
    int decimal_part = abs((int)((y - int_part) * 1000)); // 3 decimal places
    int int_part2 = (int)z;
    int decimal_part2 = abs((int)((z - int_part2) * 1000)); // 3 decimal places

    LOG_INFO("(%d.%d, %d.%d)", int_part, decimal_part, int_part2, decimal_part2);
}

/**
 *
 * @param toState to: Faulted, Unhomed, Homing, Waiting, Positioning, Drilling
 */
void updateStateMachine(const char *toState)
{
    if (strcmp(toState, "Faulted") == 0)
    {
        state.faulted = 1;
        state.unhomed = 0;
        state.homing = 0;
        state.waiting = 0;
        state.positioning = 0;
        state.drilling = 0;
        changeLEDState(redLED, "Slow");
    }
    else if (strcmp(toState, "Unhomed") == 0)
    {
        state.faulted = 0;
        state.unhomed = 1;
        state.homing = 0;
        state.waiting = 0;
        state.positioning = 0;
        state.drilling = 0;
        changeLEDState(redLED, "Solid");
    }
    else if (strcmp(toState, "Homing") == 0)
    {
        state.faulted = 0;
        state.unhomed = 0;
        state.homing = 1;
        state.waiting = 0;
        state.positioning = 0;
        state.drilling = 0;
        changeLEDState(redLED, "Fast");
    }
    else if (strcmp(toState, "Waiting") == 0)
    {
        state.faulted = 0;
        state.unhomed = 0;
        state.homing = 0;
        state.waiting = 1;
        state.positioning = 0;
        state.drilling = 0;
        changeLEDState(greenLED, "Solid");
    }
    else if (strcmp(toState, "Positioning") == 0)
    {
        state.faulted = 0;
        state.unhomed = 0;
        state.homing = 0;
        state.waiting = 0;
        state.positioning = 1;
        state.drilling = 0;
        changeLEDState(greenLED, "Slow");
    }
    else if (strcmp(toState, "Drilling") == 0)
    {
        state.faulted = 0;
        state.unhomed = 0;
        state.homing = 0;
        state.waiting = 0;
        state.positioning = 0;
        state.drilling = 1;
        changeLEDState(greenLED, "Fast");
    }
}