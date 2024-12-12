#include "controls.h"
#include "motor_hal.h"
#include "limit_switch_hal.h"
#include "hmi_hal.h"

/**
 * @brief Prints state machine vrbs.
 *
 * @param posOnly True if you only want YZ pos
 */
void PrintState(bool posOnly)
{
    printf("\n");
    if (!posOnly)
    {
        printf("Robot State:\n");
        printf("Faulted: %s\n", state.faulted ? "Yes" : "No");
        printf("Homed: %s\n", state.homed ? "Yes" : "No");
        printf("Homing: %s\n", state.homing ? "Yes" : "No");
        printf("Positioning: %s\n", state.positioning ? "Yes" : "No");
        printf("Drilling: %s\n", state.drilling ? "Yes" : "No");
    }
    printf("Current YZ Pos [mm]: ");
    PrintCartesianCoords(state.y, state.z);
    printf("\n");
}

/**
 * @brief Move the robot to the x and y coordinates.
 *
 * @param y coordniate.
 * @param z coordinate.
 * @param yRPM motor speed
 * @param zRPM motor speed
 */
void MoveTo(double y, double z, double yRPM, double zRPM)
{
    double deltaY = y - state.y;
    double deltaZ = z - state.z;

    if (y > motorY.posMax || y < motorY.posMin || z > motorZ.posMax || z < motorZ.posMin)
    {
        printf("Move is outside of range!\n");
    }
    else
    {
        deltaY = MoveByDist(&motorY, deltaY, yRPM);
        deltaZ = MoveByDist(&motorZ, deltaZ, zRPM);
        state.y += deltaY;
        state.z += deltaZ;
    }

    PrintState(true);
}

/**
 * @brief Increment the robots current position.
 *
 * @param rel_y y increment.
 * @param rel_z z increment
 * @param yRPM motor speed
 * @param zRPM motor speed
 */
void MoveBy(double rel_y, double rel_z, double yRPM, double zRPM)
{
    double new_y = state.y + rel_y;
    double new_z = state.z + rel_z;

    MoveTo(new_y, new_z, yRPM, zRPM);
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

    printf("(%d.%d, %d.%d)\n", int_part, decimal_part, int_part2, decimal_part2);
}

/**
 * @brief Update the state machine following an event
 *
 * @param toState to: Unhomed, Homing, Faulted, Manual, Auto Wait, Auto Move
 */
void updateStateMachine(const char *toState)
{
    if (strcmp(toState, "Unhomed") == 0)
    {
        state.faulted = 0;
        state.homed = 0;
        state.homing = 0;
        state.positioning = 0;
        state.drilling = 0;
        changeLEDState(redLED, "Solid");
    }
    else if (strcmp(toState, "Homing") == 0)
    {
        state.faulted = 0;
        state.homed = 0;
        state.homing = 1;
        state.positioning = 0;
        state.drilling = 0;
        changeLEDState(redLED, "Slow");
    }
    else if (strcmp(toState, "Faulted") == 0)
    {
        state.faulted = 1;
        state.homed = 0;
        state.homing = 0;
        state.positioning = 0;
        state.drilling = 0;
        changeLEDState(redLED, "Fast");
    }
    else if (strcmp(toState, "Positioning") == 0)
    {
        state.faulted = 0;
        state.homed = 1;
        state.homing = 0;
        state.positioning = 1;
        state.drilling = 0;
        changeLEDState(greenLED, "Slow");
    }
    else if (strcmp(toState, "Drilling") == 0)
    {
        state.faulted = 0;
        state.homed = 1;
        state.homing = 0;
        state.positioning = 0;
        state.drilling = 1;
        changeLEDState(greenLED, "Fast");
    }
    else if (strcmp(toState, "Idle") == 0)
    {
        state.faulted = 0;
        state.homed = 1;
        state.homing = 0;
        state.positioning = 0;
        state.drilling = 0;
        changeLEDState(greenLED, "Solid");
    }
}