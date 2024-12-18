#include "controls.h"
#include "motor_hal.h"
#include "limit_switch_hal.h"
#include "hmi_hal.h"

double calculateRPM(double delta);
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
        printf("Move is outside of range!\n");
        ErrorHandler();
    }
    else
    {
        double yRPM = calculateRPM(deltaY);
        double zRPM = calculateRPM(deltaZ);
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
 * @param delta Distance of the move
 * @return Resultant RPM
 */
double calculateRPM(double delta)
{
    if (state.drilling)
    {
        return 100.0;
    }
    else if (state.positioning)
    {
        if (abs(delta) < 25.0)
        {
            return 150.0;
        }
        else if (abs(delta) < 75)
        {
            return 300.0;
        }
        else
        {
            return 400.0;
        }
    }
    else if (state.homing)
    {
        return 150.0;
    }
    else
    {
        ErrorHandler(); // Robot should only receive move commands in 'positioning', 'homing', or 'drilling' states
        return 0;
    }
}

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
        printf("Unhomed: %s\n", state.homing ? "Yes" : "No");
        printf("Homing: %s\n", state.homing ? "Yes" : "No");
        printf("Waiting: %s\n", state.waiting ? "Yes" : "No");
        printf("Positioning: %s\n", state.positioning ? "Yes" : "No");
        printf("Drilling: %s\n", state.drilling ? "Yes" : "No");
    }
    printf("Current YZ Pos [mm]: ");
    PrintCartesianCoords(state.y, state.z);
    printf("\n");
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