#include "controls.h"
#include "motor_hal.h"
#include "limit_switch_hal.h"
#include "hmi_hal.h"

// #define DEBUG // Enables serial print statements

void CalculateJointAngle(double x, double y, double solns[2][2]);
double CalculateQuickestValidPath(double cur_theta, double targ_theta, Motor *motor);
bool IsValid(double *soln);

/**
 * @brief Converts cartesian coordinates to joint angles of the robot.
 *
 * @param x coordinate.
 * @param y coordinate.
 * @param solns An array which will contain the set(2) of possible angles.
 */
void CalculateJointAngle(double x, double y, double solns[2][2])
{
    // Calculate length of end effector vector
    double R = sqrt(x * x + y * y);

    // Angle of vector using atan2 to handle quadrants
    double THETA = atan2(y, x);

    // Handle the limits of acos
    double acosarg = (R * R - LINK_1 * LINK_1 - LINK_2 * LINK_2) / (-2 * LINK_1 * LINK_2);
    double beta;
    if (acosarg < -1.0)
    {
        beta = M_PI;
    }
    else if (acosarg > 1.0)
    {
        beta = 0.0;
    }
    else
    {
        beta = acos(acosarg);
    }

    double alpha;
    double break_r = sqrt(LINK_2 * LINK_2 - LINK_1 * LINK_1);
    if (R > break_r)
    {
        alpha = asin((LINK_2 * sin(beta)) / R);
    }
    else if (R > 0.0)
    {
        alpha = M_PI / 2 + (M_PI / 2 - asin((LINK_2 * sin(beta)) / R));
    }
    else
    {
        alpha = 0.0;
    }

    // Assembly both possible solutions [theta1, theta2]
    solns[0][0] = THETA - alpha;
    solns[0][1] = M_PI - beta;
    solns[1][0] = THETA + alpha;
    solns[1][1] = beta - M_PI;

    // Keep angles within [-180, 180]
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            if (solns[i][j] > M_PI)
            {
                solns[i][j] -= 2 * M_PI;
            }
            else if (solns[i][j] < -M_PI)
            {
                solns[i][j] += 2 * M_PI;
            }
        }
    }

#ifdef DEBUG
    PrintAnglesInDegrees(solns[0][0], solns[0][1]);
    PrintAnglesInDegrees(solns[1][0], solns[1][1]);
#endif
}

void PrintState()
{
    printf("Robot State:\n\r");
    printf("Faulted: %s\n\r", state.faulted ? "Yes" : "No");
    printf("Homed: %s\n\r", state.homed ? "Yes" : "No");
    printf("Homing: %s\n\r", state.homing ? "Yes" : "No");
    printf("Manual: %s\n\r", state.manual ? "Yes" : "No");
    printf("Test Running: %s\n\r", state.testRunning ? "Yes" : "No");
    printf("Current Angles in degrees:");
    PrintAnglesInDegrees(state.theta1, state.theta2);
    printf("Current Coords in x-y:");
    PrintCaresianCoords(state.x, state.y);
}

/**
 * @brief Prints two radian angles in degrees to the serial monitor.
 *
 * @param theta1 Radian angle 1.
 * @param theta2 Radian angle 2.
 */
void PrintAnglesInDegrees(double theta1, double theta2)
{
    theta1 = theta1 * (180.0 / M_PI);
    theta2 = theta2 * (180.0 / M_PI);

    int int_part = (int)theta1;
    int decimal_part = abs((int)((theta1 - int_part) * 100)); // 2 decimal places
    int int_part2 = (int)theta2;
    int decimal_part2 = abs((int)((theta2 - int_part2) * 100)); // 2 decimal places

    printf("(%d.%d degrees, %d.%d degrees)\n\r", int_part, decimal_part, int_part2, decimal_part2);
}

/**
 * @brief Prints cartesion coordinates to the serial monitor.
 *
 * @param x coordinate.
 * @param y coordinate.
 */
void PrintCaresianCoords(double x, double y)
{
    int int_part = (int)x;
    int decimal_part = abs((int)((x - int_part) * 100)); // 2 decimal places
    int int_part2 = (int)y;
    int decimal_part2 = abs((int)((y - int_part2) * 100)); // 2 decimal places

    printf("(%d.%d, %d.%d)\n\r", int_part, decimal_part, int_part2, decimal_part2);
}

/**
 * @brief Converts cartesion coordinates to link angles of the robot.
 *
 * @param theta1 angle of link 1.
 * @param theta2 angle of link 2/
 * @param x position
 * @param y position
 */
void CalculateCartesianCoords(double theta1, double theta2, double *x, double *y)
{
    *x = cos(theta1) * LINK_1 + cos(theta1 + theta2) * LINK_2;
    *y = sin(theta1) * LINK_1 + sin(theta1 + theta2) * LINK_2;

#ifdef DEBUG
    PrintCaresianCoords(*x, *y);
#endif
}

/**
 * @brief Caclculates the smallest absolute able between two angles.
 *
 * @param cur_theta current angle.
 * @param targ_theta desired angle.
 * @param *motor motor object
 * @return double - the smallest delta to get from the current to the desired angle.
 */
double CalculateQuickestValidPath(double cur_theta, double targ_theta, Motor *motor)
{

    double delta = targ_theta - cur_theta;
    // Normalize the delta
    if (delta > M_PI)
    {
        delta = delta - 2 * M_PI;
    }
    else if (delta < -M_PI)
    {
        delta = delta + 2 * M_PI;
    }
    // Prevent the robot from moving through the restricted angle
    if ((cur_theta + delta) > (motor->thetaMax - SAFETY_MARGIN))
    {
        return delta - 2 * M_PI;
    }
    else if ((cur_theta + delta) < (motor->thetaMin + SAFETY_MARGIN))
    {
        return delta + 2 * M_PI;
    }
    return delta;
}

bool IsValid(double *soln)
{
    // Check if normalized angles are within the specified range in radians
    if ((soln[0] < (motor1.thetaMin + SAFETY_MARGIN)) || (soln[0] > (motor1.thetaMax - SAFETY_MARGIN)))
    {
        return false;
    }
    else if ((soln[1] < (motor2.thetaMin + SAFETY_MARGIN)) || (soln[1] > (motor2.thetaMax - SAFETY_MARGIN)))
    {
        return false;
    }
    return true;
}

/**
 * @brief Move the robot to the x and y coordinates.
 *
 * @param x coordniate.
 * @param y coordinate.
 */
void MoveTo(double x, double y, double rpm)
{
    // PrintCaresianCoords(x,y);
    double solns[2][2];
    CalculateJointAngle(x, y, solns);

    // Check to see if solutions are valid
    bool soln1_valid = IsValid(solns[0]);
    bool soln2_valid = IsValid(solns[1]);

    double *best;
    // If both solutions are valid, take the quicker one
    if (soln1_valid && soln2_valid)
    {
        double delta_sum_1 = fabs(solns[0][0] - state.theta1) + fabs(solns[0][1] - state.theta2);
        double delta_sum_2 = fabs(solns[1][0] - state.theta1) + fabs(solns[1][1] - state.theta2);

        if (delta_sum_1 < delta_sum_2)
        {
            best = solns[0];
        }
        else
        {
            best = solns[1];
        }
    }
    // If only one solution is valid, take that one
    else if (soln1_valid)
    {
        best = solns[0];
    }
    else if (soln2_valid)
    {
        best = solns[1];
    }
    // If neither solution is valid, error
    else
    {
        // Blink status LED
        HAL_GPIO_WritePin(redLED.port, redLED.pin, 1);
        HAL_Delay(20);
        HAL_GPIO_WritePin(redLED.port, redLED.pin, 0);
        printf("Invalid X-Y Request\n\r");
        return;
    }

    double delta1 = CalculateQuickestValidPath(state.theta1, best[0], &motor1);
    double delta2 = CalculateQuickestValidPath(state.theta2, best[1], &motor2);

    // Path planning
    double rpmDelta, rpm1, rpm2;
    double magDelta1 = fabs(delta1);
    double magDelta2 = fabs(delta2);
    if (delta1 > delta2)
    {
        rpmDelta = rpm * ((magDelta1 - magDelta2) / (magDelta1 + magDelta2));
        rpm1 = rpm + rpmDelta;
        rpm2 = rpm - rpmDelta;
    }
    else
    {
        rpmDelta = rpm * ((magDelta2 - magDelta1) / (magDelta1 + magDelta2));
        rpm1 = rpm - rpmDelta;
        rpm2 = rpm + rpmDelta;
    }

    // Ensure RPMs are reasonable
    if (rpm1 > MAX_RPM)
    {
        rpm1 = MAX_RPM;
    }
    else if (rpm1 < MIN_RPM)
    {
        rpm1 = MIN_RPM;
    }
    if (rpm2 > MAX_RPM)
    {
        rpm2 = MAX_RPM;
    }
    else if (rpm2 < MIN_RPM)
    {
        rpm2 = MIN_RPM;
    }

    // Move the motors
    double mdelta1 = MoveByAngle(&motor1, delta1, rpm1);
    double mdelta2 = MoveByAngle(&motor2, delta2, rpm2);

#ifdef DEBUG
    PrintAnglesInDegrees(mdelta1, mdelta2);
#endif

    // Update the state machine
    state.theta1 += mdelta1;
    state.theta2 += mdelta2;
    CalculateCartesianCoords(state.theta1, state.theta2, &state.x, &state.y);
}

void MoveToZ(double z, double rpm)
{
    // Check if z coord is within limits
    if ((z > motorz.thetaMax - 5.0) || (z < motorz.thetaMin + 5.0)) // Neet to fix this once we mak it taller
    {
        HAL_GPIO_WritePin(redLED.port, redLED.pin, 1);
        HAL_Delay(20);
        HAL_GPIO_WritePin(redLED.port, redLED.pin, 0);
        printf("Invalid Z Request\n\r");
        return;
    }
    else
    {
        double deltaZ = fabs(z - state.currentZ);
        double mdeltaZ = 0;
        if (z >= state.currentZ)
        {
            mdeltaZ = MoveByDist(&motorz, deltaZ, rpm);
        }
        else if (z < state.currentZ)
        {
            deltaZ = deltaZ * -1;
            mdeltaZ = MoveByDist(&motorz, deltaZ, rpm);
        }
        state.currentZ += mdeltaZ;
    }
}

/**
 * @brief Increment the robots current position.
 *
 * @param rel_x x increment.
 * @param rel_y y increment.
 */
void MoveBy(double rel_x, double rel_y, double rpm)
{
    double new_x = state.x + rel_x;
    double new_y = state.y + rel_y;

    MoveTo(new_x, new_y, rpm);
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
        state.manual = 0;
        state.testRunning = 0;
        state.testHasRun = 0;
        changeLEDState(redLED, "Solid");
    }
    else if (strcmp(toState, "Homing") == 0)
    {
        state.faulted = 0;
        state.homed = 0;
        state.homing = 1;
        state.manual = 0;
        state.testRunning = 0;
        changeLEDState(redLED, "Fast");
    }
    else if (strcmp(toState, "Faulted") == 0)
    {
        state.faulted = 1;
        state.homed = 0;
        state.homing = 0;
        state.manual = 0;
        state.testRunning = 0;
        changeLEDState(redLED, "Slow");
    }
    else if (strcmp(toState, "Manual") == 0)
    {
        state.faulted = 0;
        state.homed = 1;
        state.homing = 0;
        state.manual = 1;
        state.testRunning = 0;
        changeLEDState(greenLED, "Slow");
    }
    else if (strcmp(toState, "Auto Wait") == 0)
    {
        state.faulted = 0;
        state.homed = 1;
        state.homing = 0;
        state.manual = 0;
        state.testRunning = 0;
        changeLEDState(greenLED, "Solid");
    }
    else if (strcmp(toState, "Auto Move") == 0)
    {
        state.faulted = 0;
        state.homed = 1;
        state.homing = 0;
        state.manual = 0;
        state.testRunning = 1;
        changeLEDState(greenLED, "Fast");
    }
}