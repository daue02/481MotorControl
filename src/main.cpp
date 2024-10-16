#include <Arduino.h>
#include <accelstepper.h>
#include "motor_hal.h"
#include "controls.h"

double receiveCoordinate(char axis)
{
  String input = ""; // Buffer to hold the serial input
  double coordinate = 0.0;

  // Prompt the user to enter the coordinate for the specified axis (Y or Z)
  Serial.print("Enter ");
  Serial.print(axis);
  Serial.print(" coordinate: ");

  // Wait for input from the user
  while (Serial.available() == 0)
  {
    // Wait until data is available in the serial buffer
  }

  // Read the input from the serial buffer
  input = Serial.readStringUntil('\n');
  input.trim(); // Remove any leading or trailing whitespace

  // Convert the input to a double
  coordinate = input.toFloat();

  return coordinate; // Return the coordinate value
}

// Main function to receive Y and Z coordinates
void receiveCoordinates(double &yCoord, double &zCoord)
{
  // Receive Y coordinate
  yCoord = receiveCoordinate('Y');

  // Receive Z coordinate
  zCoord = receiveCoordinate('Z');
}

void setup()
{
  delay(1000);
  Serial.begin(9600);
  Motors_Init();

  double y = 0, z = 0;

  while (1)
  {
    receiveCoordinates(y, z);
    moveTo(y, z);
    printState();
  }
}

void loop()
{
}