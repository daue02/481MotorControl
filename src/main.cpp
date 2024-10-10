#include <Arduino.h>

// Define pin connections & motor's steps per revolution
const int stepYPin = 3;
const int dirYPin = 6;
const int stepZPin = 4;
const int dirZPin = 7;

const int microstepY = 16;
const int microstepZ = 16;

const int stepPerRevY = 200 * microstepY;
const int stepPerRevZ = 200 * microstepZ;

void setup()
{
  // Declare pins as Outputs
  pinMode(stepYPin, OUTPUT);
  pinMode(dirYPin, OUTPUT);
  pinMode(stepZPin, OUTPUT);
  pinMode(dirZPin, OUTPUT);
}
void loop()
{
  // Set motor directions
  digitalWrite(dirYPin, HIGH);
  digitalWrite(dirZPin, LOW);

  // Spin motor slowly
  for (int x = 0; x < stepPerRevY; x++)
  {
    digitalWrite(stepYPin, HIGH);
    digitalWrite(stepZPin, HIGH);
    delayMicroseconds(200);
    digitalWrite(stepYPin, LOW);
    digitalWrite(stepZPin, LOW);
    delayMicroseconds(200);
  }
  delay(1000); // Wait a second

  // Set motor direction counterclockwise
  digitalWrite(dirYPin, LOW);
  digitalWrite(dirZPin, HIGH);

  // Spin motor quickly
  for (int x = 0; x < stepPerRevY; x++)
  {
    digitalWrite(stepYPin, HIGH);
    digitalWrite(stepZPin, HIGH);
    delayMicroseconds(200);
    digitalWrite(stepYPin, LOW);
    digitalWrite(stepZPin, LOW);
    delayMicroseconds(200);
  }
  delay(1000); // Wait a second
}