#include <Wire.h> // Include necessary libraries
#include <NewPing.h>

#define MAX_SPEED 60 // Maximum motor speed
#define Kp 5.0        // Proportional gain
#define Ki 0.0        // Integral gain
#define Kd 4.0        // Derivative gain

// Define pins for motors
#define ENA 10
#define IN1 8
#define IN2 9
#define ENB 11
#define IN3 12
#define IN4 13

// Define pins for ultrasonic sensor
#define trigPin 2
#define echoPin 3
#define trigPin1 6
#define echoPin1 7

NewPing sonar(trigPin, echoPin, 200); // Define ultrasonic sensor
NewPing sonar1(trigPin1, echoPin1, 200); // Define ultrasonic sensor

double targetValue = 0; // Target value for maintaining distance from left wall
double previousError = 0; // Variable to store previous error for derivative term
double integral = 0;
double derivative = 0;
void setup() {
  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

delay(2000);
}

void loop() {
  // Read distance from ultrasonic sensor
  double distance = sonar1.ping_cm() - sonar.ping_cm();

  // Calculate error
  double error = targetValue - distance;

  // Calculate PID terms
  integral = integral + error;
  derivative = Kd * (error - previousError);

  // Calculate motor speeds
  int leftSpeed = MAX_SPEED + Kp * error + Ki * integral + Kd * derivative;
  int rightSpeed = MAX_SPEED - Kp * error - Ki * integral - Kd * derivative;

  // Constrain motor speeds to be within valid range
  leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

  // Apply motor speeds
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);


  // Update previous error
  previousError = error;

  // Add a small delay to avoid flooding the serial monitor
  delay(50);
}
