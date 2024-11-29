#include <Stepper.h>

// Steps per revolution for the stepper motors
const int stepsPerRevolution = 200;

// Initialize stepper motors on their respective pins
Stepper motor1(stepsPerRevolution, 13, 12, 11, 10);
Stepper motor2(stepsPerRevolution, 9, 8, 7, 6);
Stepper motor3(stepsPerRevolution, 5, 4, 3, 2);

// DC motor pins
const int DC_MOTF = 14; // Forward pin
const int DC_MOTR = 15; // Reverse pin

// Laser pin
const int LASER_PIN = 16;

// Variables to hold target positions for each motor
int motor1Target = 0;
int motor2Target = 0;
int motor3Target = 0;

// Variables to hold current positions for each motor
int motor1Current = 0;
int motor2Current = 0;
int motor3Current = 0;

// Flags
bool isDCMotorRunning = true; // State of the DC motor
bool laserActivated = false; // State of the laser

void setup() {
  // Initialize the serial port for debugging and commands
  Serial.begin(9600);

  // Configure DC motor and laser pins
  pinMode(DC_MOTF, OUTPUT);
  pinMode(DC_MOTR, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);

  // Start the DC motor in the forward direction
  digitalWrite(DC_MOTF, HIGH);
  digitalWrite(DC_MOTR, LOW);

  // Initialize laser state
  digitalWrite(LASER_PIN, LOW);

  // Print a ready message
  Serial.println("Delta Robot Control Ready");

  // Run test cases
  runTestCases();
}

void loop() {
  // Check for serial input
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  // If DC motor stops, process delta robot operations
  if (!isDCMotorRunning) {
    updateMotorPosition(motor1, motor1Current, motor1Target);
    updateMotorPosition(motor2, motor2Current, motor2Target);
    updateMotorPosition(motor3, motor3Current, motor3Target);

    // Check if all motors have reached their target
    if (motorsAtTarget()) {
      activateLaser(); // Activate the laser
    }
  }

  delay(50); // Delay for smooth operation
}

// Function to process commands from Raspberry Pi
void processCommand(String command) {
  if (command.startsWith("COORDS")) {
    // Extract coordinates from the command
    int m1_pos = extractValue(command, "M1:");
    int m2_pos = extractValue(command, "M2:");
    int m3_pos = extractValue(command, "M3:");

    // Update target positions for each motor
    if (m1_pos != -1) motor1Target = m1_pos;
    if (m2_pos != -1) motor2Target = m2_pos;
    if (m3_pos != -1) motor3Target = m3_pos;

    // Stop the DC motor
    stopDCMotor();
  }
}

// Function to stop the DC motor
void stopDCMotor() {
  digitalWrite(DC_MOTF, LOW);
  digitalWrite(DC_MOTR, LOW);
  isDCMotorRunning = false;
  Serial.println("DC Motor Stopped");
}

// Function to activate the laser
void activateLaser() {
  if (!laserActivated) {
    digitalWrite(LASER_PIN, HIGH);
    laserActivated = true;
    Serial.println("Laser Activated");
  }
}

// Function to update motor positions
void updateMotorPosition(Stepper& motor, int& current, int target) {
  if (current < target) {
    motor.step(1); // Move forward
    current++;
  } else if (current > target) {
    motor.step(-1); // Move backward
    current--;
  }
}

// Function to check if all motors have reached their target
bool motorsAtTarget() {
  return (motor1Current == motor1Target && 
          motor2Current == motor2Target && 
          motor3Current == motor3Target);
}

// Helper function to extract values from a command string
int extractValue(String command, String prefix) {
  int prefixIndex = command.indexOf(prefix);
  if (prefixIndex != -1) {
    int startIndex = prefixIndex + prefix.length();
    int endIndex = command.indexOf(" ", startIndex);
    if (endIndex == -1) endIndex = command.length();
    return command.substring(startIndex, endIndex).toInt();
  }
  return -1; // Return -1 if the prefix is not found
}

// Function to run test cases
void runTestCases() {
  Serial.println("Running Test Cases...");

  // Test Case 1: Stop DC Motor and move to first coordinate
  Serial.println("Test Case 1: Moving to (M1:100, M2:200, M3:300)");
  processCommand("COORDS M1:100 M2:200 M3:300");

  // Simulate some delay to observe actions
  delay(2000);

  // Test Case 2: Move to another coordinate
  Serial.println("Test Case 2: Moving to (M1:150, M2:250, M3:350)");
  processCommand("COORDS M1:150 M2:250 M3:350");

  delay(2000);

  // Test Case 3: Reverse direction to a smaller coordinate
  Serial.println("Test Case 3: Moving to (M1:-50, M2:0, M3:50)");
  processCommand("COORDS M1:-50 M2:0 M3:50");

  delay(2000);

  // Final State
  Serial.println("Test Cases Completed");
}
