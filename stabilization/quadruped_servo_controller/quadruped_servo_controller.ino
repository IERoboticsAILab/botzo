#include <Servo.h>

// Number of legs and servos per leg
const int NUM_LEGS = 4;
const int SERVOS_PER_LEG = 3;

// Array to store servo objects
Servo servos[NUM_LEGS][SERVOS_PER_LEG];

// Arduino pins for servos (adjust these based on your wiring)
const int servoPins[NUM_LEGS][SERVOS_PER_LEG] = {
  {5, 6, 7},    // Front Left (shoulder, knee, ankle)
  {2, 3, 4},    // Front Right
  {11, 12, 13},   // Back Left
  {8, 9, 10}  // Back Right
};

// Buffer for receiving commands
const int BUFFER_SIZE = 32;
char cmdBuffer[BUFFER_SIZE];
int bufferIndex = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize all servos
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    for (int servo = 0; servo < SERVOS_PER_LEG; servo++) {
      servos[leg][servo].attach(servoPins[leg][servo]);
      // Move to initial position (90 degrees)
      servos[leg][servo].write(90);
    }
  }
}

void processCommand(char* cmd) {
  // Expected format: "S,leg,servo,angle;"
  char* token = strtok(cmd, ",");
  
  // Check if it's a servo command
  if (token[0] != 'S') return;
  
  // Get leg number
  token = strtok(NULL, ",");
  int leg = atoi(token);
  
  // Get servo number
  token = strtok(NULL, ",");
  int servo = atoi(token);
  
  // Get angle
  token = strtok(NULL, ";");
  float angle = atof(token);
  
  // Validate parameters
  if (leg >= 0 && leg < NUM_LEGS && 
      servo >= 0 && servo < SERVOS_PER_LEG && 
      angle >= 0 && angle <= 270) {
    // Write angle to servo
    servos[leg][servo].write(int(angle));
  }
}

void loop() {
  // Read incoming serial data
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Add character to buffer if there's space
    if (bufferIndex < BUFFER_SIZE - 1) {
      cmdBuffer[bufferIndex++] = c;
    }
    
    // Process command when we receive the terminator
    if (c == ';') {
      cmdBuffer[bufferIndex] = '\0';  // Null terminate the string
      processCommand(cmdBuffer);
      bufferIndex = 0;  // Reset buffer
    }
  }
} 