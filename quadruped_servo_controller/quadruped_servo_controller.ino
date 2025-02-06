#include <Servo.h>

// Number of legs (each with one servo)
const int NUM_LEGS = 4;

// Array to store servo objects
Servo servos[NUM_LEGS];

// Arduino pins for servos (one per leg)
const int servoPins[NUM_LEGS] = {
  5,  // Front Left
  2,  // Front Right
  11, // Back Left
  8   // Back Right
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
    servos[leg].attach(servoPins[leg]);
    // Move to initial position (90 degrees)
    servos[leg].write(90);
  }
}

void processCommand(char* cmd) {
  // Expected format: "S,leg,angle;"
  char* token = strtok(cmd, ",");
  
  // Check if it's a servo command
  if (token[0] != 'S') return;
  
  // Get leg number
  token = strtok(NULL, ",");
  int leg = atoi(token);
  
  // Get angle
  token = strtok(NULL, ";");
  float angle = atof(token);
  
  // Validate parameters
  if (leg >= 0 && leg < NUM_LEGS && angle >= 0 && angle <= 270) {
    // Write angle to servo
    servos[leg].write(int(angle));
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