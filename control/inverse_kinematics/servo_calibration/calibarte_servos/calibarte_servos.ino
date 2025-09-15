#include <Servo.h>

Servo servo;
int servo_pin = 9;

void setup() {
  // Start serial communication at 9600 baud rate
  Serial.begin(9600);

  // Attach the servo to the pin
  servo.attach(servo_pin);

  int pwmValue = 500;
  // Move the servo to the initial PWM value
  servo.writeMicroseconds(pwmValue);

  // Print initial PWM value
  Serial.println("Starting servo calibration...");
  Serial.print("Current PWM: ");
  Serial.println(pwmValue);
  Serial.println("Enter next PWM value (0 to 2000):");
}

void loop() {
  // Wait for new input in the serial monitor
  if (Serial.available() > 0) {
    // Read input as a string and trim any unnecessary whitespace
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Convert the input string to an integer
    int newPWM = input.toInt();

    // Check if the input is valid (non-zero)
    if (newPWM >= 500 && newPWM <= 2000) {
      // Move the servo to the new PWM value
      servo.writeMicroseconds(newPWM);

      // Print the updated PWM value
      Serial.print("Current PWM: ");
      Serial.println(newPWM);
    } else {
      // If input is out of range, print an error
      Serial.println("Invalid PWM value. Enter a value between 500 and 2000.");
    }

    // Prompt for the next PWM value
    Serial.println("Enter next PWM value (500 to 2000):");
  }
}
