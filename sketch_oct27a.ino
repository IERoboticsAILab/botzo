void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 bps
  Serial.println("Arduino ready to receive data...");
}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');  // Read until newline character
    Serial.print("Received: ");
    Serial.println(data);  // Display the received data
  }
}
