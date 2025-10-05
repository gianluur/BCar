#include <SoftwareSerial.h>
#include <ezButton.h>

SoftwareSerial BTSerial(0, 1); // RX, TX

// Define joystick pins
const int VRx = A0; // X-axis
const int VRy = A1; // Y-axis
const int PIN_BUTTON = 2;

void printAxis(int xPos, int yPos) {
  Serial.print("x: ");
  Serial.print(xPos);
  Serial.print(" | y: ");
  Serial.print(yPos);
  Serial.print(" | ");
}

void setup() {
    // Start serial communication
    Serial.begin(9600);  // For debugging with PC
    BTSerial.begin(9600); // For communication with HC-05

    // Initialize joystick pins
    pinMode(VRx, INPUT);
    pinMode(VRy, INPUT);
    pinMode(PIN_BUTTON, INPUT_PULLUP);

    Serial.println("Master is ready...");
}

void loop() {

  int xPos = analogRead(VRx);
  int yPos = analogRead(VRy);

  if (xPos < 250 && (yPos >= 100 && yPos <= 650)) {
    printAxis(xPos, yPos);
    Serial.println('L');
  }
  else if (xPos > 600 && (yPos >= 100 && yPos <= 650)) {
    printAxis(xPos, yPos);
    Serial.println('R');
  }
  else if (yPos < 400) {
    printAxis(xPos, yPos);
    Serial.println('F');
  }
  else if (yPos > 600) {
    printAxis(xPos, yPos);
    Serial.println('B');
  }
  else if (!digitalRead(PIN_BUTTON)){
    printAxis(xPos, yPos);
    Serial.println('S');
  }
}