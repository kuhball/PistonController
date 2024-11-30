// BasicControl.ino
#include <PistonController.h>

// Pin definitions
const int VALVE_PIN_1 = 25;  // ESP32 GPIO pin for valve direction 1
const int VALVE_PIN_2 = 26;  // ESP32 GPIO pin for valve direction 2
const int ENCODER_PIN_A = 32; // ESP32 GPIO pin for encoder A
const int ENCODER_PIN_B = 33; // ESP32 GPIO pin for encoder B

// Create controller instance
PistonController piston(VALVE_PIN_1, VALVE_PIN_2, ENCODER_PIN_A, ENCODER_PIN_B);

void setup() {
    Serial.begin(115200);
    
    // Initialize the controller
    piston.begin();
    
    // Set PID parameters (tune these values for your system)
    piston.setPIDParameters(1.0, 0.1, 0.05);
    
    // Calibrate the position
    piston.calibrate();
}

void loop() {
    // Example: Move to position 1000, wait, then move to position 0
    static bool movingForward = true;
    static unsigned long lastChange = 0;
    
    // Update the control loop
    piston.update();
    
    // Print current position
    Serial.print("Position: ");
    Serial.println(piston.getCurrentPosition());
    
    // Change direction every 5 seconds
    if (millis() - lastChange > 5000) {
        if (movingForward) {
            piston.setTargetPosition(1000);
        } else {
            piston.setTargetPosition(0);
        }
        movingForward = !movingForward;
        lastChange = millis();
    }
    
    delay(10); // Small delay to prevent overwhelming the serial output
}