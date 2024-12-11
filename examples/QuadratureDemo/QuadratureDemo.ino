// JoggingExample.ino
#include <PistonController.h>

// Pin definitions
const int VALVE_PIN_1 = 25;    // ESP32 GPIO pin for valve direction 1
const int VALVE_PIN_2 = 26;    // ESP32 GPIO pin for valve direction 2
const int ENCODER_PIN_A = 32;  // ESP32 GPIO pin for encoder A
const int ENCODER_PIN_B = 33;  // ESP32 GPIO pin for encoder B

// Button pins for manual control
const int BUTTON_JOG_EXTEND = 12;   // Jog extend button
const int BUTTON_JOG_RETRACT = 13;  // Jog retract button
const int BUTTON_STOP = 27;         // Emergency stop button
const int BUTTON_MODE = 14;         // Mode switch button

// DMX pins
const int DMX_RX = 16;
const int DMX_TX = 17;
const int DMX_EN = 21;

// Create controller instance
PistonController piston(VALVE_PIN_1, VALVE_PIN_2, ENCODER_PIN_A, ENCODER_PIN_B, DMX_RX, DMX_TX, DMX_EN);

// Operating modes
enum OperatingMode {
    MODE_MANUAL,
    MODE_AUTO,
    MODE_DMX_SWITCH,
    MODE_DMX_LINEAR,
    MODE_HOMING
} currentMode = MODE_MANUAL;

// Movement parameters
const long POSITION_1 = 1000;   // Extended position
const long POSITION_2 = 0;     // Retracted position
const unsigned long AUTO_MOVE_INTERVAL = 5000; // Time between auto movements (ms)
const unsigned long DEBOUNCE_TIME = 50;       // Button debounce time (ms)

// System variables
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_INTERVAL = 500;     // Status update interval (ms)
unsigned long lastAutoMove = 0;
bool movingToPosition1 = true;
bool buttonModeLastState = HIGH;
unsigned long lastModeButtonPress = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 5000);
    
    // Initialize buttons
    pinMode(BUTTON_JOG_EXTEND, INPUT_PULLUP);
    pinMode(BUTTON_JOG_RETRACT, INPUT_PULLUP);
    pinMode(BUTTON_STOP, INPUT_PULLUP);
    pinMode(BUTTON_MODE, INPUT_PULLUP);
    
    // Initialize piston controller
    piston.begin();
    piston.setPIDParameters(1.2, 0.1, 0.05);
    
    Serial.println(F("\nPneumatic Piston Control System"));
    Serial.println(F("--------------------------------"));
    printHelp();
}

void loop() {
    // Check emergency stop
    if (digitalRead(BUTTON_STOP) == LOW) {
        handleEmergencyStop();
        return;
    }
    
    // Process serial commands
    if (Serial.available()) {
        handleSerialCommand();
    }
    
    // Check mode button with debounce
    handleModeButton();

    switch(currentMode) {
        case MODE_MANUAL:
            handleJoggingButtons();
            break;
        case MODE_AUTO:
            if (!piston.isJogging()) {
                handleAutoMode();
            }
            break;
        case MODE_DMX_SWITCH:
            piston.checkDmxSignal();
            piston.dmxSwitch();
            break;
        case MODE_DMX_LINEAR:
            piston.dmxLinear();
            break;
        case MODE_HOMING:
            piston.findHome();
            currentMode = MODE_AUTO;
            break;
    }
    
    // Update position control
    piston.update();
    
    // Regular status updates
    if (millis() - lastStatusUpdate >= STATUS_INTERVAL) {
        printStatus();
        lastStatusUpdate = millis();
    }
}

void handleModeButton() {
    bool currentButtonState = digitalRead(BUTTON_MODE);
    
    if (currentButtonState != buttonModeLastState) {
        if (millis() - lastModeButtonPress > DEBOUNCE_TIME) {
            if (currentButtonState == LOW) {  // Button pressed
                toggleMode();
                lastModeButtonPress = millis();
            }
        }
    }
    buttonModeLastState = currentButtonState;
}

void toggleMode() {
    // Only allow mode change if not jogging
    if (!piston.isJogging()) {
        currentMode = (currentMode == MODE_MANUAL) ? MODE_AUTO : MODE_MANUAL;
        Serial.printf("Mode changed to: %s\n", (currentMode == MODE_AUTO) ? "AUTO" : "MANUAL");
        
        // Reset auto movement variables when entering auto mode
        if (currentMode == MODE_AUTO) {
            lastAutoMove = millis();
            movingToPosition1 = true;
        }
    } else {
        Serial.println(F("Cannot change mode while jogging"));
    }
}

void handleJoggingButtons() {
    static bool wasJogging = false;
    
    if (digitalRead(BUTTON_JOG_EXTEND) == LOW) {
        piston.jogExtend();
        wasJogging = true;
    }
    else if (digitalRead(BUTTON_JOG_RETRACT) == LOW) {
        piston.jogRetract();
        wasJogging = true;
    }
    else if (wasJogging) {
        piston.stopJog();
        wasJogging = false;
    }
}

void handleAutoMode() {
    if (millis() - lastAutoMove >= AUTO_MOVE_INTERVAL) {
        if (piston.isTargetReached()) {
            if (movingToPosition1) {
                piston.setTargetPosition(POSITION_1);
            } else {
                piston.setTargetPosition(POSITION_2);
            }
            movingToPosition1 = !movingToPosition1;
            lastAutoMove = millis();
        }
    }
}

void handleEmergencyStop() {
    piston.emergencyStop();
    currentMode = MODE_MANUAL;  // Default to manual mode after emergency stop
    Serial.println(F("\n*** EMERGENCY STOP ACTIVATED ***"));
}

void handleSerialCommand() {
    char cmd = Serial.read();
    switch (cmd) {
        case 'm': // Toggle mode
            toggleMode();
            break;
            
        case 'E': // Continuous extend
            if (currentMode == MODE_MANUAL) {
                piston.jogExtend();
            }
            break;
            
        case 'R': // Continuous retract
            if (currentMode == MODE_MANUAL) {
                piston.jogRetract();
            }
            break;
            
        case 's': // Stop
            piston.stopJog();
            break;
            
        case 'z': // Zero position
            piston.calibrate();
            break;
            
        case '?': // Help
            printHelp();
            break;
    }
}

void printStatus() {
    Serial.printf("Position: %ld, Mode: %s, %s\n",
                 piston.getCurrentPosition(),
                 (currentMode == MODE_AUTO) ? "AUTO" : "MANUAL",
                 piston.isJogging() ? "JOGGING" : "STOPPED");
}

void printHelp() {
    Serial.println(F("\nAvailable Commands:"));
    Serial.println(F("m - Toggle manual/auto mode"));
    Serial.println(F("E - Continuous extend"));
    Serial.println(F("R - Continuous retract"));
    Serial.println(F("s - Stop movement"));
    Serial.println(F("z - Zero position"));
    Serial.println(F("? - Show this help"));
    Serial.println(F("\nButtons:"));
    Serial.println(F("MODE        - Toggle auto/manual mode"));
    Serial.println(F("JOG_EXTEND  - Hold to extend (manual mode)"));
    Serial.println(F("JOG_RETRACT - Hold to retract (manual mode)"));
    Serial.println(F("STOP        - Emergency stop"));
}
