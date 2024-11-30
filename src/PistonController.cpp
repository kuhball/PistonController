#include "PistonController.h"

// ISR for encoder
void IRAM_ATTR encoderISR(void* arg) {
    PistonController* controller = (PistonController*)arg;
    controller->updateEncoder();
}

PistonController::PistonController(
    int valvePin1, 
    int valvePin2, 
    int encoderPinA, 
    int encoderPinB
)
    : _valvePin1(valvePin1)
    , _valvePin2(valvePin2)
    , _encoderPinA(encoderPinA)
    , _encoderPinB(encoderPinB)
    , _currentPosition(0)
    , _targetPosition(0)
    , _currentState(HOLD)
    , _debugEnabled(true)
    , _isJogging(false)
{
}

void PistonController::begin() {
    // Setup valve control pins
    pinMode(_valvePin1, OUTPUT);
    pinMode(_valvePin2, OUTPUT);
    digitalWrite(_valvePin1, LOW);
    digitalWrite(_valvePin2, LOW);
    
    // Setup encoder pins
    pinMode(_encoderPinA, INPUT_PULLUP);
    pinMode(_encoderPinB, INPUT_PULLUP);
    
    // Attach interrupts for encoder
    attachInterruptArg(digitalPinToInterrupt(_encoderPinA), encoderISR, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(_encoderPinB), encoderISR, this, CHANGE);
    
    _lastTime = millis();
    
    debugPrint("PistonController initialized. Encoder pins: ", 
               String(_encoderPinA) + "," + String(_encoderPinB));
}

void PistonController::updateEncoder() {
    static uint8_t oldAB = 0;
    uint8_t newAB = (digitalRead(_encoderPinA) << 1) | digitalRead(_encoderPinB);
    
    // Lookup table for encoder state changes
    static const int8_t lookup[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
    
    oldAB = ((oldAB << 2) | newAB) & 0x0F;
    _currentPosition += lookup[oldAB];
}

void PistonController::setTargetPosition(long position) {
    if (!_isJogging) {
        _targetPosition = position;
        _integral = 0; // Reset integral term when target changes
        debugPrint("New target position set: ", position);
    }
}

long PistonController::getCurrentPosition() const {
    return _currentPosition;
}

void PistonController::update() {
    if (_isJogging) {
        return; // Skip PID control during jogging
    }

    float error = _targetPosition - _currentPosition;
    float output = calculatePID(error);
    
    #if PISTON_DEBUG
    if (_debugEnabled) {
        static unsigned long lastDebugPrint = 0;
        if (millis() - lastDebugPrint > 500) {
            DEBUG_PRINTF("Position: %ld, Target: %ld, Error: %.2f, Output: %.2f\n", 
                        _currentPosition, _targetPosition, error, output);
            lastDebugPrint = millis();
        }
    }
    #endif
    
    // Determine valve state based on PID output
    if (abs(error) <= _positionTolerance) {
        setValveState(HOLD);
    } else if (output > 0) {
        setValveState(EXTEND);
    } else {
        setValveState(RETRACT);
    }
}

float PistonController::calculatePID(float error) {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - _lastTime) / 1000.0;
    
    // Calculate PID terms
    float proportional = _kp * error;
    _integral += _ki * error * deltaTime;
    float derivative = _kd * (error - _lastError) / deltaTime;
    
    // Update state variables
    _lastError = error;
    _lastTime = currentTime;
    
    // Anti-windup for integral term
    if (_integral > 100) _integral = 100;
    if (_integral < -100) _integral = -100;
    
    #if PISTON_DEBUG
    if (_debugEnabled) {
        static unsigned long lastPIDPrint = 0;
        if (millis() - lastPIDPrint > 1000) {
            DEBUG_PRINTF("PID details - P: %.2f, I: %.2f, D: %.2f\n", 
                        proportional, _integral, derivative);
            lastPIDPrint = millis();
        }
    }
    #endif
    
    return proportional + _integral + derivative;
}

void PistonController::setValveState(ValveState state) {
    if (state != _currentState) {
        switch (state) {
            case EXTEND:
                digitalWrite(_valvePin1, HIGH);
                digitalWrite(_valvePin2, LOW);
                debugPrint("Valve state changed to: ", "EXTEND");
                break;
            case RETRACT:
                digitalWrite(_valvePin1, LOW);
                digitalWrite(_valvePin2, HIGH);
                debugPrint("Valve state changed to: ", "RETRACT");
                break;
            case HOLD:
                digitalWrite(_valvePin1, LOW);
                digitalWrite(_valvePin2, LOW);
                debugPrint("Valve state changed to: ", "HOLD");
                break;
        }
        _currentState = state;
    }
}

void PistonController::jogExtend(bool continuous) {
    _isJogging = true;
    if (!continuous) {
        _targetPosition = _currentPosition + JOG_INCREMENT;
    }
    setValveState(EXTEND);
    debugPrint("Jogging extend. Continuous: ", continuous);
}

void PistonController::jogRetract(bool continuous) {
    _isJogging = true;
    if (!continuous) {
        _targetPosition = _currentPosition - JOG_INCREMENT;
    }
    setValveState(RETRACT);
    debugPrint("Jogging retract. Continuous: ", continuous);
}

void PistonController::stopJog() {
    if (_isJogging) {
        _isJogging = false;
        setValveState(HOLD);
        _targetPosition = _currentPosition; // Update target to current position
        _integral = 0; // Reset integral term
        debugPrint("Jog stopped at position: ", _currentPosition);
    }
}

void PistonController::setPIDParameters(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _integral = 0; // Reset integral term when parameters change
    
    #if PISTON_DEBUG
    if (_debugEnabled) {
        DEBUG_PRINTF("PID parameters updated - Kp: %.2f, Ki: %.2f, Kd: %.2f\n", kp, ki, kd);
    }
    #endif
}

void PistonController::emergencyStop() {
    _isJogging = false;
    setValveState(HOLD);
    _integral = 0;
    debugPrint("Emergency stop triggered at position: ", _currentPosition);
}

bool PistonController::isTargetReached() const {
    if (_isJogging) {
        return false;
    }
    bool reached = abs(_targetPosition - _currentPosition) <= _positionTolerance;
    if (reached) {
        debugPrint("Target position reached: ", _currentPosition);  // Now works with const
    }
    return reached;
}

void PistonController::calibrate() {
    _currentPosition = 0;
    _targetPosition = 0;
    _integral = 0;
    debugPrint("Position calibrated to zero at time: ", millis());
}