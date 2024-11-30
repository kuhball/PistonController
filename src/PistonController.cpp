// PistonController.cpp
#include "PistonController.h"

// ISR for encoder
void IRAM_ATTR encoderISR(void* arg) {
    PistonController* controller = (PistonController*)arg;
    controller->updateEncoder();
}

PistonController::PistonController(int valvePin1, int valvePin2, int encoderPinA, int encoderPinB)
    : _valvePin1(valvePin1)
    , _valvePin2(valvePin2)
    , _encoderPinA(encoderPinA)
    , _encoderPinB(encoderPinB)
    , _currentPosition(0)
    , _targetPosition(0)
    , _currentState(HOLD)
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
    
    // Attach interrupt for encoder
    attachInterruptArg(digitalPinToInterrupt(_encoderPinA), encoderISR, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(_encoderPinB), encoderISR, this, CHANGE);
    
    _lastTime = millis();
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
    _targetPosition = position;
    _integral = 0; // Reset integral term when target changes
}

long PistonController::getCurrentPosition() const {
    return _currentPosition;
}

void PistonController::update() {
    float error = _targetPosition - _currentPosition;
    float output = calculatePID(error);
    
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
    
    return proportional + _integral + derivative;
}

void PistonController::setValveState(ValveState state) {
    switch (state) {
        case EXTEND:
            digitalWrite(_valvePin1, HIGH);
            digitalWrite(_valvePin2, LOW);
            break;
        case RETRACT:
            digitalWrite(_valvePin1, LOW);
            digitalWrite(_valvePin2, HIGH);
            break;
        case HOLD:
            digitalWrite(_valvePin1, LOW);
            digitalWrite(_valvePin2, LOW);
            break;
    }
    _currentState = state;
}

void PistonController::setPIDParameters(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _integral = 0; // Reset integral term when parameters change
}

void PistonController::emergencyStop() {
    setValveState(HOLD);
    _integral = 0;
}

bool PistonController::isTargetReached() const {
    return abs(_targetPosition - _currentPosition) <= _positionTolerance;
}

void PistonController::calibrate() {
    _currentPosition = 0;
    _targetPosition = 0;
    _integral = 0;
}