// PistonController.h
#ifndef PISTON_CONTROLLER_H
#define PISTON_CONTROLLER_H

#include <Arduino.h>

class PistonController {
public:
    // Constructor
    PistonController(
        int valvePin1,      // Pin for valve direction 1
        int valvePin2,      // Pin for valve direction 2
        int encoderPinA,    // Encoder pin A
        int encoderPinB     // Encoder pin B
    );

    // Initialize the controller
    void begin();
    
    // Set target position in encoder counts
    void setTargetPosition(long position);
    
    // Get current position in encoder counts
    long getCurrentPosition() const;
    
    // Update control loop (call this in main loop)
    void update();
    
    // Set PID control parameters
    void setPIDParameters(float kp, float ki, float kd);
    
    // Emergency stop
    void emergencyStop();
    
    // Check if target position is reached (within tolerance)
    bool isTargetReached() const;
    
    // Calibrate the position (set current position as zero)
    void calibrate();

private:
    // Pins
    const int _valvePin1;
    const int _valvePin2;
    const int _encoderPinA;
    const int _encoderPinB;
    
    // Position variables
    volatile long _currentPosition;
    long _targetPosition;
    const int _positionTolerance = 10; // encoder counts
    
    // Control variables
    float _kp = 1.0;
    float _ki = 0.0;
    float _kd = 0.0;
    float _integral = 0.0;
    float _lastError = 0.0;
    unsigned long _lastTime = 0;
    
    // Valve control states
    enum ValveState {
        EXTEND,
        RETRACT,
        HOLD
    };
    ValveState _currentState;
    
    // Private methods
    void updateEncoder();
    void setValveState(ValveState state);
    float calculatePID(float error);
    
    // ISR friend function for encoder
    friend void IRAM_ATTR encoderISR(void* arg);
};

#endif