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
    int encoderPinB,
    int dmxPinRx,
    int dmxPinTx,
    int dmxPinEn
)
    : _valvePin1(valvePin1)
    , _valvePin2(valvePin2)
    , _encoderPinA(encoderPinA)
    , _encoderPinB(encoderPinB)
    , _dmxPinRx(dmxPinRx)
    , _dmxPinTx(dmxPinTx)
    , _dmxPinEn(dmxPinEn)

    , _dmxPort(DMX_NUM_1)
    , _currentPosition(0)
    , _targetPosition(0)
    , _currentState(HOLD)
    , _isJogging(false)
    , _homeExtend(0)
    , _homeRetract(0)
{
}

void PistonController::setup() {
    // Setup valve control pins
    pinMode(_valvePin1, OUTPUT);
    pinMode(_valvePin2, OUTPUT);
    digitalWrite(_valvePin1, LOW);
    digitalWrite(_valvePin2, LOW);

    // Setup encoder pins
    pinMode(_encoderPinA, INPUT_PULLUP);
    pinMode(_encoderPinB, INPUT_PULLUP);

    // Setup Dmx
    dmx_config_t config = DMX_CONFIG_DEFAULT; // TODO: Include build version here for RDM
    dmx_personality_t personalities[] = {
        {5, "Piston Controller"},
    };
    int personality_count = 1;
    dmx_driver_install(_dmxPort, &config, personalities, personality_count);
    dmx_set_pin(_dmxPort, _dmxPinTx, _dmxPinRx, _dmxPinEn);

    // Attach interrupts for encoder
    attachInterruptArg(digitalPinToInterrupt(_encoderPinA), encoderISR, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(_encoderPinB), encoderISR, this, CHANGE);

    _lastTime = millis();

    debugPrintf("PistonController initialized. Encoder pins: %d, %d\n",
                _encoderPinA, _encoderPinB);
}

void PistonController::loop() {
    checkDmxSignal();
    dmxSwitch();
}

void PistonController::checkDmxSignal() {
    dmx_packet_t packet;

    if (dmx_receive(_dmxPort, &packet, DMX_TIMEOUT_TICK)) {
        if (!packet.err) {
            dmx_read(1, _data, packet.size);
        }
    }
}

void PistonController::dmxSwitch(){
        if (_data[1] > 127) {
            setValveState(EXTEND);
        } else if (_data[2] > 127) {
            setValveState(RETRACT);
        } else {
            setValveState(HOLD);
        }
}

void PistonController::dmxLinear(){
        setTargetPosition((_travelLength / 255) * _data[1]);
    }

bool PistonController::findHome(unsigned long timeout) {
    debugPrintf("Starting home search\n");


    // search for home position in retract
    unsigned long startTime = millis();
    setValveState(RETRACT);

    while (millis() - startTime < timeout) {
        // Allow other tasks to run
        delay(1);
    }
    setValveState(HOLD);

    debugPrintf("Home retract found successfully at: %ld\n", _currentPosition);
    _homeRetract = _currentPosition;

    // search for home position in extend
    startTime = millis();
    setValveState(EXTEND);

    while (millis() - startTime < timeout) {
        // Allow other tasks to run
        delay(1);
    }
    setValveState(HOLD);

    debugPrintf("Home extend found successfully at: %ld\n", _currentPosition);
    _homeExtend = _currentPosition;

    _travelLength = _homeExtend - _homeRetract;
    debugPrintf("Travel length is %ld", _travelLength);
    return true;
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
        debugPrintf("New target position set: %ld\n", position);
    }
}

long PistonController::getCurrentPosition() const {
    return _currentPosition;
}

void PistonController::update() {
    // if (_isJogging) {
    //     return; // Skip PID control during jogging
    // }

    float error = _targetPosition - _currentPosition;
    float output = calculatePID(error);

    static unsigned long lastDebugPrint = 0;
    if (millis() - lastDebugPrint > 500) {
        debugPrintf("Position: %ld, Target: %ld, Error: %.2f, Output: %.2f\n",
                    _currentPosition, _targetPosition, error, output);
        lastDebugPrint = millis();
    }

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

    static unsigned long lastPIDPrint = 0;
    if (millis() - lastPIDPrint > 1000) {
        debugPrintf("PID details - P: %.2f, I: %.2f, D: %.2f\n",
                    proportional, _integral, derivative);
        lastPIDPrint = millis();
    }

    return proportional + _integral + derivative;
}

void PistonController::setValveState(ValveState state) {
    if (state != _currentState) {
        switch (state) {
            case EXTEND:
                digitalWrite(_valvePin1, HIGH);
                digitalWrite(_valvePin2, LOW);
                debugPrintf("Valve state changed to: EXTEND\n");
                break;
            case RETRACT:
                digitalWrite(_valvePin1, LOW);
                digitalWrite(_valvePin2, HIGH);
                debugPrintf("Valve state changed to: RETRACT\n");
                break;
            case HOLD:
                digitalWrite(_valvePin1, LOW);
                digitalWrite(_valvePin2, LOW);
                debugPrintf("Valve state changed to: HOLD\n");
                break;
        }
        _currentState = state;
    }
}

void PistonController::jogExtend() {
    _isJogging = true;
    setValveState(EXTEND);
    debugPrintf("Jogging extend.\n");
}

void PistonController::jogRetract() {
    _isJogging = true;
    setValveState(RETRACT);
    debugPrintf("Jogging retract.\n");
}

void PistonController::stopJog() {
    if (_isJogging) {
        _isJogging = false;
        setValveState(HOLD);
        debugPrintf("Jog stopped at position: %ld\n", _currentPosition);
    }
}

void PistonController::setPIDParameters(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _integral = 0; // Reset integral term when parameters change

    debugPrintf("PID parameters updated - Kp: %.2f, Ki: %.2f, Kd: %.2f\n", kp, ki, kd);
}

bool PistonController::isTargetReached() const {
    if (_isJogging) {
        return false;
    }
    bool reached = abs(_targetPosition - _currentPosition) <= _positionTolerance;
    if (reached) {
        debugPrintf("Target position reached: %ld\n");
    }
    return reached;
}

void PistonController::calibrate() {
    _currentPosition = 0;
    _targetPosition = 0;
    _integral = 0;
    debugPrintf("Position calibrated to zero at time: ", millis());
}
