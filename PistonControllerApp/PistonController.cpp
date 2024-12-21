#include <Arduino.h>
#include <esp_dmx.h>

#include "PistonController.h"

// ISR for encoder
void IRAM_ATTR encoderISR(void* arg) {
    PistonController* controller = (PistonController*)arg;
    controller->updateEncoder();
}

PistonController *controller = NULL;

// ISR for timer
void IRAM_ATTR timerISR(void)
{
    controller->tick();
}

void PistonController::tick()
{
    static uint8_t pwm_counter = 0;

    pwm_counter++;
    if (pwm_counter > _dutyCycle1) {
        digitalWrite(_valvePin1, LOW);
    }
    if (pwm_counter < _dutyCycle1) {
        digitalWrite(_valvePin1, HIGH);
    }

    if (pwm_counter > _dutyCycle2) {
        digitalWrite(_valvePin2, LOW);
    }
    if (pwm_counter < _dutyCycle2) {
        digitalWrite(_valvePin2, HIGH);
    }

    if (pwm_counter == 0xff) {
        _runPID = true;
    }
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

    , _state(PISTON_CONTROLLER_INVALID)
    , _dmxPort(DMX_NUM_1)
    , _currentPosition(0)
    , _targetPosition(0)
    , _currentValveState(INVALID)
    , _homeExtended(0)
    , _homeRetracted(0)
{
}

void PistonController::setup() {
    // Setup valve control pins
    pinMode(_valvePin1, OUTPUT);
    pinMode(_valvePin2, OUTPUT);
    setValveState(HOLD);

    // Setup encoder pins
    pinMode(_encoderPinA, INPUT_PULLUP);
    pinMode(_encoderPinB, INPUT_PULLUP);

    // Attach interrupts for encoder
    attachInterruptArg(digitalPinToInterrupt(_encoderPinA), encoderISR, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(_encoderPinB), encoderISR, this, CHANGE);

    // Setup Dmx
    dmx_config_t config = DMX_CONFIG_DEFAULT; // TODO: Include build version here for RDM
    dmx_personality_t personalities[] = {
        {5, "Piston Controller"},
    };
    int personality_count = 1;
    dmx_driver_install(_dmxPort, &config, personalities, personality_count);
    dmx_set_pin(_dmxPort, _dmxPinTx, _dmxPinRx, _dmxPinEn);

    enterState(PISTON_CONTROLLER_INIT);
    debugPrintf("PistonController initialized. Encoder pins: %d, %d\n",
                _encoderPinA, _encoderPinB);

    // Setup Timer
    controller = this;
    _timer = timerBegin(0, 80, true);
    timerAttachInterrupt(_timer, timerISR, true);
    timerAlarmWrite(_timer, 392, true);
    timerAlarmEnable(_timer);

    // Simulate homed state for rapid development
    enterState(PISTON_CONTROLLER_HOMED);
    _travelLength = 41277;
    _currentPosition = 41277;
}

static const char *PistonControllerStateStr[] = {
    "INIT",
    "HOME_START",
    "HOME_EXTEND",
    "HOME_RETRACT",
    "HOMED",
    "REHOME_START",
    "INVALID"
};

void PistonController::enterState(enum PistonControllerState state)
{
    if (_state == state) {
        debugPrintf("Warning: Attempted to re-enter state %s\n",
                    PistonControllerStateStr[state]);
        return;
    }

    debugPrintf("State change %s -> %s\n", PistonControllerStateStr[_state],
                PistonControllerStateStr[state]);

    _state = state;
    _lastStateChange = millis();
}

void PistonController::loop() {
    static unsigned long lastLoopPrint = 0;
    if (millis() - lastLoopPrint > 1000) {
        debugPrintf("State = %s\, currentPosition = %ld\n",
                    PistonControllerStateStr[_state],
                    _currentPosition);
        lastLoopPrint = millis();
    }

    readDmx();

    if (_dmxData[DMX_CHAN_EXTEND] > 127) {
        _isJogging = true;
        setValveState(EXTEND);
        return;
    }

    if (_dmxData[DMX_CHAN_RETRACT] > 127) {
        _isJogging = true;
        setValveState(RETRACT);
        return;
    }

    switch (_state) {
    case PISTON_CONTROLLER_INIT:
        setValveState(HOLD);
        if (_dmxData[DMX_CHAN_CTRL] == 127) {
            enterState(PISTON_CONTROLLER_HOME_START);
        }
        break;
    case PISTON_CONTROLLER_HOME_START:
        setValveState(HOLD);
        if (_dmxData[DMX_CHAN_CTRL] != 127) {
            enterState(PISTON_CONTROLLER_INIT);
        }
        if (millis() - _lastStateChange > TIMEOUT_HOME_START) {
            _homeStable = 0;
            enterState(PISTON_CONTROLLER_HOME_EXTEND);
        }
        break;
    case PISTON_CONTROLLER_HOME_EXTEND:
        setValveState(EXTEND);
        if (millis() - _lastStateChange > MIN_TIME_HOME_EXTEND) {
            if (_homeExtended == _currentPosition) {
                _homeStable++;
            } else {
                _homeExtended = _currentPosition;
                _homeStable = 0;
            }
        }
        if (millis() - _lastStateChange > TIMEOUT_HOME_EXTEND
            || _homeStable >= 30) {
            _homeExtended = _currentPosition;
            _homeStable = 0;
            enterState(PISTON_CONTROLLER_HOME_RETRACT);
        }
        break;
    case PISTON_CONTROLLER_HOME_RETRACT:
        setValveState(RETRACT);
        if (millis() - _lastStateChange > MIN_TIME_HOME_RETRACT) {
            if (_homeRetracted == _currentPosition) {
                _homeStable++;
            } else {
                _homeRetracted = _currentPosition;
                _homeStable = 0;
            }
        }
        if (millis() - _lastStateChange > TIMEOUT_HOME_RETRACT
            || _homeStable >= 30) {
            _homeRetracted = _currentPosition;
            _travelLength = _homeExtended - _homeRetracted;
            _currentPosition = 0;
            debugPrintf("Homing complete. Travel length %ld\n", _travelLength);
            enterState(PISTON_CONTROLLER_HOMED);
            _isJogging = true; // Reset PID on entering homed state
            resetPID();
        }
        break;
    case PISTON_CONTROLLER_HOMED:
        if (_dmxData[DMX_CHAN_CTRL] == 127) {
            enterState(PISTON_CONTROLLER_REHOME_START);
        }
        if (_dmxData[DMX_CHAN_CTRL] == 63) {
            dmxLinear();
            //runSimple();
            runPID();
        } else {
            setValveState(HOLD);
        }
        break;
    case PISTON_CONTROLLER_REHOME_START:
        setValveState(HOLD);
        if (_dmxData[DMX_CHAN_CTRL] != 127) {
            enterState(PISTON_CONTROLLER_HOMED);
        }
        if (millis() - _lastStateChange > TIMEOUT_HOME_START) {
            _homeStable = 0;
            enterState(PISTON_CONTROLLER_HOME_EXTEND);
        }
        break;
    }
}

void PistonController::readDmx() {
    dmx_packet_t packet;

    if (dmx_receive(_dmxPort, &packet, DMX_TIMEOUT_TICK)) {
        if (!packet.err) {
            dmx_read(1, _dmxData, packet.size);
        }
    }
}

void PistonController::dmxLinear()
{
    uint16_t dmxPosition = (_dmxData[DMX_CHAN_POS_MSB] << 8)
                       | _dmxData[DMX_CHAN_POS_LSB];

    // Beware of overflows
    uint64_t targetPosition = (65535 - dmxPosition);
    targetPosition *= _travelLength;
    targetPosition /= 65535;

    //debugPrintf("Got DMX position %u, TravelLength = %ld, Target Position = %lu\n",
    //            dmxPosition, _travelLength, targetPosition);
    setTargetPosition(targetPosition);
}

void PistonController::updateEncoder() {
    static uint8_t oldAB = 0;
    uint8_t newAB = (digitalRead(_encoderPinA) << 1) | digitalRead(_encoderPinB);

    // Lookup table for encoder state changes
    static const int8_t lookup[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

    oldAB = ((oldAB << 2) | newAB) & 0x0F;
    _currentPosition += lookup[oldAB];
}

void PistonController::resetPID() {
    debugPrintf("Resetting PID...");
    _integral = 0; // Reset integral term when target changes
    _lastError = 0;
    _lastTime = millis();
}

void PistonController::setTargetPosition(long position) {
    if (_targetPosition == position) {
        if (_isJogging) {
            //resetPID();
            _isJogging = false;
        }
        return;
    }

    _targetPosition = position;
    //resetPID();
    //debugPrintf("New target position set: %ld, Current Position: %ld\n", position, _currentPosition);
}

void PistonController::runSimple() {
    float error = _targetPosition - _currentPosition;
    // Determine valve state based on PID output
    if (abs(error) <= _positionTolerance) {
        setValveState(HOLD);
    } else if (error > 0) {
        setValveState(EXTEND);
    } else {
        setValveState(RETRACT);
    }
}

void PistonController::runPID() {
    if (!_runPID)
        return;

    _runPID = false;

    float error = _targetPosition - _currentPosition;

    float output = calculatePID(error);

    // Determine valve state based on PID output
    if (abs(error) <= _positionTolerance) {
        setValveState(HOLD);
    } else if (output > 0) {
        _dutyCycle2 = 0;
        if (output > 100) {
            _dutyCycle1 = 255;
        } else {
            _dutyCycle1 = 2.55 * output;
            if (_dutyCycle1 < VALVE_DUTY_THRESHOLD)
                _dutyCycle1 = 0;
        }
    } else {
        _dutyCycle1 = 0;
        if (output < -100.0) {
            _dutyCycle2 = 255;
        } else {
            _dutyCycle2 = abs(output) * 2.55;
            if (_dutyCycle2 < VALVE_DUTY_THRESHOLD)
                _dutyCycle2 = 0;
        }
    }

    debugPrintf("%ld Position: %ld, Target: %ld, Error: %.2f, Output: %.2f Duty1: %u Duty2: %u\n",
                millis(), _currentPosition, _targetPosition, error, output, _dutyCycle1, _dutyCycle2);
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
    if (state == _currentValveState)
        return;

    _currentValveState = state;
    switch (state) {
        case EXTEND:
            _dutyCycle1 = 255;
            _dutyCycle2 = 0;
            debugPrintf("Valve state changed to: EXTEND\n");
            break;
        case RETRACT:
            _dutyCycle1 = 0;
            _dutyCycle2 = 255;
            debugPrintf("Valve state changed to: RETRACT\n");
            break;
        case HOLD:
            _dutyCycle1 = 0;
            _dutyCycle2 = 0;
            debugPrintf("Valve state changed to: HOLD\n");
            break;
    }
}
