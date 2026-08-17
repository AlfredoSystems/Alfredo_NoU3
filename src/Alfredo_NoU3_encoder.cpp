#include "Alfredo_NoU3_encoder.h"

// Static members
Encoder* Encoder::instances[MAX_ENCODERS] = {nullptr};
uint8_t Encoder::numEncoders = 0;

Encoder::Encoder()
    : _pinA(0), _pinB(0), _position(0), _prevState(0), _index(MAX_ENCODERS),
      _mux(portMUX_INITIALIZER_UNLOCKED) {
    // Slot registration is deferred to begin() so that motors without
    // encoders don't consume one of the 8 available slots.
}

void Encoder::begin(uint8_t pinA, uint8_t pinB) {
    // Register on first call; re-calling begin() just updates the pins.
    if (_index >= MAX_ENCODERS) {
        if (numEncoders >= MAX_ENCODERS) return;
        _index = numEncoders;
        instances[numEncoders] = this;
        numEncoders++;
    } else {
        // Already registered: detach the old pins so they stop feeding
        // this encoder's ISR before we switch to the new ones.
        detachInterrupt(digitalPinToInterrupt(_pinA));
        detachInterrupt(digitalPinToInterrupt(_pinB));
    }

    _pinA = pinA;
    _pinB = pinB;

    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    _prevState = (digitalRead(_pinA) << 1) | digitalRead(_pinB);

    void (*isrFunc)();

    switch (_index) {
        case 0: isrFunc = isr0; break;
        case 1: isrFunc = isr1; break;
        case 2: isrFunc = isr2; break;
        case 3: isrFunc = isr3; break;
        case 4: isrFunc = isr4; break;
        case 5: isrFunc = isr5; break;
        case 6: isrFunc = isr6; break;
        case 7: isrFunc = isr7; break;
        default: return;
    }

    attachInterrupt(digitalPinToInterrupt(pinA), isrFunc, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB), isrFunc, CHANGE);
}

int32_t Encoder::getPosition() {
    portENTER_CRITICAL(&_mux);
    int32_t pos = _position;
    portEXIT_CRITICAL(&_mux);
    return pos;
}

void Encoder::resetPosition() {
    portENTER_CRITICAL(&_mux);
    _position = 0;
    portEXIT_CRITICAL(&_mux);
}

void Encoder::update() {
    uint8_t state = (digitalRead(_pinA) << 1) | digitalRead(_pinB);
    uint8_t transition = (_prevState << 2) | state;

    static const int8_t dirLookup[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    portENTER_CRITICAL_ISR(&_mux);
    _position += dirLookup[transition];
    _prevState = state;
    portEXIT_CRITICAL_ISR(&_mux);
}

// Static ISR stubs — call back into instance
void Encoder::isr0() { if (instances[0]) instances[0]->update(); }
void Encoder::isr1() { if (instances[1]) instances[1]->update(); }
void Encoder::isr2() { if (instances[2]) instances[2]->update(); }
void Encoder::isr3() { if (instances[3]) instances[3]->update(); }
void Encoder::isr4() { if (instances[4]) instances[4]->update(); }
void Encoder::isr5() { if (instances[5]) instances[5]->update(); }
void Encoder::isr6() { if (instances[6]) instances[6]->update(); }
void Encoder::isr7() { if (instances[7]) instances[7]->update(); }
