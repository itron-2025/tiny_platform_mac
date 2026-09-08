// =============================================================================
//  Encoder.h  -  Quadrature encoder with 4x decoding via pin-change interrupts.
//
//  Why 4x: we interrupt on BOTH channels A and B, on every edge, and use a
//  transition lookup table. This gives 4 counts per encoder line, i.e. the
//  finest resolution -> better low-speed velocity estimation.
//
//  Header-only on purpose: it is included exactly once (by the .ino), so the
//  inline ISR and table do not cause multiple-definition errors.
// =============================================================================
#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
  // Quadrature transition table indexed by (prevState<<2 | newState).
  // prev/new states are 2-bit values (A<<1 | B). Valid single steps give +/-1;
  // invalid (missed) transitions give 0.
  static constexpr int8_t QEM[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
  };

  void begin(uint8_t pinA, uint8_t pinB, bool usePullup) {
    _pinA = pinA;
    _pinB = pinB;
    pinMode(_pinA, usePullup ? INPUT_PULLUP : INPUT);
    pinMode(_pinB, usePullup ? INPUT_PULLUP : INPUT);
    _state = (digitalRead(_pinA) << 1) | digitalRead(_pinB);
    _count = 0;
    // attachInterruptArg passes "this" so one static ISR serves every instance.
    attachInterruptArg(digitalPinToInterrupt(_pinA), Encoder::isr, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(_pinB), Encoder::isr, this, CHANGE);
  }

  // Atomically read the accumulated counts since the last call, and reset to 0.
  long readDelta() {
    noInterrupts();
    long c = _count;
    _count = 0;
    interrupts();
    return c;
  }

private:
  uint8_t _pinA = 0, _pinB = 0;
  volatile uint8_t _state = 0;
  volatile long _count = 0;

  // Single ISR for all encoders; "arg" is the Encoder* registered above.
  //
  // Deliberately NOT IRAM_ATTR. The core builds with CONFIG_ARDUINO_ISR_IRAM
  // unset, so gpio_install_isr_service() is called with flag 0 (no
  // ESP_INTR_FLAG_IRAM) and handlers may live in flash. Forcing this
  // in-class (hence inline) method into .iram1 leaves its literal pool behind
  // in flash, which the Xtensa l32r instruction cannot reach backwards ->
  // "dangerous relocation: literal placed after use" at link time. It would
  // also be wrong on its own terms: the body calls digitalRead(), which is
  // itself not in IRAM, so this ISR was never IRAM-safe to begin with.
  static void isr(void *arg) {
    Encoder *e = static_cast<Encoder *>(arg);
    uint8_t s = (digitalRead(e->_pinA) << 1) | digitalRead(e->_pinB);
    uint8_t idx = (e->_state << 2) | s;
    e->_count += QEM[idx];
    e->_state = s;
  }
};

// Definition for the static constexpr array (needed for ODR-use pre-C++17 style).
constexpr int8_t Encoder::QEM[16];

#endif // ENCODER_H
