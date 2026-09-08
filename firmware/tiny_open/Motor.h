// =============================================================================
//  Motor.h  -  TB6612FNG single-channel motor driver wrapper.
//
//  Direction via IN1/IN2, speed via PWM (LEDC). Signed duty in [-PWM_MAX, +PWM_MAX]:
//    duty > 0 -> forward  (IN1=H, IN2=L)
//    duty < 0 -> reverse  (IN1=L, IN2=H)
//    duty = 0 -> coast    (IN1=L, IN2=L)
//  A "forward" command should make the wheel move the robot forward; if not,
//  flip MOTOR_DIR for that wheel in config.h.
// =============================================================================
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
  // dir: +1 or -1, applied to every command (wiring polarity correction).
  void begin(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t pwmCh,
             uint32_t freqHz, uint8_t resBits, int8_t dir) {
    _in1 = in1; _in2 = in2; _pwmCh = pwmCh; _dir = dir;
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    // ESP32 Arduino core 2.0.x LEDC API:
    ledcSetup(_pwmCh, freqHz, resBits);
    ledcAttachPin(pwmPin, _pwmCh);
    _max = (1 << resBits) - 1;
    stop();
  }

  // Drive with a signed duty value. Values outside +/-_max are clamped.
  void drive(float signedDuty) {
    signedDuty *= _dir;
    if (signedDuty > _max)  signedDuty = _max;
    if (signedDuty < -_max) signedDuty = -_max;

    int duty = (int)(signedDuty >= 0 ? signedDuty + 0.5f : signedDuty - 0.5f);

    if (duty > 0) {
      digitalWrite(_in1, HIGH);
      digitalWrite(_in2, LOW);
      ledcWrite(_pwmCh, duty);
    } else if (duty < 0) {
      digitalWrite(_in1, LOW);
      digitalWrite(_in2, HIGH);
      ledcWrite(_pwmCh, -duty);
    } else {
      stop();
    }
  }

  void stop() {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    ledcWrite(_pwmCh, 0);
  }

private:
  uint8_t _in1 = 0, _in2 = 0, _pwmCh = 0;
  int8_t  _dir = 1;
  int     _max = 1023;
};

#endif // MOTOR_H
