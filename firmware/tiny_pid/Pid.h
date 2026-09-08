// =============================================================================
//  Pid.h  -  Velocity PI(D) controller for one wheel, with feedforward-aware
//            anti-windup. Adapted from mecanum_base/Pid.h.
//
//  Input : setpoint (target rad/s), measurement (filtered rad/s), dt (s),
//          ff (feedforward duty from the identified static motor model)
//  Output: signed PWM duty, clamped to [-outMax, +outMax]
//
//  WHY ff IS A PARAMETER instead of being added by the caller afterwards:
//  the saturation clamp and the conditional-integration anti-windup must see
//  the TOTAL output (ff + p + i + d). If ff were added outside, the PID would
//  clamp only its own share, never detect that the motor is saturated, and the
//  integrator would wind up whenever |ff| is large (worst at speed reversals).
//
//  Other features kept from the original:
//   - Integrator clamped to +/-iMax, and only committed when not pushing
//     further into saturation (conditional integration).
//   - Derivative on measurement (not error) to avoid setpoint kick.
// =============================================================================
#ifndef PID_H
#define PID_H

class Pid {
public:
  void begin(float kp, float ki, float kd, float iMax, float outMax) {
    _kp = kp; _ki = ki; _kd = kd;
    _iMax = iMax; _outMax = outMax;
    reset();
  }

  void reset() {
    _integral = 0.0f;
    _prevMeas = 0.0f;
    _first = true;
  }

  // Live gain update (from the /pid_gains topic). The integrator state is
  // intentionally PRESERVED so retuning while driving does not cause a duty
  // step; it stays bounded by the unchanged iMax.
  void setGains(float kp, float ki, float kd) {
    _kp = kp; _ki = ki; _kd = kd;
  }

  float update(float setpoint, float measurement, float dt, float ff = 0.0f) {
    float error = setpoint - measurement;

    // Proportional
    float p = _kp * error;

    // Derivative on measurement. Skip the very first sample.
    float d = 0.0f;
    if (!_first && dt > 0.0f) {
      d = -_kd * (measurement - _prevMeas) / dt;
    }
    _prevMeas = measurement;
    _first = false;

    // Tentative integral
    float integral = _integral + _ki * error * dt;
    integral = clamp(integral, -_iMax, _iMax);

    // Clamp the TOTAL (see header comment).
    float out = ff + p + integral + d;
    if (out > _outMax) {
      out = _outMax;
      if (error < 0.0f) _integral = integral;   // allow unwinding
    } else if (out < -_outMax) {
      out = -_outMax;
      if (error > 0.0f) _integral = integral;   // allow unwinding
    } else {
      _integral = integral;                     // not saturated: commit
    }

    return out;
  }

private:
  static float clamp(float v, float lo, float hi) {
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
  }

  float _kp = 0, _ki = 0, _kd = 0;
  float _iMax = 0, _outMax = 0;
  float _integral = 0;
  float _prevMeas = 0;
  bool  _first = true;
};

#endif // PID_H
