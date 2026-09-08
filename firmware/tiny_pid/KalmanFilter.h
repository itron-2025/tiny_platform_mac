// =============================================================================
//  KalmanFilter.h  -  Scalar (1D) Kalman filter for one wheel's velocity.
//
//  Model: the wheel speed is treated as a slowly-changing scalar (random walk).
//    Predict:  x stays, uncertainty grows by Q
//                P = P + Q
//    Update:   blend prediction with the noisy measurement z
//                K = P / (P + R)
//                x = x + K * (z - x)
//                P = (1 - K) * P
//
//  Intuition: large R  -> trust the model -> heavy smoothing (but more lag).
//             large Q  -> trust the measurement -> light smoothing (more noise).
//  This is the minimal "real" Kalman filter; for a scalar random walk it reduces
//  to an adaptive low-pass whose gain K self-tunes from Q and R.
// =============================================================================
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter1D {
public:
  void begin(float q, float r) {
    _q = q;
    _r = r;
    _x = 0.0f;
    _p = 1.0f;   // initial uncertainty
  }

  // Feed one raw measurement, get the filtered estimate back.
  float update(float z) {
    // Predict
    _p += _q;
    // Update
    float k = _p / (_p + _r);
    _x += k * (z - _x);
    _p *= (1.0f - k);
    return _x;
  }

  float value() const { return _x; }

  // Reset estimate to a known value (e.g. on safety stop).
  void reset(float x = 0.0f) {
    _x = x;
    _p = 1.0f;
  }

private:
  float _q = 0.5f;   // process noise variance
  float _r = 8.0f;   // measurement noise variance
  float _x = 0.0f;   // state estimate (filtered velocity)
  float _p = 1.0f;   // estimate covariance
};

#endif // KALMAN_FILTER_H
