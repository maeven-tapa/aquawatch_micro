#pragma once
#include <stdint.h>
#include <math.h>

// A failed stationary window is retried without disabling sensor reads.
// Values: gyro X/Y/Z in deg/s, then acceleration X/Y/Z in g.
class ImuCalibration {
 public:
  enum Status { COLLECTING, MOTION, HIGH_RATE, INVALID_GRAVITY, COMPLETE };

  bool addSample(const float values[6], uint32_t now) {
    if (status_ == COMPLETE) return true; // Never re-zero a running device.
    if (count_ && uint32_t(now - lastAt_) > 200) clearWindow();
    for (int i = 0; i < 6; i++) {
      if (!isfinite(values[i])) { reject(INVALID_GRAVITY); return false; }
    }
    if (!count_) startedAt_ = now;
    lastAt_ = now;
    ++count_;
    // Welford variance avoids subtracting two nearly equal squared sums.
    for (int i = 0; i < 6; i++) {
      float delta = values[i] - mean_[i];
      mean_[i] += delta / count_;
      m2_[i] += delta * (values[i] - mean_[i]);
    }
    if (uint32_t(now - startedAt_) < 1000 || count_ < 20) return false;
    for (int i = 0; i < 6; i++) {
      if (m2_[i] / count_ > (i < 3 ? 1.0f : .01f)) { reject(MOTION); return false; }
      if (i < 3 && fabsf(mean_[i]) > 5.0f) { reject(HIGH_RATE); return false; }
    }
    float magnitude = sqrtf(mean_[3]*mean_[3] + mean_[4]*mean_[4] + mean_[5]*mean_[5]);
    // The magnitude matters, not which sensor axis points up or down.
    if (magnitude < .85f || magnitude > 1.15f) { reject(INVALID_GRAVITY); return false; }
    status_ = COMPLETE;
    return true;
  }

  float mean(int axis) const { return mean_[axis]; }
  uint16_t samples() const { return count_; }
  Status status() const { return status_; }
  void reject(Status reason) { clearWindow(); status_ = reason; }

 private:
  void clearWindow() {
    count_ = 0;
    for (int i = 0; i < 6; i++) mean_[i] = m2_[i] = 0;
  }
  float mean_[6] = {}, m2_[6] = {};
  uint16_t count_ = 0;
  uint32_t startedAt_ = 0, lastAt_ = 0;
  Status status_ = COLLECTING;
};
