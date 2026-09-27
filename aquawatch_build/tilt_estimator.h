#pragma once
#include <math.h>

// Gravity-referenced inclinations, in degrees, relative to the startup pose.
// Accelerations are in g; gyro rates are in degrees/second in SENSOR axes.
class TiltEstimator {
 public:
  struct Vector { float x, y, z; };

  bool begin(Vector acceleration) {
    ready_ = false;
    if (!gravityReliable(acceleration)) return false;
    up_ = unit(acceleration);
    // Preserve sensor +Y where possible. For a flat board this gives X=+X;
    // for sensor +X pointing up it gives X=-Z, matching the old gyro mapping.
    Vector preferredY = {0, 1, 0};
    if (fabsf(up_.y) > .9f) preferredY = {0, 0, -1};
    axisY_ = unit(subtract(preferredY, scale(up_, dot(preferredY, up_))));
    axisX_ = cross(axisY_, up_);
    gravity_ = up_;
    x_ = y_ = 0;
    ready_ = true;
    return true;
  }

  bool update(Vector acceleration, Vector gyro, float dt) {
    if (!ready_ || !finite(acceleration) || !finite(gyro) ||
        !isfinite(dt) || dt <= 0) return false;
    const bool reliable = gravityReliable(acceleration);
    if (dt > .2f) {
      // A blocked loop cannot reconstruct missing gyro motion. Recover the
      // current tilt from gravity, while keeping the original zero reference.
      if (!reliable) return false;
      gravity_ = unit(acceleration);
    } else {
      const float radians = .0174532925199433f;
      Vector omega = scale(gyro, radians);
      float speed = length(omega);
      if (speed > 1e-6f) {
        // A world-fixed gravity vector rotates opposite to the sensor body.
        Vector axis = scale(omega, 1.0f / speed);
        float angle = speed * dt, c = cosf(angle), s = sinf(angle);
        gravity_ = add(add(scale(gravity_, c), scale(cross(gravity_, axis), s)),
                       scale(axis, dot(axis, gravity_) * (1.0f - c)));
      }
      if (reliable) {
        Vector measured = unit(acceleration);
        // Time-based correction for both axes; never decay angles toward zero.
        float weight = 1.0f - expf(-dt / .5f);
        if (dot(gravity_, measured) < -.95f) gravity_ = measured;
        else gravity_ = unit(add(scale(gravity_, 1.0f - weight), scale(measured, weight)));
      }
    }
    gravity_ = unit(gravity_);
    float x = dot(gravity_, axisX_), y = dot(gravity_, axisY_), z = dot(gravity_, up_);
    const float degrees = 57.2957795130823f;
    // Plane inclinations retain the inverted quadrant (>90 degrees) needed
    // by the distress alarm. These are tilt angles, not yaw/Euler orientation.
    x_ = hypotf(y, z) > .001f ? atan2f(y, z) * degrees : 0;
    y_ = hypotf(x, z) > .001f ? atan2f(-x, z) * degrees : 0;
    return true;
  }

  float x() const { return x_; }
  float y() const { return y_; }

 private:
  static bool finite(Vector v) { return isfinite(v.x) && isfinite(v.y) && isfinite(v.z); }
  static float dot(Vector a, Vector b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
  static float length(Vector v) { return sqrtf(dot(v, v)); }
  static Vector scale(Vector v, float s) { return {v.x*s, v.y*s, v.z*s}; }
  static Vector add(Vector a, Vector b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
  static Vector subtract(Vector a, Vector b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
  static Vector cross(Vector a, Vector b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
  }
  static Vector unit(Vector v) { return scale(v, 1.0f / length(v)); }
  static bool gravityReliable(Vector v) {
    float magnitude = length(v);
    return finite(v) && magnitude >= .85f && magnitude <= 1.15f;
  }
  Vector up_ = {0, 0, 1}, axisX_ = {1, 0, 0}, axisY_ = {0, 1, 0};
  Vector gravity_ = {0, 0, 1};
  float x_ = 0, y_ = 0;
  bool ready_ = false;
};
