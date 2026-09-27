#include "../tilt_estimator.h"
#include <stdio.h>
#include <stdlib.h>
#include <initializer_list>

using V = TiltEstimator::Vector;
const float RAD = .0174532925199433f;
const float DT = .04f;

void require(bool condition, const char *message) {
  if (!condition) { fprintf(stderr, "FAIL: %s\n", message); exit(1); }
}
void near(float actual, float expected, float tolerance, const char *message) {
  float error = remainderf(actual - expected, 360.0f);
  if (fabsf(error) > tolerance) {
    fprintf(stderr, "FAIL: %s (actual %.3f, expected %.3f)\n", message, actual, expected);
    exit(1);
  }
}
struct Mount {
  V x, y, up;
  V sensor(V body) const {
    return {x.x*body.x + y.x*body.y + up.x*body.z,
            x.y*body.x + y.y*body.y + up.y*body.z,
            x.z*body.x + y.z*body.y + up.z*body.z};
  }
};
V gravityAt(int axis, float angle) {
  return axis == 0 ? V{0, sinf(angle*RAD), cosf(angle*RAD)}
                   : V{-sinf(angle*RAD), 0, cosf(angle*RAD)};
}
float reading(const TiltEstimator &filter, int axis) { return axis == 0 ? filter.x() : filter.y(); }
void step(TiltEstimator &filter, const Mount &mount, int axis, float angle, float rate, float bias = 0) {
  V gyro = axis == 0 ? V{rate+bias, 0, 0} : V{0, rate+bias, 0};
  require(filter.update(mount.sensor(gravityAt(axis, angle)), mount.sensor(gyro), DT), "valid sample accepted");
}

int main() {
  const Mount mounts[] = {
    {{1,0,0}, {0,1,0}, {0,0,1}},       // flat, sensor Z up
    {{0,0,-1}, {0,1,0}, {1,0,0}},      // vertical, sensor X up: old -Z/+Y mapping
    {{1,0,0}, {0,0,-1}, {0,1,0}},      // vertical, sensor Y up
    {{-1,0,0}, {0,1,0}, {0,0,-1}},     // PCB facing downward at startup
    {{.70710678f,0,-.70710678f}, {0,1,0}, {.70710678f,0,.70710678f}}
  };
  for (const Mount &mount : mounts) {
    for (int axis = 0; axis < 2; axis++) {
      TiltEstimator filter;
      require(filter.begin(mount.up), "upright reference accepted");
      float previous = 0;
      // 25 complete back-and-forth cycles with residual gyro bias.
      for (int i = 1; i <= 2500; i++) {
        float angle = 60.0f * sinf(i * 2.0f * 3.14159265f / 100.0f);
        step(filter, mount, axis, angle, (angle-previous)/DT, .3f);
        near(reading(filter, axis), angle, .4f, "cyclic motion must not accumulate drift");
        previous = angle;
      }
      for (int i = 0; i < 250; i++) step(filter, mount, axis, 0, 0, .3f);
      near(reading(filter, axis), 0, .2f, "return to upright");
      // Hold positive/negative angles and positions past the 90-degree alarm limit.
      for (float target : {45.0f, -60.0f, 100.0f, -120.0f, 170.0f, 190.0f, 0.0f}) {
        for (int i = 1; i <= 100; i++) {
          float angle = previous + (target-previous) * i/100.0f;
          step(filter, mount, axis, angle, (target-previous)/(100.0f*DT));
        }
        for (int i = 0; i < 500; i++) step(filter, mount, axis, target, 0, .3f);
        near(reading(filter, axis), target, .2f, "held angle must not decay to zero");
        if (fabsf(target) > 90 && fabsf(target) < 180)
          require(fabsf(filter.x()) >= 90 || fabsf(filter.y()) >= 90, "inverted tilt must reach alarm limit");
        previous = target;
      }
      // Gyro bias alone used to accumulate indefinitely on X.
      for (int i = 0; i < 15000; i++) step(filter, mount, axis, 0, 0, .3f);
      near(reading(filter, axis), 0, .2f, "ten-minute bias remains bounded");
      require(filter.update(mount.sensor(gravityAt(axis, 55)), {0,0,0}, 12.0f), "recover after blocked loop");
      near(reading(filter, axis), 55, .01f, "gap recovery keeps original reference");
    }
  }
  TiltEstimator filter;
  require(!filter.begin({0,0,0}), "reject zero gravity reference");
  require(filter.begin({0,0,1}), "flat reference");
  // Transient 2g accelerations must not pull the tilt estimate sideways.
  for (int i = 0; i < 10; i++) require(filter.update({2,0,0}, {0,0,0}, DT), "gyro prediction during acceleration");
  near(filter.y(), 0, .01f, "reject strong translational acceleration");
  require(!filter.update({2,0,0}, {0,0,0}, 2), "no reliable estimate after gap without gravity");
  require(!filter.update({NAN,0,1}, {0,0,0}, DT), "reject nonfinite sample");
  // Turning flat around gravity is yaw and must not create a false tilt alarm.
  for (int i = 0; i < 1000; i++) require(filter.update({0,0,1}, {0,0,90}, DT), "yaw samples");
  near(filter.x(), 0, .01f, "yaw is not X tilt");
  near(filter.y(), 0, .01f, "yaw is not Y tilt");
  require(filter.update({0,0,-1}, {0,0,0}, DT), "recover antipodal gravity");
  require(fabsf(filter.x()) >= 90 || fabsf(filter.y()) >= 90, "upside-down must not look level");
  puts("PASS: 5 mount orientations; both axes; 25 back-and-forth cycles; held angles; return to zero; 10-minute drift; >90-degree alarm; gaps; acceleration rejection; yaw.");
}
