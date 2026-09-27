#include "../imu_calibration.h"
#include "../tilt_estimator.h"
#include <stdio.h>
#include <stdlib.h>

void require(bool ok, const char *message) {
  if (!ok) { fprintf(stderr, "FAIL: %s\n", message); exit(1); }
}
void window(ImuCalibration &calibration, float values[6], uint32_t &now) {
  for (int i=0; i<26; i++, now+=40) calibration.addSample(values, now);
}
void verifyReference(ImuCalibration &calibration) {
  require(calibration.status() == ImuCalibration::COMPLETE, "calibration completes");
  TiltEstimator tilt;
  require(tilt.begin({calibration.mean(3), calibration.mean(4), calibration.mean(5)}), "calibrated gravity initializes tilt");
  require(fabsf(tilt.x()) < .01f && fabsf(tilt.y()) < .01f, "startup pose is zero");
}
int main() {
  const float poses[][3] = {{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};
  for (const auto &pose : poses) {
    ImuCalibration calibration;
    uint32_t now=100;
    float values[] = {.2f, -.15f, .05f, pose[0], pose[1], pose[2]};
    // A rejected startup window must recover using the same object.
    for (int i=0; i<26; i++, now+=40) {
      values[0] = i%2 ? 2.0f : -2.0f;
      require(!calibration.addSample(values, now), "movement must not calibrate");
    }
    require(calibration.status() == ImuCalibration::MOTION, "report movement instead of unavailable");
    values[0]=.2f;
    window(calibration, values, now);
    verifyReference(calibration);
    require(fabsf(calibration.mean(0)-.2f) < .0001f, "stable gyro bias learned after retry");
    float originalGravity[] = {calibration.mean(3), calibration.mean(4), calibration.mean(5)};
    values[3]=-pose[0]; values[4]=-pose[1]; values[5]=-pose[2];
    window(calibration, values, now);
    for (int i=0; i<3; i++) require(calibration.mean(i+3)==originalGravity[i], "later tilt must not reset the reference");
  }
  {
    ImuCalibration calibration;
    uint32_t now=0;
    float values[] = {8,0,0, 0,1,0};
    window(calibration, values, now);
    require(calibration.status()==ImuCalibration::HIGH_RATE, "report high gyro rate");
    values[0]=0; values[4]=1.5f;
    window(calibration, values, now);
    require(calibration.status()==ImuCalibration::INVALID_GRAVITY, "report invalid acceleration magnitude");
    values[4]=1;
    window(calibration, values, now);
    verifyReference(calibration);
  }
  {
    ImuCalibration calibration;
    float values[] = {0,0,0, 1,0,0};
    for (uint32_t now=0; now<=800; now+=40) require(!calibration.addSample(values, now), "incomplete window");
    uint32_t now=3000;
    require(!calibration.addSample(values, now), "read gap restarts stationary window");
    require(calibration.samples()==1, "old samples discarded after read gap");
    for (int i=1; i<=25; i++) calibration.addSample(values, now+i*40);
    verifyReference(calibration);
  }
  {
    ImuCalibration calibration;
    float values[] = {0,0,0, 0,0,-1};
    uint32_t now=UINT32_MAX-500;
    window(calibration, values, now);
    verifyReference(calibration);
  }
  {
    ImuCalibration calibration;
    float values[] = {0,0,0, NAN,0,1};
    require(!calibration.addSample(values, 0), "nonfinite samples rejected");
    values[3]=0;
    uint32_t now=40;
    window(calibration, values, now);
    verifyReference(calibration);
  }
  puts("PASS: all 6 startup orientations; movement rejection then automatic retry; gyro/gravity diagnostics; read-gap recovery; timer rollover; fixed reference after calibration.");
}
