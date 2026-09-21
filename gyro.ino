#include <Wire.h>
#include <DFRobot_BMI160.h>
#include <math.h>

// =====================================================
//              NODEMCU + BMI160 ROTATION TEST
// =====================================================

// BMI160 I2C address
// 0x69 = SDO HIGH
// 0x68 = SDO LOW
const int8_t I2C_ADDR = 0x69;

DFRobot_BMI160 bmi160;

// NodeMCU I2C
// D2 = GPIO4  = SDA
// D1 = GPIO5  = SCL


// =====================================================
//                    ROTATION
// =====================================================

float rotationX = 0.0;
float rotationY = 0.0;

float gyroOffsetX = 0.0;
float gyroOffsetY = 0.0;

unsigned long lastGyroTime = 0;


// BMI160 default gyro range = ±2000 dps
// 16.4 LSB per degree/second
const float GYRO_SCALE = 16.4;


// =====================================================
//                 LINEAR ACCELERATION
// =====================================================

const float GRAVITY = 9.80665;

float linearAcceleration = 0.0;


// =====================================================
//                       TARE
// =====================================================

bool tared = false;


// =====================================================
//                  CALIBRATE GYRO
// =====================================================

void calibrateGyro()
{
  Serial.println();
  Serial.println("=================================");
  Serial.println("       GYRO CALIBRATION");
  Serial.println("=================================");
  Serial.println("Keep the BMI160 completely STILL!");

  float sumX = 0.0;
  float sumY = 0.0;

  int validSamples = 0;

  int16_t data[6];

  for (int i = 0; i < 500; i++)
  {
    if (bmi160.getAccelGyroData(data) == BMI160_OK)
    {
      // DFRobot BMI160:
      // data[0] = Gyro X
      // data[1] = Gyro Y
      // data[2] = Gyro Z
      // data[3] = Accel X
      // data[4] = Accel Y
      // data[5] = Accel Z

      float gx = -(data[2] / GYRO_SCALE);
      float gy = data[1] / GYRO_SCALE;

      sumX += gx;
      sumY += gy;

      validSamples++;
    }

    delay(5);
  }

  if (validSamples > 0)
  {
    gyroOffsetX = sumX / validSamples;
    gyroOffsetY = sumY / validSamples;
  }

  rotationX = 0.0;
  rotationY = 0.0;

  lastGyroTime = micros();

  Serial.print("Gyro X offset: ");
  Serial.print(gyroOffsetX, 4);
  Serial.println(" deg/s");

  Serial.print("Gyro Y offset: ");
  Serial.print(gyroOffsetY, 4);
  Serial.println(" deg/s");

  Serial.println("Gyro calibrated!");
}


// =====================================================
//              UPDATE X/Y ROTATION
// =====================================================

void updateRotation()
{
  int16_t data[6];

  if (bmi160.getAccelGyroData(data) != BMI160_OK)
    return;


  // ---------------------------------------------------
  // Read gyro
  // ---------------------------------------------------

  float gx = -(data[2] / GYRO_SCALE);
  float gy = (data[1] / GYRO_SCALE);


  // ---------------------------------------------------
  // Remove gyro zero offset
  // ---------------------------------------------------

  gx -= gyroOffsetX;
  gy -= gyroOffsetY;


  // ---------------------------------------------------
  // Deadband
  // Prevent tiny gyro noise from accumulating
  // ---------------------------------------------------

  if (fabs(gx) < 0.5)
    gx = 0.0;

  if (fabs(gy) < 0.5)
    gy = 0.0;


  // ---------------------------------------------------
  // Calculate time
  // ---------------------------------------------------

  unsigned long now = micros();

  float dt = (now - lastGyroTime) / 1000000.0;

  lastGyroTime = now;


  // Ignore abnormal time gaps
  if (dt <= 0.0 || dt > 0.1)
    return;


  // ---------------------------------------------------
  // Integrate gyro
  // ---------------------------------------------------

  rotationX += gx * dt;
  rotationY += gy * dt;


  // ===================================================
  //                  X WRAP
  // ===================================================

  if (rotationX >= 360.0)
    rotationX -= 360.0;

  if (rotationX <= -360.0)
    rotationX += 360.0;


  // ===================================================
  //                  Y WRAP
  // ===================================================

  if (rotationY >= 360.0)
    rotationY -= 360.0;

  if (rotationY <= -360.0)
    rotationY += 360.0;
}


// =====================================================
//              CALCULATE LINEAR ACCELERATION
// =====================================================

float getLinearAcceleration()
{
  int16_t data[6];

  if (bmi160.getAccelGyroData(data) != BMI160_OK)
    return 0.0;


  // Raw accelerometer → g
  float ax = data[3] / 16384.0;
  float ay = data[4] / 16384.0;
  float az = data[5] / 16384.0;


  // g → m/s²
  ax *= GRAVITY;
  ay *= GRAVITY;
  az *= GRAVITY;


  // ---------------------------------------------------
  // Total measured acceleration
  // ---------------------------------------------------

  float totalAccel =
      sqrt(
        ax * ax +
        ay * ay +
        az * az
      );


  // ---------------------------------------------------
  // Estimate gravity direction
  //
  // Normalize measured acceleration vector.
  // When stationary, this represents approximately
  // the gravity vector.
  // ---------------------------------------------------

  if (totalAccel < 0.1)
    return 0.0;


  float gravityX = (ax / totalAccel) * GRAVITY;
  float gravityY = (ay / totalAccel) * GRAVITY;
  float gravityZ = (az / totalAccel) * GRAVITY;


  // ---------------------------------------------------
  // Remove gravity
  // ---------------------------------------------------

  float linearX = ax - gravityX;
  float linearY = ay - gravityY;
  float linearZ = az - gravityZ;


  // ---------------------------------------------------
  // Magnitude of dynamic acceleration
  // ---------------------------------------------------

  float linearAccel =
      sqrt(
        linearX * linearX +
        linearY * linearY +
        linearZ * linearZ
      );


  // Small noise deadband
  if (linearAccel < 0.20)
    linearAccel = 0.0;


  return linearAccel;
}


// =====================================================
//                         TARE
// =====================================================

void tare()
{
  rotationX = 0.0;
  rotationY = 0.0;

  linearAcceleration = 0.0;

  lastGyroTime = micros();

  tared = true;

  Serial.println();
  Serial.println("=================================");
  Serial.println("             TARED!");
  Serial.println("=================================");
  Serial.println("X Rotation = 0.0 deg");
  Serial.println("Y Rotation = 0.0 deg");
  Serial.println("Logging started.");
  Serial.println();
}


// =====================================================
//                         SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);


  // =================================================
  // I2C
  // =================================================

  // SDA = D2 / GPIO4
  // SCL = D1 / GPIO5

  Wire.begin(D2, D1);


  // =================================================
  // HEADER
  // =================================================

  Serial.println();
  Serial.println("=================================");
  Serial.println("    NODEMCU + BMI160 TILT TEST");
  Serial.println("=================================");


  // =================================================
  // BMI160 INIT
  // =================================================

  int8_t result = bmi160.I2cInit(I2C_ADDR);

  if (result != BMI160_OK)
  {
    Serial.println("BMI160 NOT detected!");
    Serial.println("Check:");
    Serial.println("- VCC");
    Serial.println("- GND");
    Serial.println("- SDA = D2");
    Serial.println("- SCL = D1");
    Serial.println("- I2C address = 0x69 / 0x68");

    while (1)
    {
      delay(1000);
    }
  }

  Serial.println("BMI160 detected!");


  // =================================================
  // GYRO CALIBRATION
  // =================================================

  calibrateGyro();


  // =================================================
  // TARE MODE
  // =================================================

  Serial.println();
  Serial.println("=================================");
  Serial.println("             TARE MODE");
  Serial.println("=================================");
  Serial.println("Place the sensor in your");
  Serial.println("desired ZERO position.");
  Serial.println();
  Serial.println("Press T to set ZERO.");
  Serial.println("No logging before T.");
  Serial.println();

  tared = false;

  lastGyroTime = micros();
}


// =====================================================
//                          LOOP
// =====================================================

void loop()
{
  // =================================================
  // WAIT FOR TARE
  // =================================================

  if (!tared)
  {
    if (Serial.available())
    {
      char command = Serial.read();

      if (command == 'T' || command == 't')
      {
        tare();
      }
    }

    return;
  }


  // =================================================
  // PRESS T AGAIN TO RESET ZERO
  // =================================================

  if (Serial.available())
  {
    char command = Serial.read();

    if (command == 'T' || command == 't')
    {
      tare();
    }
  }


  // =================================================
  // UPDATE ROTATION
  // =================================================

  updateRotation();


  // =================================================
  // LINEAR ACCELERATION
  // =================================================

  linearAcceleration = getLinearAcceleration();


  // =================================================
  // PRINT EVERY 500 ms
  // =================================================

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 500)
  {
    lastPrint = millis();

    Serial.println("--------------------------------");

    Serial.print("X Rotation: ");
    Serial.print(rotationX, 1);
    Serial.println(" deg");

    Serial.print("Y Rotation: ");
    Serial.print(rotationY, 1);
    Serial.println(" deg");

    Serial.print("Acceleration: ");
    Serial.print(linearAcceleration, 2);
    Serial.println(" m/s^2");
  }
}