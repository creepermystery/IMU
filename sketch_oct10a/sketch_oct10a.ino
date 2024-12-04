#include <Wire.h>
#include <math.h>
#define MPU9250_ADDR 0x68

unsigned long lastTime = 0;
double velocity[3] = {0, 0, 0}; // Initial velocities in m/s for x, y, z
double position[3] = {0, 0, 0}; // Initial positions in meters for x, y, z
double accelBias[3] = {0, 0, 0}; // Bias for accelerometer to reduce drift
double velocityThreshold = 0.05; // Threshold to reduce drift due to noise
double acceleration[3] = {0, 0, 0}; // Store acceleration values for output

double pitch = 0.0; // Pitch angle
double roll = 0.0;  // Roll angle

// Kalman filter variables
struct Kalman {
  double q; // Process noise covariance
  double r; // Measurement noise covariance
  double x; // Value
  double p; // Estimation error covariance
  double k; // Kalman gain
};

Kalman kalmanX = {0.001, 0.1, 0, 1, 0};
Kalman kalmanY = {0.001, 0.1, 0, 1, 0};
Kalman kalmanZ = {0.001, 0.1, 0, 1, 0};

// Kalman filter function
double kalmanUpdate(Kalman &kalman, double measurement) {
  // Prediction update
  kalman.p = kalman.p + kalman.q;
  // Measurement update
  kalman.k = kalman.p / (kalman.p + kalman.r);
  kalman.x = kalman.x + kalman.k * (measurement - kalman.x);
  kalman.p = (1 - kalman.k) * kalman.p;
  return kalman.x;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin();

  // Initialize the MPU9250
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x6B); // Power management register
  Wire.write(0);    // Wake up the MPU9250
  if (Wire.endTransmission() != 0) {
    Serial.println("MPU9250 initialization unsuccessful");
    Serial.println("Check connections and try again");
    while (1) {}
  }

  // Calculate bias by averaging initial readings
  const int numSamples = 500;
  for (int i = 0; i < numSamples; i++) {
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(0x3B); // Starting register for accelerometer data
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDR, 6, true);

    int16_t rawAx = Wire.read() << 8 | Wire.read();
    int16_t rawAy = Wire.read() << 8 | Wire.read();
    int16_t rawAz = Wire.read() << 8 | Wire.read();

    accelBias[0] += (rawAx / 16384.0) * 9.81;
    accelBias[1] += (rawAy / 16384.0) * 9.81;
    accelBias[2] += (rawAz / 16384.0) * 9.81;
    delay(10);
  }

  accelBias[0] /= numSamples;
  accelBias[1] /= numSamples;
  accelBias[2] /= numSamples;

  // Adjust bias to compensate for gravity (assuming z-axis is aligned with gravity)
  accelBias[2] -= 9.81;

  // Initialize the last time for integration
  lastTime = micros();
}

void loop() {
  // Read acceleration values from MPU9250
  Wire.beginTransmission(MPU9250_ADDR);
  Wire.write(0x3B); // Starting register for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDR, 6, true);

  int16_t rawAx = Wire.read() << 8 | Wire.read();
  int16_t rawAy = Wire.read() << 8 | Wire.read();
  int16_t rawAz = Wire.read() << 8 | Wire.read();

  // Convert raw values to acceleration in m/s^2 and apply bias correction
  double ax = (rawAx / 16384.0) * 9.81 - accelBias[0];
  double ay = (rawAy / 16384.0) * 9.81 - accelBias[1];
  double az = (rawAz / 16384.0) * 9.81 - accelBias[2];

  // Apply Kalman filter to acceleration values
  ax = kalmanUpdate(kalmanX, ax);
  ay = kalmanUpdate(kalmanY, ay);
  az = kalmanUpdate(kalmanZ, az);

  // Store the corrected acceleration values
  acceleration[0] = ax;
  acceleration[1] = ay;
  acceleration[2] = az;

  // Calculate delta time
  unsigned long currentTime = micros();
  double dt = (currentTime - lastTime) / 1000000.0; // Delta time in seconds
  lastTime = currentTime;

  // Use a more robust method to determine the change in velocity
  for (int i = 0; i < 3; i++) {
    if (abs(acceleration[i]) > velocityThreshold) {
      velocity[i] += acceleration[i] * dt;
    } else {
      velocity[i] *= 0.85; // Dampen velocity more aggressively when below threshold to reduce drift
    }
  }

  // Integrate velocity to calculate position with added correction to reduce accumulation of drift
  for (int i = 0; i < 3; i++) {
    position[i] += velocity[i] * dt;
    if (abs(velocity[i]) < velocityThreshold) {
      position[i] *= 0.99; // Gradually reduce position accumulation when movement is small to counteract drift
    }
  }

  // Calculate pitch and roll with more accurate filtering and compensation
  double axFiltered = kalmanUpdate(kalmanX, acceleration[0]);
  double ayFiltered = kalmanUpdate(kalmanY, acceleration[1]);
  double azFiltered = kalmanUpdate(kalmanZ, acceleration[2] + 9.81); // Compensate for gravity

  pitch = atan2(-axFiltered, sqrt(ayFiltered * ayFiltered + azFiltered * azFiltered)) * 180.0 / M_PI;
  roll = atan2(ayFiltered, azFiltered) * 180.0 / M_PI;

  // Output the position, acceleration, pitch, and roll
  Serial.print("Acceleration X: ");
  Serial.print(acceleration[0]);
  Serial.print(" m/s^2, Y: ");
  Serial.print(acceleration[1]);
  Serial.print(" m/s^2, Z: ");
  Serial.print(acceleration[2]);
  Serial.println(" m/s^2");

  Serial.print("Position X: ");
  Serial.print(position[0]);
  Serial.print(" m, Y: ");
  Serial.print(position[1]);
  Serial.print(" m, Z: ");
  Serial.print(position[2]);
  Serial.println(" m");

  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print(" degrees, Roll: ");
  Serial.print(roll);
  Serial.println(" degrees");

  // Small delay to avoid spamming the serial monitor
  delay(100);
}
