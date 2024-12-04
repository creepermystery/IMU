#include <Wire.h>
#include <math.h>
<<<<<<< HEAD

#define MPU_ADDR 0x68 // Adresse I2C par défaut du MPU-6050
#define GRAVITY 9.81 // Accélération due à la gravité
#define ALPHA 0.9 // Coefficient de filtre pour réduire le bruit
#define VELOCITY_DAMPING 0.98 // Coefficient pour amortir la vitesse pendant le mouvement
#define STATIC_THRESHOLD 0.05 // Seuil pour déterminer l'arrêt

unsigned long t_prev = 0;
float positionX = 0, positionY = 0, positionZ = 0;
float vx = 0, vy = 0, vz = 0;
float ax_bias = 0, ay_bias = 0, az_bias = GRAVITY;
float prev_ax = 0, prev_ay = 0, prev_az = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Accéder au registre PWR_MGMT_1
  Wire.write(0);    // Réveiller le MPU-6050
  if (Wire.endTransmission() != 0) {
    Serial.println("Erreur d'initialisation de l'IMU. Vérifiez les connexions.");
    while (1) {}
  }

  delay(100); // Attendre un peu pour s'assurer que l'IMU est bien réveillé
  Serial.println("IMU initialisée avec succès");

  // Calibration de l'accéléromètre pour compenser les biais
  calibrateAccelerometer();

  t_prev = millis(); // Initialiser t_prev après l'initialisation réussie
}

void loop() {
  unsigned long t_now = millis();
  float dt = (t_now - t_prev) / 1000.0; // Calculer le delta temps en secondes
  t_prev = t_now;

  // Obtenir les lectures de l'accéléromètre
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Adresse du premier registre d'accélération
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); // Demander 6 octets pour les trois axes d'accélération

  if (Wire.available() < 6) {
    Serial.println("Erreur: données d'accélération non disponibles");
    delay(500);
    return;
  }

  int16_t ax_raw = Wire.read() << 8 | Wire.read();
  int16_t ay_raw = Wire.read() << 8 | Wire.read();
  int16_t az_raw = Wire.read() << 8 | Wire.read();

  // Convertir les valeurs brutes en m/s²
  float ax = (ax_raw / 16384.0) * GRAVITY - ax_bias;
  float ay = (ay_raw / 16384.0) * GRAVITY - ay_bias;
  float az = (az_raw / 16384.0) * GRAVITY - az_bias;

  // Appliquer un filtre pour réduire le bruit
  static float ax_filtered = 0, ay_filtered = 0, az_filtered = 0;
  ax_filtered = ALPHA * ax_filtered + (1 - ALPHA) * ax;
  ay_filtered = ALPHA * ay_filtered + (1 - ALPHA) * ay;
  az_filtered = ALPHA * az_filtered + (1 - ALPHA) * az;

  // Détecter si l'IMU est à l'arrêt (accélérations proches de zéro)
  bool is_static = fabs(ax_filtered) < STATIC_THRESHOLD && fabs(ay_filtered) < STATIC_THRESHOLD && fabs(az_filtered) < STATIC_THRESHOLD;

  if (is_static) {
    // Amortir la vitesse si l'IMU est à l'arrêt
    vx *= VELOCITY_DAMPING;
    vy *= VELOCITY_DAMPING;
    vz *= VELOCITY_DAMPING;
  } else {
    // Mise à jour des vitesses en intégrant l'accélération filtrée (v = v0 + a * dt)
    vx += ax_filtered * dt;
    vy += ay_filtered * dt;
    vz += az_filtered * dt;
  }

  // Mise à jour des positions en intégrant la vitesse (x = x0 + v * dt)
  positionX += vx * dt;
  positionY += vy * dt;
  positionZ += vz * dt;

  // Affichage des données
  Serial.print("Acceleration (X, Y, Z): ");
  Serial.print(ax_filtered); Serial.print(", ");
  Serial.print(ay_filtered); Serial.print(", ");
  Serial.print(az_filtered); Serial.println(" m/s²");

  Serial.print("Position (X, Y, Z): ");
  Serial.print(positionX); Serial.print(", ");
  Serial.print(positionY); Serial.print(", ");
  Serial.print(positionZ); Serial.println(" m");

  Serial.println();
  delay(500);  // Augmenter le délai pour laisser plus de temps à la lecture des capteurs
}

void calibrateAccelerometer() {
  const int num_samples = 100;
  long ax_sum = 0, ay_sum = 0, az_sum = 0;

  Serial.println("Calibration de l'accéléromètre en cours...");
  for (int i = 0; i < num_samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); // Adresse du premier registre d'accélération
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    if (Wire.available() >= 6) {
      int16_t ax_raw = Wire.read() << 8 | Wire.read();
      int16_t ay_raw = Wire.read() << 8 | Wire.read();
      int16_t az_raw = Wire.read() << 8 | Wire.read();

      ax_sum += ax_raw;
      ay_sum += ay_raw;
      az_sum += az_raw;
    }
    delay(10);
  }

  ax_bias = (ax_sum / num_samples) / 16384.0 * GRAVITY;
  ay_bias = (ay_sum / num_samples) / 16384.0 * GRAVITY;
  az_bias = (az_sum / num_samples) / 16384.0 * GRAVITY;

  Serial.println("Calibration terminée.");
=======
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
>>>>>>> 6e2df7ca282d7663e059f4f176941f00ee31ce3b
}
