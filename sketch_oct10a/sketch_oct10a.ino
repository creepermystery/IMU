#include <Wire.h>
#include <math.h>

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
}
