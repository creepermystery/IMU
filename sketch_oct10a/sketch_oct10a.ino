#include <Wire.h>
#include <TimerOne.h>

#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS 0x00 
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2_G 0x00 
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

// Variables for storing offset values
int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;

// Variables for storing velocity and position
float vx = 0, vy = 0, vz = 0; // Velocity in m/s
float x = 0, y = 0, z = 0;     // Position in meters

// Function to read Nbytes bytes from I2C device
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.endTransmission();
    
    Wire.requestFrom(Address, Nbytes); 
    uint8_t index = 0;
    while (Wire.available())
        Data[index++] = Wire.read();
}

// Function to write a byte to I2C device
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
    Wire.beginTransmission(Address);
    Wire.write(Register);
    Wire.write(Data);
    Wire.endTransmission();
}

// Function to calibrate the accelerometer
void calibrate_accelerometer() {
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int samples = 1000;  // Number of samples for averaging

    for (int i = 0; i < samples; i++) {
        uint8_t Buf[6];
        I2Cread(MPU9250_ADDRESS, 0x3B, 6, Buf);
        
        int16_t ax = -(Buf[0] << 8 | Buf[1]);
        int16_t ay = -(Buf[2] << 8 | Buf[3]);
        int16_t az = Buf[4] << 8 | Buf[5];

        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;

        delay(3);  // Small delay to gather samples
    }

    // Compute offsets
    ax_offset = ax_sum / samples;
    ay_offset = ay_sum / samples;
    az_offset = az_sum / samples - 4096; // Compensate for 1G on Z-axis
}

// Initial time
long int ti;
volatile bool intFlag = false;

// Initializations
void setup() {
    // Arduino initializations
    Wire.begin();
    Serial.begin(115200);

    // Set accelerometer and gyroscope low pass filters at 5Hz
    I2CwriteByte(MPU9250_ADDRESS, 29, 0x06);  // Accelerometer
    I2CwriteByte(MPU9250_ADDRESS, 26, 0x06);  // Gyroscope
    
    // Configure gyroscope and accelerometer ranges
    I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);  // Gyroscope range
    I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_4_G);        // Accelerometer range
    
    // Set by pass mode for the magnetometers
    I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);

    pinMode(13, OUTPUT);
    Timer1.initialize(10000);  // Initialize timer1
    Timer1.attachInterrupt(callback);  // Attach interrupt

    // Perform accelerometer calibration
    calibrate_accelerometer();
    
    // Store initial time
    ti = millis();
}

// Counter
long int cpt = 0;

void callback() {
    intFlag = true;
    digitalWrite(13, digitalRead(13) ^ 1);
}

// Main loop, read and display data
void loop() {
    while (!intFlag);
    intFlag = false;

    // Read accelerometer and gyroscope data
    uint8_t Buf[14];
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

    // Create 16-bit values from 8-bit data
    // Apply offset correction after calibration

    // Accelerometer
    int16_t ax = -(Buf[0] << 8 | Buf[1]) - ax_offset; // Corrected accelerometer X
    int16_t ay = -(Buf[2] << 8 | Buf[3]) - ay_offset; // Corrected accelerometer Y
    int16_t az = (Buf[4] << 8 | Buf[5]) - az_offset;  // Corrected accelerometer Z

    // Gyroscope
    int16_t gx = -(Buf[8] << 8 | Buf[9]); // Corrected gyroscope X
    int16_t gy = -(Buf[10] << 8 | Buf[11]); // Corrected gyroscope Y
    int16_t gz = Buf[12] << 8 | Buf[13]; // Corrected gyroscope Z

    // Convert raw acceleration to m/s²
    float ax_m_s2 = ax * (4.0 / 32768.0) * 9.81; // Adjust factor based on ±4G setting
    float ay_m_s2 = ay * (4.0 / 32768.0) * 9.81;
    float az_m_s2 = az * (4.0 / 32768.0) * 9.81;

    // Remove gravity component from Z-axis
    az_m_s2 -= 9.81;  // Remove gravity from Z-axis

    // Calculate time difference
    long int currentTime = millis();
    float dt = (currentTime - ti) / 1000.0; // Convert ms to seconds
    ti = currentTime; // Update previous time

    // Update velocities by integrating acceleration
    vx += ax_m_s2 * dt;
    vy += ay_m_s2 * dt;
    vz += az_m_s2 * dt;

    // Update positions by integrating velocity
    x += vx * dt * 1000; // Convert from m to mm
    y += vy * dt * 1000; // Convert from m to mm
    z += vz * dt * 1000; // Convert from m to mm

    // Display accelerometer values
    Serial.print(ax, DEC);
    Serial.print("\t");
    Serial.print(ay, DEC);
    Serial.print("\t");
    Serial.print(az, DEC);
    Serial.print("\t");

    // Display gyroscope values
    Serial.print(gx, DEC);
    Serial.print("\t");
    Serial.print(gy, DEC);
    Serial.print("\t");
    Serial.print(gz, DEC);
    Serial.print("\t");

    // Display coordinates in mm
    Serial.print("\tX: ");
    Serial.print(x);
    Serial.print("\tY: ");
    Serial.print(y);
    Serial.print("\tZ: ");
    Serial.print(z);
    Serial.println();

    // End of line
    Serial.println("");
}
