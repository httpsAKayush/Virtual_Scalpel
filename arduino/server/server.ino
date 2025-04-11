#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Adafruit_MPU6050 mpu;

float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

// Variables for orientation angles
float pitch = 0, roll = 0, yaw = 0;
unsigned long lastTime = 0;  // Time in microseconds

#define FLEX_PIN1 15  // Use a GPIO with ADC capability
#define FLEX_PIN2 32  // Use a GPIO with ADC capability
#define FLEX_PIN3 33  // Use a GPIO with ADC capability
#define FLEX_PIN4 34  // Use a GPIO with ADC capability
// #define FLEX_PIN 33 // Use a GPIO with ADC capability

const char* ssid = "Free";
const char* password = "hehehaha";
const char* serverIP = "192.168.213.39";  // Unity Server IP
const int serverPort = 8080;


WiFiUDP udp;

void calibrateGyro() {
  const int samples = 100;
  float sumX = 0, sumY = 0, sumZ = 0;
  sensors_event_t a, g, temp;

  Serial.println("Calibrating gyro... Make sure the sensor is stationary.");
  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(10);
  }
  gyroOffsetX = sumX / samples;
  gyroOffsetY = sumY / samples;
  gyroOffsetZ = sumZ / samples;
  Serial.println("Gyro calibration complete!");
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  analogReadResolution(12);  // ESP32 supports 12-bit resolution (0-4095)


  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
    Serial.println("MPU6050 Found!");

    // Set sensor ranges and filter bandwidth as desired.
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    // Optional: Print current sensor settings (using your existing switches)
    // ...

    delay(100);  // Wait for sensor to settle

    // Calibrate the gyroscope
    calibrateGyro();

    // Initialize lastTime to the current time in microseconds.
    lastTime = micros();
  }

  Serial.println("\nConnected to WiFi");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Determine time step (in seconds)
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;

  // Subtract offsets from gyro readings
  float gx = g.gyro.x - gyroOffsetX;
  float gy = g.gyro.y - gyroOffsetY;
  float gz = g.gyro.z - gyroOffsetZ;

  float accRoll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float accPitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  float alpha = 0.98;

  // Integrate gyro data and combine with accelerometer data
  roll = alpha * (roll + gx * dt) + (1 - alpha) * accRoll;
  pitch = alpha * (pitch + gy * dt) + (1 - alpha) * accPitch;

  // Yaw is integrated from the gyro only (this value will drift over time)
  yaw += gz * dt;

  // Print the computed values
  Serial.print("Pitch: ");
  Serial.print(pitch);
  Serial.print("  Roll: ");
  Serial.print(roll);
  Serial.print("  Yaw: ");
  Serial.println(yaw);



  float flex1 = analogRead(FLEX_PIN1);
  float flex2 = analogRead(FLEX_PIN2);
  float flex3 = analogRead(FLEX_PIN3);
  float flex4 = analogRead(FLEX_PIN4);
  float sensorData[8] = { 1.0, flex1, flex2, flex3, flex4, pitch, roll, yaw };  // Example float array
  int dataSize = sizeof(sensorData);

  udp.beginPacket(serverIP, serverPort);
  udp.write((uint8_t*)sensorData, dataSize);  // Send raw bytes
  udp.endPacket();

  Serial.println("Data Sent!");
  delay(3000);  // Send every second
}