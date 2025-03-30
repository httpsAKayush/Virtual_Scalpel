#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MPU6050 mpu;

// Angle variables
float roll = 0, pitch = 0, yaw = 0;
float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
unsigned long last_time = 0;

float normalizeAngle(float angle) {
  angle = fmod(angle + 180, 360);
  if (angle < 0) angle += 360;
  return angle - 180;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(4, 5);  // SDA -> D2 (GPIO4), SCL -> D1 (GPIO5)

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  // Gyro calibration
  Serial.println("Calibrating gyro...");
  const int num_samples = 500;
  float gx = 0, gy = 0, gz = 0;
  
  for(int i = 0; i < num_samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gx += g.gyro.x;
    gy += g.gyro.y;
    gz += g.gyro.z;
    delay(1);
  }
  gyro_bias_x = gx / num_samples;
  gyro_bias_y = gy / num_samples;
  gyro_bias_z = gz / num_samples;

  // Initial angles from accelerometer
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;

  roll = atan2(accelY, accelZ) * RAD_TO_DEG;
  pitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * RAD_TO_DEG;
  
  last_time = millis();
}



void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Convert gyro to deg/s and remove bias
  float gyro_x = (g.gyro.x - gyro_bias_x) * RAD_TO_DEG;
  float gyro_y = (g.gyro.y - gyro_bias_y) * RAD_TO_DEG;
  float gyro_z = (g.gyro.z - gyro_bias_z) * RAD_TO_DEG;

  // Calculate time difference
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
  last_time = current_time;

  // Calculate accelerometer angles
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;

  float roll_acc = atan2(accelY, accelZ) * RAD_TO_DEG;
  float pitch_acc = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * RAD_TO_DEG;

  // Complementary filter
  const float alpha = 0.98;
  roll = alpha * (roll + gyro_x * dt) + (1 - alpha) * roll_acc;
  pitch = alpha * (pitch + gyro_y * dt) + (1 - alpha) * pitch_acc;
  yaw += gyro_z * dt;

  // Normalize angles
  roll = normalizeAngle(roll);
  pitch = normalizeAngle(pitch);
  yaw = normalizeAngle(yaw);

  // Serial output
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print("°, Pitch: ");
  Serial.print(pitch);
  Serial.print("°, Yaw: ");
  Serial.print(yaw);
  Serial.println("°");

  delay(10);
}