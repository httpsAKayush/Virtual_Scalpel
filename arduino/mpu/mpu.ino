#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>  // Required for atan2() and M_PI

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("Adafruit MPU6050 Angle Test!");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  Serial.println("");
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate Roll and Pitch (in radians) from accelerometer
  float roll_rad = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z));
  float pitch_rad = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z));

  // Convert to degrees
  float roll_deg = roll_rad * 180.0 / M_PI;        //x axis
  float pitch_deg = pitch_rad * 180.0 / M_PI;      //y axis


  //only their is 6 degree of freedom 
  //their is no magnetometer reading in mpu6050 

  // Print Accelerometer and Angle Data
  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s², ");
  Serial.print("Y: "); Serial.print(a.acceleration.y); Serial.print(" m/s², ");
  Serial.print("Z: "); Serial.print(a.acceleration.z); Serial.println(" m/s²");

  Serial.print("Roll: "); Serial.print(roll_deg); Serial.print("° | ");
  Serial.print("Pitch: "); Serial.print(pitch_deg); Serial.println("°");

  Serial.print("Gyro X: "); Serial.print(g.gyro.x * 57.2958); Serial.print("°/s, ");
  Serial.print("Y: "); Serial.print(g.gyro.y * 57.2958); Serial.print("°/s, ");
  Serial.print("Z: "); Serial.print(g.gyro.z * 57.2958); Serial.println("°/s");

  Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" °C");
  Serial.println("---------------------");
  delay(500);
}