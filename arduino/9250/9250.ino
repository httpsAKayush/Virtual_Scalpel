#include <Wire.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

void setup() {
  Serial.begin(115200);
  delay(2000); // Let sensor stabilize

  Wire.begin(21, 22); // ESP32 WROOM I2C pins: SDA, SCL
  mySensor.setWire(&Wire);

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag(); // Magnetometer must be initialized

  Serial.println("MPU9250 Initialized!");
}

void loop() {
  mySensor.accelUpdate();
  mySensor.gyroUpdate();
  uint8_t magStatus = mySensor.magUpdate();

  Serial.println("------");

  // Accelerometer
  Serial.print("Accel X: "); Serial.print(mySensor.accelX(), 3);
  Serial.print(" Y: "); Serial.print(mySensor.accelY(), 3);
  Serial.print(" Z: "); Serial.println(mySensor.accelZ(), 3);

  // Gyroscope
  Serial.print("Gyro  X: "); Serial.print(mySensor.gyroX(), 3);
  Serial.print(" Y: "); Serial.print(mySensor.gyroY(), 3);
  Serial.print(" Z: "); Serial.println(mySensor.gyroZ(), 3);

  // Magnetometer
  if (magStatus == 0) {
    Serial.print("Mag   X: "); Serial.print(mySensor.magX(), 3);
    Serial.print(" Y: "); Serial.print(mySensor.magY(), 3);
    Serial.print(" Z: "); Serial.println(mySensor.magZ(), 3);
  } else {
    Serial.println("Magnetometer update failed!");
  }

  delay(500);
}
