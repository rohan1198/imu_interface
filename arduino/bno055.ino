#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long lastTime = 0;
const long interval = 10000; // Interval in microseconds for 100Hz

void setup() {
  Serial.begin(115200);
  if (!bno.begin()) {
    Serial.print("No BNO055 detected");
    while(1);
  }
  bno.setExtCrystalUse(true);
}

void loop() {
  unsigned long currentMicros = micros();

  // Check if the interval has passed
  if (currentMicros - lastTime >= interval) {
    lastTime = currentMicros;

    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    Serial.print("{\"timestamp\":");
    Serial.print(currentMicros);
    Serial.print(", \"orientation\":{\"x\":");
    Serial.print(quat.x(), 4);
    Serial.print(", \"y\":");
    Serial.print(quat.y(), 4);
    Serial.print(", \"z\":");
    Serial.print(quat.z(), 4);
    Serial.print(", \"w\":");
    Serial.print(quat.w(), 4);
    Serial.print("}, \"angular_velocity\":{\"x\":");
    Serial.print(gyro.x(), 4);
    Serial.print(", \"y\":");
    Serial.print(gyro.y(), 4);
    Serial.print(", \"z\":");
    Serial.print(gyro.z(), 4);
    Serial.print("}, \"linear_acceleration\":{\"x\":");
    Serial.print(accel.x(), 4);
    Serial.print(", \"y\":");
    Serial.print(accel.y(), 4);
    Serial.print(", \"z\":");
    Serial.print(accel.z(), 4);
    Serial.println("}}");
  }
}
