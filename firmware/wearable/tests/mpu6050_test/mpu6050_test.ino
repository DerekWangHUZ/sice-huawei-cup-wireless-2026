#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 设置I2C引脚：SDA=IO14，SCL=IO15
  Wire.begin(14, 15);
  Wire.setClock(100000);

  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("没有找到MPU6050，请检查接线");
    while (true) {
      delay(100);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050连接成功");
}

void loop() {
  sensors_event_t acceleration;
  sensors_event_t rotation;
  sensors_event_t temperature;

  mpu.getEvent(&acceleration, &rotation, &temperature);

  Serial.printf(
    "ax=%.2f  ay=%.2f  az=%.2f  gx=%.2f  gy=%.2f  gz=%.2f\n",
    acceleration.acceleration.x,
    acceleration.acceleration.y,
    acceleration.acceleration.z,
    rotation.gyro.x,
    rotation.gyro.y,
    rotation.gyro.z
  );

  delay(100);
}