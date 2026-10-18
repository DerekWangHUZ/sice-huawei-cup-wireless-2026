#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readBytes(uint8_t reg, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t received =
      Wire.requestFrom(MPU_ADDR, (size_t)length, true);

  if (received != length) {
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    data[i] = Wire.read();
  }

  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(14, 15);  // SDA=IO14，SCL=IO15
  Wire.setClock(100000);

  uint8_t who = 0;

  if (!readBytes(0x75, &who, 1)) {
    Serial.println("无法读取传感器识别码");
    while (true) delay(100);
  }

  Serial.printf("WHO_AM_I = 0x%02X\n", who);

  // 唤醒传感器
  writeRegister(0x6B, 0x00);

  // 低通滤波约20Hz
  writeRegister(0x1A, 0x04);

  // 陀螺仪量程：±2000度/秒
  writeRegister(0x1B, 0x18);

  // 加速度量程：±16g
  writeRegister(0x1C, 0x18);

  delay(100);

  Serial.println("开始读取运动数据");
}

void loop() {
  uint8_t data[14];

  if (!readBytes(0x3B, data, 14)) {
    Serial.println("读取数据失败");
    delay(500);
    return;
  }

  int16_t axRaw = (int16_t)((data[0] << 8) | data[1]);
  int16_t ayRaw = (int16_t)((data[2] << 8) | data[3]);
  int16_t azRaw = (int16_t)((data[4] << 8) | data[5]);

  int16_t gxRaw = (int16_t)((data[8] << 8) | data[9]);
  int16_t gyRaw = (int16_t)((data[10] << 8) | data[11]);
  int16_t gzRaw = (int16_t)((data[12] << 8) | data[13]);

  float ax = axRaw / 2048.0;
  float ay = ayRaw / 2048.0;
  float az = azRaw / 2048.0;

  float gx = gxRaw / 16.4;
  float gy = gyRaw / 16.4;
  float gz = gzRaw / 16.4;

  Serial.printf(
    "ax=%.2fg  ay=%.2fg  az=%.2fg  "
    "gx=%.2f  gy=%.2f  gz=%.2f dps\n",
    ax, ay, az, gx, gy, gz
  );

  delay(100);
}