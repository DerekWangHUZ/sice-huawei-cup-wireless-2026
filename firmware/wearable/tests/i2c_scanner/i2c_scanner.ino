#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(14, 15);  // SDA=IO14，SCL=IO15
  Wire.setClock(100000);

  Serial.println("开始扫描I2C设备...");
}

void loop() {
  int count = 0;

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("找到设备，地址：0x%02X\n", address);
      count++;
    }
  }

  if (count == 0) {
    Serial.println("没有扫描到任何I2C设备");
  }

  delay(2000);
}