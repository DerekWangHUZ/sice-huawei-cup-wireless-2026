#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>

#include "board_config.h"

// 手机热点信息
const char *ssid = "CSI_FALL_AP";
const char *password = "12345678";

void startCameraServer();
void setupLedFlash();

// ===========================
// MPU 配置
// ===========================

const uint8_t MPU_ADDR = 0x68;

// 使用第二组 I2C，避免和摄像头控制总线冲突
TwoWire MPUWire = TwoWire(0);

bool mpuReady = false;
unsigned long lastMpuReadTime = 0;

bool writeMpuRegister(uint8_t reg, uint8_t value) {
  MPUWire.beginTransmission(MPU_ADDR);
  MPUWire.write(reg);
  MPUWire.write(value);
  return MPUWire.endTransmission() == 0;
}

bool readMpuBytes(uint8_t reg, uint8_t *data, uint8_t length) {
  MPUWire.beginTransmission(MPU_ADDR);
  MPUWire.write(reg);

  if (MPUWire.endTransmission(false) != 0) {
    return false;
  }

  size_t received =
      MPUWire.requestFrom(MPU_ADDR, (size_t)length, true);

  if (received != length) {
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    data[i] = MPUWire.read();
  }

  return true;
}

bool setupMpu() {
  // SDA=IO14，SCL=IO15，速度100kHz
  if (!MPUWire.begin(14, 15, 100000)) {
    Serial.println("MPU I2C启动失败");
    return false;
  }

  delay(100);

  uint8_t who = 0;

  if (!readMpuBytes(0x75, &who, 1)) {
    Serial.println("无法读取MPU识别码");
    return false;
  }

  Serial.printf("MPU WHO_AM_I = 0x%02X\n", who);

  bool ok = true;

  // 唤醒传感器
  ok = ok && writeMpuRegister(0x6B, 0x00);

  // 低通滤波约20Hz
  ok = ok && writeMpuRegister(0x1A, 0x04);

  // 陀螺仪量程：±2000度/秒
  ok = ok && writeMpuRegister(0x1B, 0x18);

  // 加速度量程：±16g
  ok = ok && writeMpuRegister(0x1C, 0x18);

  if (!ok) {
    Serial.println("MPU寄存器设置失败");
    return false;
  }

  Serial.println("MPU启动成功");
  return true;
}

void printMpuData() {
  uint8_t data[14];

  if (!readMpuBytes(0x3B, data, 14)) {
    Serial.println("MPU数据读取失败");
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
}

// ===========================
// 主程序
// ===========================

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;

  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;

#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // 启动摄像头
  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();

  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }

  // 默认使用低分辨率，减轻卡顿
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  // 启动 MPU
  mpuReady = setupMpu();

  if (!mpuReady) {
    Serial.println("MPU没有连接，摄像头仍会继续运行");
  }

  // 连接手机热点
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // 每100毫秒读取一次MPU，不阻塞摄像头
  if (mpuReady && millis() - lastMpuReadTime >= 100) {
    lastMpuReadTime = millis();
    printMpuData();
  }

  delay(1);
}
