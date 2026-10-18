#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <WiFiUdp.h>

#include "board_config.h"

// 正式联调热点信息（由固定基站 ESP32-S3 提供）
const char *ssid = "CSI_FALL_AP";
const char *password = "12345678";

void startCameraServer();
void setupLedFlash();

// ===========================
// MPU 配置
// ===========================

const uint8_t MPU_ADDR = 0x68;

// 使用 I2C 控制器 0；实测控制器 1 会与摄像头占用的总线冲突
TwoWire MPUWire = TwoWire(0);

bool mpuReady = false;
unsigned long lastMpuReadTime = 0;
// 跌倒检测状态
bool fallCandidate = false;
unsigned long fallCandidateTime = 0;
unsigned long stillStartTime = 0;
unsigned long lastFallAlarmTime = 0;
// UDP通信参数
WiFiUDP udp;

// 固定基站地址
IPAddress receiverIP(192, 168, 4, 1);

// 穿戴端正式静态网络参数
IPAddress localIP(192, 168, 4, 2);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// 团队约定端口
const uint16_t receiverPort = 8888;
const uint16_t localUdpPort = 8889;

uint16_t packetSequence = 0;
unsigned long lastSerialPrintTime = 0;

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

// 将int16_t按小端序放入数据包
void putInt16LE(uint8_t *packet, int offset, int16_t value) {
  packet[offset] = (uint8_t)(value & 0xFF);
  packet[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
}

// 发送团队约定的18字节MPU数据包
void sendImuPacket(
  int16_t axRaw,
  int16_t ayRaw,
  int16_t azRaw,
  int16_t gxRaw,
  int16_t gyRaw,
  int16_t gzRaw
) {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  uint8_t packet[18];

  // Byte 0~1：帧头
  packet[0] = 0xAA;
  packet[1] = 0x55;

  // Byte 2~3：包序号，小端序
  packet[2] = (uint8_t)(packetSequence & 0xFF);
  packet[3] = (uint8_t)((packetSequence >> 8) & 0xFF);

  // Byte 4~15：MPU原始数据
  putInt16LE(packet, 4, axRaw);
  putInt16LE(packet, 6, ayRaw);
  putInt16LE(packet, 8, azRaw);
  putInt16LE(packet, 10, gxRaw);
  putInt16LE(packet, 12, gyRaw);
  putInt16LE(packet, 14, gzRaw);

  // Byte 16：Byte 2到Byte 15的异或校验
  uint8_t checksum = 0;

  for (int i = 2; i <= 15; i++) {
    checksum ^= packet[i];
  }

  packet[16] = checksum;

  // Byte 17：帧尾
  packet[17] = 0x0D;

  udp.beginPacket(receiverIP, receiverPort);
  udp.write(packet, sizeof(packet));
  udp.endPacket();

  packetSequence++;
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

void checkFall(
  float ax, float ay, float az,
  float gx, float gy, float gz
) {
  float accelerationMagnitude =
      sqrtf(ax * ax + ay * ay + az * az);

  float gyroMagnitude =
      sqrtf(gx * gx + gy * gy + gz * gz);

  unsigned long now = millis();

  // 第一阶段：检测剧烈冲击或者快速旋转
  bool strongMotion =
      accelerationMagnitude > 2.2 ||
      gyroMagnitude > 180.0;

  if (!fallCandidate) {
    bool cooldownFinished =
        lastFallAlarmTime == 0 ||
        now - lastFallAlarmTime > 3000;

    if (strongMotion && cooldownFinished) {
      fallCandidate = true;
      fallCandidateTime = now;
      stillStartTime = 0;

      Serial.printf(
        "\n检测到剧烈运动，等待静止确认..."
        " A=%.2fg G=%.1fdps\n",
        accelerationMagnitude,
        gyroMagnitude
      );
    }

    return;
  }

  // 超过3秒仍未满足条件，取消本次检测
  if (now - fallCandidateTime > 3000) {
    fallCandidate = false;
    stillStartTime = 0;
    Serial.println("未满足跌倒条件，取消报警");
    return;
  }

  // 第二阶段：判断剧烈运动后是否静止
  bool isStill =
      accelerationMagnitude > 0.75 &&
      accelerationMagnitude < 1.25 &&
      gyroMagnitude < 25.0;

  if (isStill) {
    if (stillStartTime == 0) {
      stillStartTime = now;
    }

    // 连续静止0.8秒，确认疑似跌倒
    if (now - stillStartTime >= 800) {
      Serial.println();
      Serial.println("************************");
      Serial.println("***   疑似跌倒！     ***");
      Serial.println("************************");
      Serial.println();

      fallCandidate = false;
      stillStartTime = 0;
      lastFallAlarmTime = now;
    }
  } else {
    // 中途又发生运动，重新计算静止时间
    stillStartTime = 0;
  }
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

  // 每次读取后立即发送18字节UDP数据包
  sendImuPacket(
    axRaw, ayRaw, azRaw,
    gxRaw, gyRaw, gzRaw
  );

  float ax = axRaw / 2048.0;
  float ay = ayRaw / 2048.0;
  float az = azRaw / 2048.0;

  float gx = gxRaw / 16.4;
  float gy = gyRaw / 16.4;
  float gz = gzRaw / 16.4;

  // 保留第五关的本地跌倒检测
  checkFall(ax, ay, az, gx, gy, gz);

  // 串口只每200毫秒显示一次，避免影响摄像头
  if (millis() - lastSerialPrintTime >= 200) {
    lastSerialPrintTime = millis();

    Serial.printf(
      "UDP #%u  ax=%.2fg ay=%.2fg az=%.2fg "
      "gx=%.2f gy=%.2f gz=%.2f dps\n",
      packetSequence,
      ax, ay, az,
      gx, gy, gz
    );
  }
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

  // 连接固定基站热点
  if (!WiFi.config(localIP, gateway, subnet)) {
    Serial.println("静态IP配置失败");
  }

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  udp.begin(localUdpPort);

  Serial.print("UDP发送目标：");
  Serial.print(receiverIP);
  Serial.print(":");
  Serial.println(receiverPort);

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // 每20毫秒读取并发送一次MPU数据，即50Hz
  if (mpuReady && millis() - lastMpuReadTime >= 20) {
    lastMpuReadTime = millis();
    printMpuData();
  }

  delay(1);
}
