#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <WiFiUdp.h>
#include "esp32-hal-ledc.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "board_config.h"

// ESP32-CAM 穿戴端固件：接入 ESP32-S3 基站 SoftAP，
// 以 50Hz 推送 18 字节原始 IMU 帧，并支持基站下发的异步抓拍命令。
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

// 最近一次由UDP命令触发抓拍的JPEG照片
uint8_t *latestPhoto = nullptr;
size_t latestPhotoLen = 0;
SemaphoreHandle_t latestPhotoMutex = nullptr;

// JPEG获取和复制在独立任务执行；主循环继续读MPU、收发UDP。
SemaphoreHandle_t captureRequestSemaphore = nullptr;
SemaphoreHandle_t captureDoneSemaphore = nullptr;
TaskHandle_t captureTaskHandle = nullptr;
bool cameraReady = false;
bool commandPortReady = false;
bool captureBusy = false;
bool captureWaitingForLight = false;
unsigned long captureLightReadyTime = 0;
uint32_t captureSequence = 0;

// 非阻塞控制板载闪光灯的关闭时间
bool captureFlashActive = false;
unsigned long captureFlashOffTime = 0;

// 正式网络参数：ESP32-S3为192.168.4.1，穿戴端固定为192.168.4.2。
// 基站侧必须将192.168.4.2排除在DHCP地址池之外，避免分配给电脑。
IPAddress receiverIP(192, 168, 4, 1);
IPAddress localIP(192, 168, 4, 2);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// 团队约定端口
const uint16_t receiverPort = 8888;
const uint16_t localUdpPort = 8889;

uint16_t packetSequence = 0;
unsigned long lastSerialPrintTime = 0;

// 只在启动阶段输出并立即发送日志，定位阻塞的具体调用；不改变采样/协议。
void startupStep(const char *message) {
  Serial.printf("[STEP %lu ms] %s\n", (unsigned long)millis(), message);
  Serial.flush();
}

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
  startupStep("MPU: I2C0 begin ENTER (SDA=14, SCL=15)");
  bool busStarted = MPUWire.begin(14, 15, 100000);
  startupStep(busStarted ? "MPU: I2C0 begin RETURN true"
                         : "MPU: I2C0 begin RETURN false");
  if (!busStarted) {
    Serial.println("MPU I2C启动失败");
    return false;
  }

  delay(100);

  uint8_t who = 0;

  startupStep("MPU: WHO_AM_I read ENTER (addr=0x68, reg=0x75)");
  bool whoRead = readMpuBytes(0x75, &who, 1);
  startupStep(whoRead ? "MPU: WHO_AM_I read RETURN true"
                      : "MPU: WHO_AM_I read RETURN false");
  if (!whoRead) {
    Serial.println("无法读取MPU识别码");
    return false;
  }

  Serial.printf("MPU WHO_AM_I = 0x%02X\n", who);

  bool ok = true;

  startupStep("MPU: configuration writes ENTER");
  // 唤醒传感器
  ok = ok && writeMpuRegister(0x6B, 0x00);

  // 低通滤波约20Hz
  ok = ok && writeMpuRegister(0x1A, 0x04);

  // 陀螺仪量程：±2000度/秒
  ok = ok && writeMpuRegister(0x1B, 0x18);

  // 加速度量程：±16g
  ok = ok && writeMpuRegister(0x1C, 0x18);

  startupStep(ok ? "MPU: configuration writes RETURN true"
                 : "MPU: configuration writes RETURN false");
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

// 锁定最近一次抓拍照片，供 app_httpd.cpp 的 /pic 路由发送。
// 返回 true 时，调用方必须在发送结束后调用 unlockLatestPhoto()。
bool lockLatestPhoto(const uint8_t **data, size_t *length) {
  if (!data || !length || !latestPhotoMutex) {
    return false;
  }

  if (xSemaphoreTake(latestPhotoMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    return false;
  }

  if (!latestPhoto || latestPhotoLen == 0) {
    xSemaphoreGive(latestPhotoMutex);
    return false;
  }

  *data = latestPhoto;
  *length = latestPhotoLen;
  return true;
}

void unlockLatestPhoto() {
  if (latestPhotoMutex) {
    xSemaphoreGive(latestPhotoMutex);
  }
}

void updateCaptureFlash() {
#if defined(LED_GPIO_NUM)
  if (captureFlashActive &&
      (int32_t)(millis() - captureFlashOffTime) >= 0) {
    ledcWrite(LED_GPIO_NUM, 0);
    captureFlashActive = false;
  }
#endif
}

bool captureAndStorePhoto() {
  // 仅由captureWorker调用。开灯预热已由主循环的状态机完成。
  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("抓拍失败：摄像头没有返回图像");
    return false;
  }

  uint8_t *newPhoto = (uint8_t *)heap_caps_malloc(
    fb->len,
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );

  // PSRAM分配失败时尝试普通内存。
  if (!newPhoto) {
    newPhoto = (uint8_t *)malloc(fb->len);
  }

  if (!newPhoto) {
    Serial.println("抓拍失败：没有足够内存");
    esp_camera_fb_return(fb);
    return false;
  }

  memcpy(newPhoto, fb->buf, fb->len);
  size_t newPhotoLen = fb->len;
  esp_camera_fb_return(fb);

  if (!latestPhotoMutex ||
      xSemaphoreTake(latestPhotoMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    free(newPhoto);
    Serial.println("抓拍失败：照片缓冲区正忙");
    return false;
  }

  uint8_t *oldPhoto = latestPhoto;
  latestPhoto = newPhoto;
  latestPhotoLen = newPhotoLen;
  xSemaphoreGive(latestPhotoMutex);

  if (oldPhoto) {
    free(oldPhoto);
  }

  captureSequence++;
  Serial.printf("抓拍成功 #%lu：%u 字节，可打开 /pic 查看\n",
                (unsigned long)captureSequence,
                (unsigned int)newPhotoLen);

  return true;
}

void captureWorker(void *parameter) {
  (void)parameter;
  for (;;) {
    if (xSemaphoreTake(captureRequestSemaphore, portMAX_DELAY) == pdTRUE) {
      captureAndStorePhoto();
      xSemaphoreGive(captureDoneSemaphore);
    }
  }
}

void requestCapture() {
  // 以下状态和闪光灯只由主循环修改，工作任务只通过信号量通信。
  if (!cameraReady || !captureTaskHandle) {
    Serial.println("抓拍不可用：摄像头或工作任务未就绪");
    return;
  }
  if (captureBusy) {
    Serial.println("抓拍忙：本次命令未执行，请稍后重试");
    return;
  }

  captureBusy = true;
  captureWaitingForLight = true;
  captureLightReadyTime = millis();
#if defined(LED_GPIO_NUM)
  ledcWrite(LED_GPIO_NUM, 255);
  captureLightReadyTime += 150;
  captureFlashOffTime = millis() + 1000;
  captureFlashActive = true;
#endif
  Serial.println("抓拍已排队，MPU/UDP采集继续运行");
}

void serviceCapture() {
  updateCaptureFlash();

  if (captureBusy && captureWaitingForLight &&
      (int32_t)(millis() - captureLightReadyTime) >= 0) {
    captureWaitingForLight = false;
    if (xSemaphoreGive(captureRequestSemaphore) != pdTRUE) {
      captureBusy = false;
      Serial.println("抓拍失败：无法提交工作任务");
    }
  }

  if (captureDoneSemaphore &&
      xSemaphoreTake(captureDoneSemaphore, 0) == pdTRUE) {
    captureBusy = false;
  }
}

void checkCaptureCommand() {
  if (!commandPortReady) {
    return;
  }
  int packetSize = udp.parsePacket();
  uint8_t processed = 0;

  // 限制单轮处理数量，避免大量入站数据让MPU采样一直等不到执行。
  while (packetSize > 0 && processed < 4) {
    processed++;
    uint8_t command[4] = {0};
    int received = udp.read(command, sizeof(command));

    // 丢弃当前UDP报文中超出4字节的剩余内容。
    while (udp.available()) {
      udp.read();
    }

    bool validCommand =
        packetSize == 4 &&
        received == 4 &&
        command[0] == 0x55 &&
        command[1] == 0xAA &&
        command[2] == 0x01 &&
        command[3] == 0xFF;

    if (validCommand) {
      Serial.print("收到抓拍命令，来源：");
      Serial.print(udp.remoteIP());
      Serial.print(":");
      Serial.println(udp.remotePort());
      requestCapture();
    } else {
      Serial.printf("忽略未知UDP命令，长度=%d\n", packetSize);
    }

    packetSize = udp.parsePacket();
  }
}

// ===========================
// 主程序
// ===========================

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("[BOOT] wearable_firmware: static IP + async capture");
  Serial.flush();

  latestPhotoMutex = xSemaphoreCreateMutex();

  if (!latestPhotoMutex) {
    Serial.println("照片缓冲区初始化失败");
  }

  camera_config_t config = {};

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
  startupStep("CAMERA: esp_camera_init ENTER");
  esp_err_t err = esp_camera_init(&config);
  Serial.printf("[CAMERA] esp_camera_init RETURN 0x%x\n", err);
  Serial.flush();

  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    // TODO: 摄像头失败不应中断 WiFi/UDP 初始化（见 CHANGELOG 待处理项）；本次保持现有行为
    return;
  }
  cameraReady = true;

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
  startupStep("LED: setupLedFlash ENTER");
  setupLedFlash();
  startupStep("LED: setupLedFlash RETURN");
#endif

  // 低优先级抓拍任务运行在核0；Arduino主循环仍负责MPU的20ms采样。
  startupStep("CAPTURE: worker creation ENTER");
  captureRequestSemaphore = xSemaphoreCreateBinary();
  captureDoneSemaphore = xSemaphoreCreateBinary();
  if (latestPhotoMutex && captureRequestSemaphore && captureDoneSemaphore) {
    if (xTaskCreatePinnedToCore(captureWorker, "capture_worker", 6144,
                                nullptr, 1, &captureTaskHandle, 0) != pdPASS) {
      captureTaskHandle = nullptr;
      Serial.println("抓拍任务创建失败；MPU和视频仍可继续运行");
    } else {
      Serial.println("抓拍任务已启动（异步执行）");
    }
  } else {
    Serial.println("抓拍同步对象创建失败；抓拍功能不可用");
  }
  startupStep("CAPTURE: worker creation RETURN");

  // 启动 MPU
  startupStep("MPU: setupMpu ENTER");
  mpuReady = setupMpu();
  startupStep(mpuReady ? "MPU: setupMpu RETURN true"
                      : "MPU: setupMpu RETURN false");

  if (!mpuReady) {
    Serial.println("MPU没有连接，摄像头仍会继续运行");
  }

  // 连接真实基站：先启用STA，再配置固定IPv4地址，最后发起连接。
  // Wi-Fi信道由ESP32-S3 SoftAP固定为6，穿戴端跟随该基站接入。
  startupStep("WIFI: WiFi.mode ENTER");
  if (!WiFi.mode(WIFI_STA)) {
    startupStep("WIFI: WiFi.mode RETURN false; setup stopped");
    return;
  }
  startupStep("WIFI: WiFi.mode RETURN; static IP configuration ENTER");
  if (!WiFi.config(localIP, gateway, subnet)) {
    Serial.println("静态IP配置失败，停止联调初始化");
    Serial.flush();
    return;
  }
  startupStep("WIFI: static IP configuration RETURN; WiFi.begin ENTER");
  WiFi.begin(ssid, password);
  startupStep("WIFI: WiFi.begin RETURN; WiFi.setSleep ENTER");
  WiFi.setSleep(false);
  startupStep("WIFI: WiFi.setSleep RETURN; waiting for connection");

  Serial.print("WiFi connecting");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("穿戴端静态地址：");
  Serial.println(WiFi.localIP());
  Serial.print("基站网关：");
  Serial.println(WiFi.gatewayIP());
  Serial.print("子网掩码：");
  Serial.println(WiFi.subnetMask());
  commandPortReady = udp.begin(localUdpPort) != 0;
  if (commandPortReady) {
    Serial.printf("抓拍命令监听：UDP %u\n", localUdpPort);
  } else {
    Serial.println("UDP端口绑定失败，不能接收抓拍命令");
  }

  Serial.print("UDP发送目标：");
  Serial.print(receiverIP);
  Serial.print(":");
  Serial.println(receiverPort);

  startupStep("HTTP: startCameraServer ENTER");
  startCameraServer();
  startupStep("HTTP: startCameraServer RETURN");

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  startupStep("SETUP complete; entering main loop");
}

void loop() {
  // 非阻塞推进闪光预热、提交抓拍和关闭闪光灯。
  serviceCapture();

  // 监听真实基站发往本机8889端口的4字节抓拍命令。
  checkCaptureCommand();

  // 每20毫秒读取并发送一次MPU数据，即50Hz
  if (mpuReady && millis() - lastMpuReadTime >= 20) {
    lastMpuReadTime = millis();
    printMpuData();
  }

  delay(1);
}
