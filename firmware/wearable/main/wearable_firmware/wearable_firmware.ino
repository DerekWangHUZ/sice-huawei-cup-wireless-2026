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

/*
 * 摄像头启动开关（对应 README §4/§7/§11）：
 *   1（默认）= 上电初始化摄像头（三次自检）+ 闪光灯 + 网页服务，全功能；
 *   0 = 跳过摄像头相关初始化，仅保留 MPU 50Hz 推流与 8889 抓拍命令
 *       接收（校验+日志，不取图），用于隔离摄像头链路问题或最小化运行。
 */
#define CAMERA_BOOT_ENABLE 1

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

/*
 * 抓拍链路诊断计数（供 /status 查询，不依赖串口）：
 *   cmdRx    收到并通过校验的抓拍命令数
 *   rejected requestCapture 因未就绪/忙而拒绝的次数
 *   ok       成功存入 PSRAM 的照片数
 *   fail     取帧或分配内存失败的次数
 */
volatile uint32_t g_captureCmdRx  = 0;
volatile uint32_t g_captureReject = 0;
volatile uint32_t g_captureOk     = 0;
volatile uint32_t g_captureFail   = 0;

/*
 * 姿态帧 UDP 发送失败计数（write/endPacket 失败或 beginPacket 拒绝）。
 * 失败不递增 packetSequence（序号严格等于成功发出的帧数），
 * 只累计在此，随 200ms 诊断行汇报，不逐帧刷屏。
 */
volatile uint32_t g_udpSendFail = 0;
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

  /*
   * UDP 无连接，丢一帧由基站掉线保护兜底，不必重发。
   * 但 beginPacket 在 socket 未就绪时返回 0，此时继续 write/endPacket
   * 无意义且会掩盖故障，因此判一下。
   * 三步全部成功才递增 packetSequence：序号严格等于"成功发出的帧数"，
   * 失败只累计 g_udpSendFail（随 200ms 诊断行汇报），不逐帧刷屏。
   */
  bool sentOk = false;
  if (udp.beginPacket(receiverIP, receiverPort) != 0) {
    size_t written = udp.write(packet, sizeof(packet));
    if (written == sizeof(packet) && udp.endPacket() != 0) {
      sentOk = true;
    }
  }

  if (sentOk) {
    packetSequence++;
  } else {
    g_udpSendFail++;
  }
}

bool setupMpu() {
  // SDA=IO14，SCL=IO15，速度100kHz
  startupStep("MPU: I2C0 begin ENTER (SDA=14, SCL=15)");
  bool busStarted = MPUWire.begin(14, 15, 100000);
  /*
   * I2C 超时保护：MPU 掉线或杜邦线松动时，Wire 会在 SDA 被拉低时长时间
   * 等待，把主循环的 50Hz 采样拖死（表现为数据停发、串口无输出）。
   * 设 50ms 上限，让读写快速失败返回 false，上层跳过本次采样。
   */
  MPUWire.setTimeOut(50);
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

  // 第一阶段：剧烈冲击（与基站一致：仅合加速度判据）
  bool strongMotion = accelerationMagnitude > 3.0;

  if (!fallCandidate) {
    bool cooldownFinished =
        lastFallAlarmTime == 0 ||
        now - lastFallAlarmTime > 3000;

    if (strongMotion && cooldownFinished) {
      fallCandidate = true;
      fallCandidateTime = now;
      stillStartTime = 0;

      Serial.printf(
        "\n[本地] 检测到冲击（基站口径），等待静止确认..."
        " A=%.2fg G=%.1fdps\n",
        accelerationMagnitude,
        gyroMagnitude
      );
    }

    return;
  }

  // 超过观察上限（PENDING_MAX_MS）仍未满足条件，取消本次检测
  if (now - fallCandidateTime > 4000) {
    fallCandidate = false;
    stillStartTime = 0;
    Serial.println("[本地] 观察期内未确认，取消（与基站口径一致）");
    return;
  }

  // 第二阶段：判断剧烈运动后是否持续静止（静止带与基站一致）
  bool isStill =
      accelerationMagnitude > 0.65 &&
      accelerationMagnitude < 1.35 &&
      gyroMagnitude < 50.0;

  if (isStill) {
    if (stillStartTime == 0) {
      stillStartTime = now;
    }

    // 连续静止满 STILL_WINDOW_MS，确认疑似跌倒
    if (now - stillStartTime >= 1500) {
      Serial.println();
      Serial.println("************************");
      Serial.println("*** 疑似跌倒（本地诊断，");
      Serial.println("*** 正式判定以基站为准）");
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
      "UDP #%u fail=%u  ax=%.2fg ay=%.2fg az=%.2fg "
      "gx=%.2f gy=%.2f gz=%.2f dps\n",
      packetSequence,
      (unsigned)g_udpSendFail,
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
  /*
   * 仅由 captureWorker 调用。开灯预热已由主循环的状态机完成。
   *
   * 与 /stream 的竞争：网页视频流会在 handler 里持续 esp_camera_fb_get()，
   * 同一时刻再取帧可能拿到 NULL。抓拍不追求帧率，重试 3 次（间隔 100ms）
   * 让出一个帧周期即可，仍失败才放弃。
   */
  camera_fb_t *fb = NULL;
  for (int attempt = 0; attempt < 3 && fb == NULL; attempt++) {
    fb = esp_camera_fb_get();
    if (!fb) {
      delay(100);
    }
  }

  if (!fb) {
    g_captureFail++;
    Serial.println("抓拍失败：摄像头没有返回图像（可能被视频流占用）");
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
    g_captureFail++;
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
    g_captureFail++;
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
  g_captureOk++;
  Serial.printf("抓拍成功 #%lu：%u 字节，可打开 /pic 查看\n",
                (unsigned long)captureSequence,
                (unsigned int)newPhotoLen);

  return true;
}

void captureWorker(void *parameter) {
  (void)parameter;
  for (;;) {
    if (captureRequestSemaphore == nullptr) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    if (xSemaphoreTake(captureRequestSemaphore, portMAX_DELAY) == pdTRUE) {
      captureAndStorePhoto();
      if (captureDoneSemaphore != nullptr) {
        xSemaphoreGive(captureDoneSemaphore);
      }
    }
  }
}

void requestCapture() {
  // 以下状态和闪光灯只由主循环修改，工作任务只通过信号量通信。
  if (!cameraReady || !captureTaskHandle) {
    g_captureReject++;
    Serial.println("抓拍不可用：摄像头未就绪或工作任务未创建");
    return;
  }
  if (captureBusy) {
    g_captureReject++;
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
    if (captureRequestSemaphore == nullptr ||
        xSemaphoreGive(captureRequestSemaphore) != pdTRUE) {
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

    /*
     * 丢弃当前报文的剩余字节。加计数上限：udp.available() 在报文读完后
     * 仍可能因内部状态返回非 0，无界 while 会卡住主循环、拖慢 MPU 采样。
     */
    int guard = 0;
    while (udp.available() > 0 && guard < 64) {
      (void)udp.read();
      guard++;
    }

    bool validCommand =
        packetSize == 4 &&
        received == 4 &&
        command[0] == 0x55 &&
        command[1] == 0xAA &&
        command[2] == 0x01 &&
        command[3] == 0xFF;

    if (validCommand) {
      g_captureCmdRx++;
      Serial.print("收到抓拍命令，来源：");
      Serial.print(udp.remoteIP());
      Serial.print(":");
      Serial.println(udp.remotePort());
      requestCapture();
    } else {
      /*
       * 8889 对全网开放，未知报文可能高频到达（端口扫描/误配设备）。
       * 逐条打印会在收包路径上刷屏拖慢采样：限流为每 5 秒最多一条。
       */
      static unsigned long lastUnknownPrintMs = 0;
      if (millis() - lastUnknownPrintMs >= 5000) {
        lastUnknownPrintMs = millis();
        Serial.printf("忽略未知UDP命令，长度=%d（每5秒最多提示一次）\n",
                      packetSize);
      }
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

  /*
   * 关闭 brownout 检测器。
   *
   * 现象：射频上电瞬间（WiFi.mode 之后）电压瞬时跌落，检测器判定掉电并
   * 复位芯片，形成「启动→到 WiFi.mode→复位」的无限循环，基站侧永远
   * 收不到数据。
   *
   * 权衡：检测器本意是电压真崩时保护芯片，但在供电刚好处于临界点时
   * 它会让设备完全无法启动，比"电压真崩时死机"更糟。
   *
   * 地址推导（ESP32，非 S3）：
   *   RTC_CNTL_BROWN_OUT_REG = DR_REG_RTCCNTL_BASE + 0xd4
   *   DR_REG_RTCCNTL_BASE    = 0x3ff48000
   *   => 0x3ff480d4
   * 不用 WRITE_PERI_REG + soc 头文件，避免引入额外寄存器定义影响摄像头。
   */
  {
    volatile uint32_t *brown_out_reg = (volatile uint32_t *)0x3ff480d4u;
    *brown_out_reg = 0u;
  }

  latestPhotoMutex = xSemaphoreCreateMutex();

  if (!latestPhotoMutex) {
    Serial.println("照片缓冲区初始化失败");
  }

#if CAMERA_BOOT_ENABLE
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
  /*
   * 初始分辨率直接给 QVGA（320x240）。
   * 用最终分辨率直接初始化，避免先按大分辨率分配缓冲再缩水的浪费：
   * 既省 PSRAM，也缩短初始化时间。
   */
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_JPEG;

  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      /*
       * 有 PSRAM：给足帧缓冲。
       *
       * fb_count 直接决定 /stream 能否连续出帧：httpd_resp_send_chunk 是
       * 阻塞发送，WiFi 拥塞时一帧会占着缓冲不放；只有 2 个缓冲时驱动补不上
       * 新帧，esp_camera_fb_get() 返回 NULL，stream_handler 就会 break，
       * 浏览器自动重连 -> 灯亮/灭反复 = 用户看到的闪光灯闪烁。
       *
       * QVGA JPEG 约 8KB，4 个缓冲在 PSRAM 里不到 64KB，毫无压力。
       */
      config.jpeg_quality = 10;
      config.fb_count = 4;
      config.grab_mode = CAMERA_GRAB_LATEST;
      /* 直接按实际使用的 QVGA 初始化，避免先按 UXGA 分配再降档的浪费 */
      config.frame_size = FRAMESIZE_QVGA;
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
  } else {
    cameraReady = true;

    sensor_t *s = esp_camera_sensor_get();

    if (s != NULL) {
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
    }

    /*
     * 摄像头自检：初始化成功后立刻试着取一帧并打印结果。
     * 这样「初始化成功但取图失败」能直接从串口日志看出来，
     * 不必依赖网页或切换网络。取到帧立即归还，不占内存。
     */
    Serial.println("[CAMERA] selftest: esp_camera_fb_get() ...");
    Serial.flush();
    {
      camera_fb_t *probe = NULL;
      for (int k = 0; k < 3 && probe == NULL; k++) {
        probe = esp_camera_fb_get();
        if (!probe) {
          Serial.printf("[CAMERA] selftest attempt %d: NULL\n", k + 1);
          Serial.flush();
          delay(200);
        }
      }
      if (probe) {
        Serial.printf("[CAMERA] selftest OK: %u bytes, %ux%u, fmt=%d\n",
                      (unsigned)probe->len, (unsigned)probe->width,
                      (unsigned)probe->height, (int)probe->format);
        Serial.flush();
        esp_camera_fb_return(probe);
      } else {
        Serial.println("[CAMERA] selftest FAILED: fb_get always NULL");
        Serial.flush();
      }
    }
  }
#else
  /*
   * 摄像头关闭（CAMERA_BOOT_ENABLE=0）时的运行路径（README §4/§7/§11）：
   * 不碰摄像头引脚与 XCLK，不初始化驱动；cameraReady 保持 false，
   * 抓拍命令仍会接收校验并打印日志（requestCapture 因 cameraReady
   * 为 false 拒绝取图）。MPU/WiFi/50Hz 推流完全不受影响。
   */
  Serial.println("[CAMERA] disabled: hardware unavailable (see README)");
  Serial.flush();
#endif

#if CAMERA_BOOT_ENABLE && defined(LED_GPIO_NUM)
  startupStep("LED: setupLedFlash ENTER");
  setupLedFlash();
  startupStep("LED: setupLedFlash RETURN");

  /*
   * 闪光灯初始化后的二次自检：确认 setupLedFlash() 没有影响摄像头取图。
   * 闪光灯用 LEDC 通道 0 之外的资源；若这里取帧失败，
   * 说明闪光灯的初始化影响了摄像头 XCLK。
   */
  if (cameraReady) {
    camera_fb_t *probe2 = esp_camera_fb_get();
    if (probe2) {
      Serial.printf("[CAMERA] post-LED selftest OK: %u bytes\n",
                    (unsigned)probe2->len);
      esp_camera_fb_return(probe2);
    } else {
      Serial.println("[CAMERA] post-LED selftest FAILED: LEDC conflict!");
    }
    Serial.flush();
  }
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

  /*
   * MPU 初始化后的摄像头自检。
   *
   * MPUWire 是 TwoWire(0)，与摄像头 SCCB 共用 I2C 控制器 0（引脚不同：
   * MPU 用 GPIO14/15，SCCB 用 GPIO26/27）。若 TwoWire::begin() 重置了
   * 整个控制器并影响 SCCB，摄像头会在这里失去响应。加一次取帧自检，
   * 让「接上 MPU 导致摄像头失效」这种情况在串口里立刻可见，
   * 不必等到网页报错才发现。
   */
  if (cameraReady) {
    camera_fb_t *probe3 = esp_camera_fb_get();
    if (probe3) {
      Serial.printf("[CAMERA] post-MPU selftest OK: %u bytes\n",
                    (unsigned)probe3->len);
      esp_camera_fb_return(probe3);
    } else {
      Serial.println("[CAMERA] post-MPU selftest FAILED: I2C conflict!");
    }
    Serial.flush();
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

  /*
   * 等待连接，但不无限阻塞：每 10 秒打印一次状态，每 30 秒重发一次
   * WiFi.begin()。基站热点可能因上电顺序、信道切换或路由器重启而
   * 暂时不可见；卡死在 while 里会让 MPU 采样与 UDP 永远不启动，
   * 表现为基站侧「穿戴端无数据」。
   */
  {
    uint32_t wait_ms  = 0;
    uint32_t retry_ms = 0;

    /*
     * 等待上限 60 秒：超时也继续往下走，不把 setup() 永久卡死。
     * 若一直卡在这个 while 里，MPU 采样、抓拍任务、HTTP 服务全都不会启动，
     * 基站侧永远「穿戴端无数据」。超时后由主循环的 wifi_keepalive()
     * 每 6 秒继续重连，连上即恢复推流。
     */
    const uint32_t kWaitLimitMs = 60000u;

    while (WiFi.status() != WL_CONNECTED && wait_ms < kWaitLimitMs) {
      delay(500);
      wait_ms  += 500;
      retry_ms += 500;

      if ((wait_ms % 10000u) == 0u) {
        Serial.printf("\n[WIFI] waiting... status=%d elapsed=%lus\n",
                      (int)WiFi.status(), (unsigned long)(wait_ms / 1000u));
        Serial.flush();
      }
      if (retry_ms >= 30000u) {
        retry_ms = 0;
        Serial.println("[WIFI] retry WiFi.begin()");
        Serial.flush();
        /*
         * 重试时也必须重设静态 IP。
         * WiFi.disconnect() 会清掉 dhcp/静态地址配置，只调 begin() 的话
         * 下次连上会退回 DHCP 或根本没有地址，基站侧表现为「已连但无数据」。
         * 顺序：disconnect -> config -> begin。
         */
        WiFi.disconnect(true, false);
        delay(100);
        if (!WiFi.config(localIP, gateway, subnet)) {
          Serial.println("[WIFI] retry config failed");
          Serial.flush();
        }
        WiFi.begin(ssid, password);
      }
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connected");
  } else {
    Serial.println();
    Serial.println("[WIFI] not connected within 60s, continue and retry in loop");
    Serial.flush();
  }
  Serial.print("穿戴端静态地址：");
  Serial.println(WiFi.localIP());
  Serial.print("基站网关：");
  Serial.println(WiFi.gatewayIP());
  Serial.print("子网掩码：");
  Serial.println(WiFi.subnetMask());
  /*
   * 绑定抓拍命令端口。
   *
   * 先 stop() 再 begin()：WiFi 尚未连上时 begin() 会失败，
   * 而失败后 socket 可能处于半开状态，后续再次 begin() 仍会失败，
   * 导致命令端口永远收不到包（表现为抓拍不响应）。
   * stop() 把状态清干净，重试才有意义。
   */
  udp.stop();
  delay(20);
  commandPortReady = udp.begin(localUdpPort) != 0;
  Serial.printf("抓拍命令监听：UDP %u -> %s\n", localUdpPort,
                commandPortReady ? "OK" : "FAIL");
  Serial.flush();

  Serial.print("UDP发送目标：");
  Serial.print(receiverIP);
  Serial.print(":");
  Serial.println(receiverPort);

#if CAMERA_BOOT_ENABLE
  startupStep("HTTP: startCameraServer ENTER");
  startCameraServer();
  startupStep("HTTP: startCameraServer RETURN");

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
#else
  /* 摄像头停用：网页服务不启动（没有画面可看），锚点见 README §7 */
  Serial.println("[HTTP] not started: CAMERA_BOOT_ENABLE=0");
  Serial.flush();
#endif

#if !CAMERA_BOOT_ENABLE
  /* 抓拍链路状态行：命令会照常接收校验，但不会取图/闪灯（README §4） */
  Serial.println("[CAPTURE] disabled: camera unavailable");
  Serial.flush();
#endif
  startupStep("SETUP complete; entering main loop");
}

/*
 * 运行期 WiFi 断线自愈。
 *
 * 基站可能因上电顺序、重启、信道切换而短暂离线；若穿戴端在断线后
 * 不主动重连，就会永久停留在"已连过热点但掉线"的状态，基站侧表现为
 * 「穿戴端无数据」，只能靠人工断电重启。
 *
 * 策略：每 3 秒检查一次，连续 6 秒未连上就重新 WiFi.begin()；
 * 重连成功后重新绑定 UDP 8889 命令端口（socket 可能已失效）。
 *
 * 关于本函数内的 delay(20)/delay(50)（2026-10-25 评估保留）：
 *   它们只发生在「链路切换」时刻（掉线重连 / 恢复重绑端口），此时
 *   数据流本已中断，主循环最多漂移 50ms（丢 1~2 个采样点）无实际
 *   影响；而 socket 清理与 config->begin 之间的间隔时序是上板验证过
 *   的修复（c5bd068），在无法编译验证的情况下不改动已验证时序。
 *   若日后要把 delay 改成 millis() 状态机，须连板回归抓拍端口修复。
 */
static void wifi_keepalive(void)
{
  static uint32_t last_check_ms = 0;
  static uint32_t down_since_ms = 0;
  static bool     was_connected = true;

  uint32_t now = millis();

  if (now - last_check_ms < 3000u) {
    return;
  }
  last_check_ms = now;

  if (WiFi.status() == WL_CONNECTED) {
    if (!was_connected) {
      /* 刚恢复：先 stop 再重新绑定命令端口，并刷新本地 IP 打印 */
      udp.stop();
      delay(20);
      commandPortReady = udp.begin(localUdpPort) != 0;
      Serial.printf("[WIFI] reconnected, ip=%s cmd_port=%s\n",
                    WiFi.localIP().toString().c_str(),
                    commandPortReady ? "OK" : "FAIL");
      Serial.flush();
      was_connected = true;
    }
    down_since_ms = 0;
    return;
  }

  /* 掉线中 */
  if (was_connected) {
    was_connected  = false;
    down_since_ms  = now;
    Serial.println("[WIFI] link lost");
    Serial.flush();
  }

  if (down_since_ms != 0u && (now - down_since_ms) >= 6000u) {
    down_since_ms = now;              /* 重置计时，避免每次循环都重连 */
    Serial.println("[WIFI] reconnect attempt");
    Serial.flush();
    /*
     * 顺序：先 disconnect 让接口回到干净状态，再 config 静态地址，
     * 最后 begin。config 必须在 disconnect 之后、begin 之前调用，
     * 否则旧连接会让 config 返回失败、静态地址不生效。
     */
    WiFi.disconnect(true, false);
    delay(50);
    if (!WiFi.config(localIP, gateway, subnet)) {
      Serial.println("[WIFI] config failed, retry later");
    }
    WiFi.begin(ssid, password);
  }
}

void loop() {
  // 非阻塞推进闪光预热、提交抓拍和关闭闪光灯。
  serviceCapture();

  // 监听真实基站发往本机8889端口的4字节抓拍命令。
  checkCaptureCommand();

  // WiFi 断线自愈（基站重启后自动重连，无需人工断电）
  wifi_keepalive();

  // 每20毫秒读取并发送一次MPU数据，即50Hz
  if (mpuReady && millis() - lastMpuReadTime >= 20) {
    lastMpuReadTime = millis();
    printMpuData();
  }

  delay(1);
}
