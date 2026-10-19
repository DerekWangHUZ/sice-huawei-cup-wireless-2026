# ESP32-CAM 穿戴端

> 硬件：**AI Thinker ESP32-CAM + OV2640 + MPU 六轴模块**
> 固件入口：`main/wearable_firmware/wearable_firmware.ino`

本工程是跌倒检测系统的穿戴标签端：以 50 Hz 向固定基站推送 18 字节原始 IMU 帧，
摄像头当前**不可用**（硬件故障，详见第 11 节），网页视频与 `/pic` 抓拍均已停用；
本工程实际运行的能力是 **50 Hz 姿态推流** 与 **UDP 8889 抓拍命令接收（仅记录）**。

---

## 1. 目录结构

```text
wearable/
├── main/
│   └── wearable_firmware/
│       ├── wearable_firmware.ino   # Arduino 主草图（MPU 采样 / UDP 推流 / 抓拍调度）
│       ├── app_httpd.cpp           # 摄像头网页服务 + /pic 照片路由
│       ├── board_config.h          # 板型选择（CAMERA_MODEL_AI_THINKER）
│       ├── camera_pins.h           # 各板型摄像头引脚
│       ├── camera_index.h          # 网页首页资源
│       ├── ci.yml                  # 编译矩阵
│       └── partitions.csv          # 分区表
├── tests/                          # 分阶段硬件自测草图
│   ├── esp32_cam_test/             # 摄像头与网页视频流
│   ├── i2c_scanner/                # 扫描 MPU 的 I2C 地址
│   ├── mpu_raw_test/               # 直接读取 MPU 寄存器
│   ├── mpu6050_test/               # Adafruit 库兼容性记录，不用于正式固件
│   ├── camera_mpu_test/            # 摄像头与 MPU 同时运行
│   └── fall_detection_test/        # 本地跌倒判定测试
└── tools/
    └── udp_receiver.py             # 电脑端 18 字节协议接收与校验工具
```

Arduino 要求主 `.ino` 与所在文件夹同名，因此正式入口为
`main/wearable_firmware/wearable_firmware.ino`。
**打开本目录的 ino，不要只把 ino 拷进别的工程，也不要混用其他版本的 `app_httpd.cpp`**
（主草图与网页服务通过 `lockLatestPhoto()` / `unlockLatestPhoto()` 协作，两者必须配套）。

---

## 2. 网络参数

| 项目 | 值 |
|---|---|
| 热点提供方 | ESP32-S3 基站（SoftAP） |
| SSID | `CSI_FALL_AP` |
| 密码 | `12345678` |
| 固定信道 | 6（由基站固定，穿戴端跟随接入） |
| 基站 IP | `192.168.4.1` |
| 穿戴端静态 IP | `192.168.4.2` |
| 子网掩码 | `255.255.255.0` |
| 上行姿态数据 | 穿戴端 → `192.168.4.1:8888/UDP` |
| 抓拍命令监听 | 穿戴端 ← `192.168.4.2:8889/UDP` |
| 摄像头主页 | 不可用（摄像头故障） |
| 最近抓拍照片 | 不可用（摄像头故障） |

穿戴端以**静态地址**接入：`setup()` 中依次执行 `WiFi.mode(WIFI_STA)` →
`WiFi.config(192.168.4.2, 192.168.4.1, 255.255.255.0)` → `WiFi.begin()`，
任一步失败即打印原因并停止后续初始化。

> 基站侧须把 `192.168.4.2` 排除在 DHCP 地址池之外，避免后接入的电脑先占用该地址，
> 导致抓拍命令发错设备。基站固件已将该地址池设为 `192.168.4.10 ~ 192.168.4.100`。

---

## 3. 上行姿态帧（18 字节，50 Hz）

发送频率 50 Hz（每 20 ms 一帧），所有多字节整数均为**小端序**。

| 偏移 | 内容 | 类型 |
|---|---|---|
| 0–1 | 固定帧头 `AA 55` | 2 字节 |
| 2–3 | 循环包序号 | `uint16_t` |
| 4–5 | AccX 原始值 | `int16_t` |
| 6–7 | AccY 原始值 | `int16_t` |
| 8–9 | AccZ 原始值 | `int16_t` |
| 10–11 | GyroX 原始值 | `int16_t` |
| 12–13 | GyroY 原始值 | `int16_t` |
| 14–15 | GyroZ 原始值 | `int16_t` |
| 16 | Byte 2–15 的 XOR | `uint8_t` |
| 17 | 固定帧尾 `0D` | 1 字节 |

电脑端自测：

```powershell
python tools/udp_receiver.py
```

出现约 `50.0 包/秒`，且晃动 MPU 时六轴原始值变化，即表示推流链路正常。

---

## 4. 抓拍链路（UDP 8889）—— 当前不可用

> **当前状态：摄像头硬件故障，本条链路只到"接收并校验命令"为止。**
> 命令会被正确解析并打印日志，但不会取图、不会闪灯、`/pic` 不可访问。
> 更换摄像头模组或排线后，把 `CAMERA_BOOT_ENABLE` 改回 1 重新编译即可恢复。

1. 基站判定跌倒后，向 `192.168.4.2:8889` 发送 **4 字节**命令 `55 AA 01 FF`。
2. 穿戴端在 `loop()` 中调用 `checkCaptureCommand()` 轮询该端口，校验帧长与字节内容；
   非法命令打印后丢弃，单轮最多处理 4 个报文，避免阻塞采样。
3. 校验通过后置位抓拍请求：先点亮板载闪光灯预热 **150 ms**，再交给**独立工作任务**
   （`capture_worker`，运行在 core 0）取图。
4. 工作任务把 JPEG 复制到 PSRAM（失败时回落普通内存），挂到互斥量保护的照片缓冲区，
   替换上一张并释放旧图；主循环继续以 50 Hz 采样，**不因抓拍阻塞**。
5. 闪光灯在约 1 s 后由主循环状态机自动关闭。
6. 浏览器访问 `http://192.168.4.2/pic` 可查看最近一次触发的照片；
   尚未抓拍时返回 404，断电后照片丢失、需重新抓拍。

> 抓拍命令报文头 `55 AA` 与上行姿态帧头 `AA 55` **方向相反**，两者不要混淆。
> 网页上的 Get Still 按钮是另一条 HTTP 接口，不能代替基站 UDP 抓拍。

---

## 5. 硬件接线

### 5.1 MPU 与 ESP32-CAM

| MPU 引脚 | ESP32-CAM 引脚 | 当前线色 |
|---|---|---|
| `VCC` | `3V3` | 白 |
| `GND` | 普通 `GND` | 黑 |
| `SCL` | `GPIO15` | 紫 |
| `SDA` | `GPIO14` | 蓝 |

MPU 不要接 5V。本方案不使用 ESP32-CAM 的 microSD 卡槽，因为 `GPIO14/15` 已用于 MPU。

### 5.2 ESP32-CAM-MB 与 ESP32-CAM（脱离插座调试时）

| MB 连接 | ESP32-CAM 引脚 | 当前线色 |
|---|---|---|
| 供电 | `5V` | 灰 |
| 地 | 普通 `GND` | 黑 |
| 串口发送 | `U0T` | 白 |

注意：板上标为 `GND/R` 的引脚带复位功能，不作为普通地线使用。

---

## 6. 开发环境与烧录

- Arduino IDE：已在 `2.3.10` 验证。
- 开发板：`AI Thinker ESP32-CAM`。
- 串口监视器：`115200 baud`。
- 正式固件直接读写 MPU 寄存器，不依赖 Adafruit MPU6050 库。
- 摄像头网页代码来自 Arduino ESP32 的 CameraWebServer 示例配套文件。

| 设置 | 值 |
|---|---|
| Board | AI Thinker ESP32-CAM |
| Port | 当前真实出现的 COM 口 |
| Upload Speed | 115200 |
| Flash Mode | DIO |
| Flash Frequency | 40MHz |
| Partition Scheme | Huge APP (3MB No OTA/1MB SPIFFS) |
| Core Debug Level | Info |

烧录流程：

1. 关闭其他串口监视器，拔 USB，按已验证方向把板子插回 USB 底板。
2. 用能识别 COM 口的 USB 数据线连接电脑。
3. 点 Upload；若需手动下载，按住 IO0、短按并松开 RST，保持 IO0 直到开始 Writing 再松开。
4. 等写入校验成功、无 `Failed uploading`。
5. 烧录完成后断开电脑 USB，再用充电宝 5V 供电运行；**不要同时用电脑 USB 和外接 5V 并联供电**。

---

## 7. 启动日志核对

串口 115200。省略 `[STEP ... ms]` 行后的关键锚点：

```text
[BOOT] wearable_firmware: static IP + async capture
[CAMERA] disabled: hardware unavailable (see README)
抓拍任务已启动（异步执行）
MPU WHO_AM_I = 0x74
MPU启动成功
WiFi connected
穿戴端静态地址：192.168.4.2
基站网关：192.168.4.1
抓拍命令监听：UDP 8889
UDP发送目标：192.168.4.1:8888
[HTTP] not started: CAMERA_BOOT_ENABLE=0
[CAPTURE] disabled: camera unavailable
```

一直停在 `WiFi connecting` 通常需检查基站是否开机、热点参数是否一致。
`Brownout` 或重复启动日志属于供电不足，先解决供电再看其余。

---

## 8. 真机验收步骤

1. 基站串口确认 `SoftAP started: ssid=CSI_FALL_AP channel=6 ip=192.168.4.1`
   与 `UDP listening on :8888`。
2. 穿戴端上电；电脑连 `CSI_FALL_AP`，确认电脑在 `192.168.4.x` 网段、网关为
   `192.168.4.1`，且**电脑本机不是 `192.168.4.2`**。
3. 浏览器打开 `http://192.168.4.2/`，先确认视频画面，再点 Stop Stream。
   串口打出 `Camera Ready` 本身不证明 HTTP 实际可访问。
4. 基站连续观察 30 秒的 `STATS packets=... frames_ok=... frames_bad=...`：
   计数应增长，坏帧不持续增加。相邻有效帧增量除以实际时间间隔应接近 50 包/秒。
5. 基站按其已验证动作方案判定（模块固定在绝缘支撑上受控操作，不让真人跌倒、不扔板撞地），
   应经历 `PENDING -> CONFIRMED` 并打印 `snap cmd sent to 192.168.4.2:8889`。
   未达到 CONFIRMED 不等于抓拍接口失效。
6. 确认穿戴端闪灯约 1 秒，打开 `http://192.168.4.2/pic` 查看照片；
   期间基站姿态包应持续接收。
7. 按基站恢复动作解除告警、更换镜头前物体后再次触发，刷新 `/pic` 确认照片更新。
   重复至少 3 轮。
8. 打开实时流，同时运行姿态接收与抓拍，持续至少 5 分钟，记录实际速率、坏帧、断流与重启。
   单独视频正常不能代替此项。
9. 如已连接 FPGA，实际核对声光报警及解除；未接则记录为“穿戴端—基站接口联调”，
   不宣称整机报警已通过。

> 第一次还未抓拍时 `/pic` 返回 404 属正常。基站日志“已发送”只代表发送调用完成，
> 不能代替穿戴端收到命令、闪灯与照片实际更新的证据。

---

## 9. 重要兼容性说明

1. 当前 MPU 模块在 I2C 地址 `0x68` 可正常读取，但返回 `WHO_AM_I = 0x74`。
   Adafruit MPU6050 库会因型号识别不匹配而报告“没有找到 MPU6050”，
   所以正式固件采用寄存器直读方式。
2. 摄像头与 MPU 同时运行时，MPU 使用 `TwoWire(0)`。
   改用 `TwoWire(1)` 会出现 `I2C bus id(1) has already been acquired`。
3. 摄像头默认切到 QVGA，以降低延迟和内存压力；网页中继续调高分辨率可能明显卡顿。
4. `mpu6050_test` 仅保留失败排查过程，不是正式入口。

---

---

## 10. 范围外

- 算法对真实跌倒的准确性、长时间续航、穿戴可靠性，均不由接口联调自动证明。
- 充电宝供电、线材固定、外壳佩戴、续航与长时间稳定性需单独实测。

---

## 11. 摄像头与抓拍链路

### 抓拍流程

基站判定跌倒后向穿戴端 `192.168.4.2:8889/UDP` 下发 4 字节命令 `55 AA 01 FF`。

```text
checkCaptureCommand()  校验命令，ok -> requestCapture()
requestCapture()       点亮闪光灯，标记 150 ms 预热，置 captureBusy
serviceCapture()       预热到点后交出信号量（主循环每轮推进）
captureWorker()        取一帧 JPEG，复制到 PSRAM
latest_photo_handler() /pic 返回该帧
```

抓拍在独立任务（core 0）里执行，主循环的 50 Hz 采样与 UDP 发送不受影响。

### 抓拍命令端口

命令端口用 `udp.begin(8889)` 绑定。绑定前必须先 `udp.stop()`：

WiFi 尚未连上时 `begin()` 会失败，失败后 socket 可能留在半开状态，
后续再次 `begin()` 仍会失败。若不清理状态，命令端口会永久失效，
`checkCaptureCommand()` 因 `commandPortReady == false` 直接返回，
表现为抓拍无响应、`/pic` 一直 `404`。

### 自检点

固件在三个位置各取一帧并打印结果，便于从串口直接判断摄像头状态：

| 自检 | 位置 |
|---|---|
| `selftest` | `esp_camera_init()` 之后 |
| `post-LED selftest` | `setupLedFlash()` 之后 |
| `post-MPU selftest` | `setupMpu()` 之后 |

正常输出示例：

```text
[CAMERA] esp_camera_init RETURN 0x0
[CAMERA] selftest OK: 8518 bytes, 320x240, fmt=4
[CAMERA] post-LED selftest OK: 8311 bytes
[CAMERA] post-MPU selftest OK: 8779 bytes
```

### 抓拍链路诊断

`/status` 在摄像头字段之后追加以下计数，不接串口也能定位卡在哪一步：

| 字段 | 含义 |
|---|---|
| `camera_ready` | 摄像头是否初始化成功 |
| `cmd_port_ready` | 8889 命令端口是否绑定成功 |
| `capture_cmd_rx` | 收到并通过校验的抓拍命令数 |
| `capture_reject` | 因未就绪或忙而被拒绝的次数 |
| `capture_ok` | 成功存入 PSRAM 的照片数 |
| `capture_fail` | 取帧或分配内存失败的次数 |

判读：`cmd_port_ready=0` 为端口绑定问题；`cmd_rx>0` 而 `ok=0` 为取帧问题；
`ok>0` 而 `/pic` 返回 `404` 为存储或路由问题。

### 帧缓冲

`fb_count` 决定连续出帧能力。`httpd_resp_send_chunk()` 是阻塞发送，
WiFi 拥塞时一帧会占住缓冲不放；缓冲过少时驱动补不上新帧，
`esp_camera_fb_get()` 返回 `NULL`。

- `fb_count = 4`，`grab_mode = CAMERA_GRAB_LATEST`
- 分辨率 `QVGA`（320x240），单帧约 8 KB，4 个缓冲在 PSRAM 中占用不到 64 KB
- `stream_handler` 对取帧失败做 10 次重试（每次 50 ms），避免瞬时竞争导致断流

### 排线接触

`esp_camera_init()` 返回 `0x106`（`ESP_ERR_NOT_SUPPORTED`）表示 SCCB 读不到
传感器型号，属于排线接触问题：断电后掀开排线座卡扣、拔出排线、重新插到底并
锁上卡扣即可。固件本身不需要改动。