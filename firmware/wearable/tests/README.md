# 分阶段测试草图

这些草图记录穿戴端从单模块到完整链路的验证过程。日常烧录应使用 `../main/wearable_firmware/`，只有排查具体硬件时才回到本目录。

| 草图 | 用途 | 结果/备注 |
|---|---|---|
| `esp32_cam_test` | 验证 OV2640、Wi-Fi 和网页视频流 | 已通过 |
| `i2c_scanner` | 在 SDA=GPIO14、SCL=GPIO15 扫描 I2C | 已在 `0x68` 找到设备 |
| `mpu_raw_test` | 绕过第三方库，读取识别码和六轴数据 | 已通过，识别码为 `0x74` |
| `mpu6050_test` | 使用 Adafruit MPU6050 库测试 | 会因识别码不匹配而失败，仅留作问题记录 |
| `camera_mpu_test` | 验证摄像头与 MPU 并行运行 | 已通过，MPU 应使用 `TwoWire(0)` |
| `fall_detection_test` | 验证本地两阶段疑似跌倒提示 | 已通过基本动作测试，阈值待标定 |

若扫描器能找到 `0x68`，而 `mpu6050_test` 仍显示“没有找到 MPU6050”，通常不是接线故障；请改用 `mpu_raw_test` 或正式固件继续验证。

