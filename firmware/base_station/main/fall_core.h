/*
 * fall_core.h -- 跌倒检测核心：姿态帧解析 + 跌倒判定状态机
 *
 * 设计约束（重要）：
 *   本模块是【纯标准 C99】，不得包含任何 ESP-IDF / FreeRTOS / 平台头文件，
 *   只允许 <stdint.h> <stdbool.h> <stddef.h>。
 *   原因：同一份源码要被宿主机离线测试程序直接 #include 编译运行
 *   （见 firmware/base_station/tools/offline_test/），若引入平台依赖则
 *   离线自测无法复用，会产生"两份逻辑漂移"的风险。
 *
 * 两部分内容：
 *   A. posture_parser_*   18 字节姿态 UDP 帧的逐字节流式解析器
 *   B. fall_detector_*    跌倒判定状态机（三段式规则）
 */

#ifndef FALL_CORE_H
#define FALL_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ==========================================================================
 * A. 姿态帧解析（协议见 README.md / docs/团队协作规范与接口契约.md §1.2）
 *
 * 18 字节定长，小端：
 *   Byte0~1  帧头  0xAA 0x55
 *   Byte2~3  序号  uint16 循环自增
 *   Byte4~15 六轴  int16[6] = AccX,AccY,AccZ,GyroX,GyroY,GyroZ
 *   Byte16   校验  Byte2..Byte15 共 14 字节 XOR
 *   Byte17   帧尾  0x0D
 *
 * 解析器是【流式逐字节】的：UDP 一个包正好 18 字节，但流式实现天然
 * 兼容粘包/半包/错位，且能用于串口等字节流场景。
 * 帧头搜索为滑动窗口：垃圾字节流后紧跟合法帧必然被解出。
 * ========================================================================== */

#define POSTURE_FRAME_LEN 18

typedef struct {
    uint16_t seq;        /* 包序号 0..65535 */
    int16_t  acc[3];     /* 加速度原始值：0=X 1=Y 2=Z */
    int16_t  gyro[3];    /* 角速度原始值：0=X 1=Y 2=Z */
} posture_frame_t;

typedef enum {
    POSTURE_WAIT  = 0,   /* 收集中（含正在找帧头），本次未产出帧 */
    POSTURE_OK    = 1,   /* 本字节收完，成功产出一帧，out 已填充 */
    POSTURE_BAD   = 2    /* 校验/帧尾失败，已复位重新找帧头 */
} posture_result_t;

/* 逐字节喂入。返回 POSTURE_OK 时 *out 有效。 */
posture_result_t posture_parser_feed(uint8_t byte, posture_frame_t *out);

/* 复位解析器状态（换流/超时时调用） */
void posture_parser_reset(void);

/* ==========================================================================
 * B. 跌倒判定状态机
 *
 * 规则依据 docs/团队协作规范与接口契约.md §2.2（成员 C 交付的三段式逻辑）：
 *   1) 冲击段：合加速度 |a| > IMPACT_THRESH_G
 *   2) 静止段：冲击后 STILL_WINDOW_MS 内姿态稳定（角速度低 + 加速度回到重力附近）
 *   3) 判定：满足静止段 -> 确认跌倒
 *
 * 传感器换算（MPU6050 直接传 ADC 原始值）：
 *   加速度 ±16g 量程，16 位有符号 -> 1g = 32768/16 = 2048 LSB
 *   角速度 ±2000°/s 量程         -> 1°/s = 32768/2000 = 16.384 LSB
 *
 * 阈值以"平方"形式参与比较以避免浮点 sqrt：
 *   判定 |a| > T  等价于  a2 = ax²+ay²+az²  >  T²
 *   其中 a2 的单位是 LSB²，T² 需换算：(T_g * 2048)²
 * ========================================================================== */

/* --- 可调阈值（成员 C 调参后直接改这里即可生效） --- */

/* 冲击阈值：合加速度超过此值认为发生撞击。3.0g */
#define IMPACT_THRESH_G        3.0f

/* 静止确认窗口：冲击后在此时间内姿态保持静止则确认跌倒。1500ms */
#define STILL_WINDOW_MS        1500u

/* 静止判据：窗口内角速度模长低于此值（°/s） */
#define STILL_GYRO_DPS         50.0f

/* 静止判据：窗口内合加速度需落回 [1-STILL_G_TOL, 1+STILL_G_TOL] g */
#define STILL_G_TOL            0.35f

/* 走动判据：合加速度偏离 1g 超过此值，或角速度超过此值，视为运动。0.25g */
#define WALK_THRESH_G          0.25f
#define WALK_GYRO_DPS          50.0f

/* 运动持续/静止持续的时间门槛（消抖，避免单帧抖动误切状态） */
#define MOTION_HOLD_MS         300u    /* 连续运动这么久 -> WALK */
#define CALM_HOLD_MS           1500u   /* 连续平静这么久 -> IDLE */

/*
 * 告警解除判据（2026-10-16 语义修正）：
 *   CONFIRMED 不是定时解除——人躺在地上昏迷时穿戴端仍在推流（静止特征），
 *   定时解除 = 人还在地上报警器却闭嘴了，报警器失去意义。
 *   解除唯一判据：检测到持续起身活动（对齐 fall_recover 的"爬起"判据）；
 *   数据停摆时告警保持（人昏迷 + 设备故障恰是最需要报警的时刻）。
 */

/* 起身判据：告警中角速度模长超过此值视为活动（°/s） */
#define GETUP_GYRO_DPS         100.0f

/* 起身判据：持续活动这么久才解除（过滤翻身等短暂动作）。400ms */
#define GETUP_HOLD_MS          400u

/* 帧超时：超过此时间没收到新帧则回到 IDLE（穿戴端掉线保护；
 * 仅对非告警态生效，CONFIRMED 告警锁存不受掉线影响） */
#define DATA_TIMEOUT_MS        3000u

/* --- 传感器换算常量 --- */
#define ACC_LSB_PER_G          2048.0f    /* 1g 对应的 ADC 计数 */
#define GYRO_LSB_PER_DPS       16.384f    /* 1°/s 对应的 ADC 计数 */

typedef enum {
    FALL_STATE_IDLE      = 0,   /* 正常静止 */
    FALL_STATE_WALK      = 1,   /* 走动中 */
    FALL_STATE_PENDING   = 2,   /* 已发生冲击，静止确认窗口内 */
    FALL_STATE_CONFIRMED = 3    /* 跌倒确认（告警中） */
} fall_state_t;

typedef struct {
    fall_state_t state;

    /* --- 内部计时与记忆字段（调用方只读 state，别改这些） --- */
    uint32_t impact_time_ms;    /* 最近一次冲击时刻 */
    uint32_t motion_start_ms;   /* 连续运动的起始时刻（0=未运动）；
                                 * CONFIRMED 态下复用为"起身活动"起始时刻 */
    uint32_t calm_start_ms;     /* 连续平静的起始时刻（0=未平静） */
    uint32_t confirm_time_ms;   /* 进入 CONFIRMED 的时刻（日志用） */
    uint32_t last_frame_ms;     /* 上一帧时间戳 */
    uint32_t last_mag2;         /* 上一帧合加速度平方（LSB²），供日志 */
    uint32_t last_gyro2;        /* 上一帧角速度平方（LSB²） */
    bool     have_time;         /* last_frame_ms 是否已初始化 */
} fall_detector_t;

/* 初始化状态机（上电调用一次） */
void fall_detector_init(fall_detector_t *d);

/*
 * 每收到一帧调用一次。
 *   now_ms  单调递增毫秒时间戳（S3 侧用 esp_timer_get_time()/1000）
 * 返回本次处理后的当前状态。
 */
fall_state_t fall_detector_feed(fall_detector_t *d,
                                const int16_t acc[3],
                                const int16_t gyro[3],
                                uint32_t now_ms);

/*
 * 无帧时钟：数据流停摆时也要推进的超时逻辑（真机缺陷修复 2026-10-16）。
 *   - 告警保持 ALARM_HOLD_MS 到点必须解除——不能依赖"下一帧到达"才执行，
 *     否则演示/掉线场景下 FPGA 会永远响（实测缺陷）。
 *   - DATA_TIMEOUT_MS 掉线保护同理。
 * 主循环以固定周期（如 100ms）调用，与 feed 互补；返回当前状态。
 */
fall_state_t fall_detector_tick(fall_detector_t *d, uint32_t now_ms);

/* 是否处于告警态（state == CONFIRMED） */
bool fall_detector_alarm_active(const fall_detector_t *d);

/* 状态名，日志用 */
const char *fall_state_name(fall_state_t s);

/* --- 辅助：把三轴原始值换算成物理量的平方，供日志/调试 --- */
uint32_t fall_acc_magnitude2(const int16_t acc[3]);   /* 单位 LSB² */
uint32_t fall_gyro_magnitude2(const int16_t gyro[3]); /* 单位 LSB² */

#endif /* FALL_CORE_H */
