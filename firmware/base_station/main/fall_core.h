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
 *   2) 静止段：冲击后【持续】静止满 STILL_WINDOW_MS
 *      （角速度低 + 加速度回到重力附近；被运动打断则计时清零）
 *   3) 判定：持续静止段满 -> 确认跌倒；
 *      冲击后 PENDING_MAX_MS 内未满足则按非跌倒退出
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

/*
 * 冲击阈值：合加速度超过此值认为发生撞击。3.0g
 * 依据（2026-10-18 文献对照，详见 CHANGELOG 同日条目）：
 *  - 学术阈值法常用 2.5g（Kepski & Kwolek, IET 2017：SV_total > 2.5g 判冲击）；
 *  - KFall 数据集门限扫描：gate=3.0g 时 fall recall 仅 0.827（漏约 17%），
 *    gate=1.7g 时 0.896 —— 3.0g 偏高，缓慢跌倒（扶墙滑下/晕倒）可能达不到；
 *  - 本期保持 3.0g：演示场景优先压误报（快速坐下冲击可达 2~4g）。
 *    调参建议：成员 C 拿 SD 卡 CSV 对真实动作做阈值扫描后再决定是否下调。
 */
#define IMPACT_THRESH_G        3.0f

/*
 * 静止确认窗口：冲击后【持续】静止满此时长则确认跌倒。1500ms
 * 2026-10-18 修订：确认按"持续静止计时"执行（静止被运动打断即清零），
 * 不再是"冲击后恰满此时长的那一帧是否静止"的快照式判定——后者在
 * 到期帧恰逢抽动、或静止晚于到期才出现时会漏报（对照文献
 * post-fall-inactivity 标准做法，详见 CHANGELOG）。
 */
#define STILL_WINDOW_MS        1500u

/*
 * 冲击后观察上限：进入 PENDING 后持续这么久（无论动或不动）仍未确认，
 * 则按非跌倒退出（回到 WALK/IDLE）。2026-10-18 新增。
 * 作用：配合"持续静止计时"的确认方式，防止"碰撞后继续走动、随后才
 * 坐下静止"在长观察期内被误确认——到期时坐下静止尚不足 STILL_WINDOW_MS。
 */
#define PENDING_MAX_MS         4000u

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

/*
 * 姿态门（2026-10-18 新增，默认关闭）：
 *   文献对照结论：|a| 模长判据无法区分"快速坐下后坐稳"与"跌倒后躺平"
 *   ——两者都是"冲击 + 之后静止"，这是 magnitude-only 判定的公认头号
 *   误报源（KFall 项目原文 "sitting down heavily sails past it"）。
 *   开启后，CONFIRMED 判定额外要求：确认时刻重力方向与"竖直参考"
 *   的夹角 > POSTURE_LYING_ANGLE_DEG（即人处于躺平朝向）。
 *   竖直参考在 IDLE 持续静止 POSTURE_CALIB_STILL_MS 后自动校准
 *   （上电时人处于直立/坐直姿态）；未校准时门放行（fail-open），
 *   行为与门关闭完全一致。
 *   ⚠️ 前提：佩戴位置/朝向相对固定（胸口/腰部）。手臂佩戴朝向不可靠。
 *   待成员 B 确定佩戴方案后再置 1。
 */
#define POSTURE_CHECK_ENABLE      0
#define POSTURE_CALIB_STILL_MS    2000u

/* 躺平判据：与竖直参考夹角超过此角度视为躺平。60°。
 * 实现用 cos² 定点整数比较（见 fall_core.c posture_allows_confirm）：
 * POSTURE_LYING_COS2_Q4 = cos²(阈值角)·16，60° -> 0.25·16 = 4。
 * 改角度时两个宏必须同步修改。 */
#define POSTURE_LYING_ANGLE_DEG   60.0f
#define POSTURE_LYING_COS2_Q4     4u

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
#if POSTURE_CHECK_ENABLE
    /* 姿态门竖直参考（POSTURE_CHECK_ENABLE=1 时使用，默认关闭不占内存） */
    int16_t  ref_acc[3];        /* 校准时刻的重力向量（人直立/坐直朝向） */
    uint32_t ref_acc2;          /* 参考向量平方模长（LSB²） */
    bool     have_ref;          /* 是否已完成竖直参考校准 */
#endif
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

/*
 * 人工解除告警（操作者显式命令，供 OLED 侧二次确认后调用）。
 *
 * 语义与边界：
 *   - 只对 CONFIRMED 生效：把状态置回 IDLE 并清除冲击/计时字段，
 *     返回 true；其它状态一律返回 false 且不改任何字段。
 *   - 不改变任何阈值，也不替代"持续起身活动"的自动解除判据——
 *     两者并存：自动解除照旧工作，人工解除是额外的显式出口。
 *   - 调用方（base_station.c）负责随后下发 CLEAR 并清除冲击幅值。
 */
bool fall_detector_force_clear(fall_detector_t *d);

/* 状态名，日志用 */
const char *fall_state_name(fall_state_t s);

/* --- 辅助：把三轴原始值换算成物理量的平方，供日志/调试 --- */
uint32_t fall_acc_magnitude2(const int16_t acc[3]);   /* 单位 LSB² */
uint32_t fall_gyro_magnitude2(const int16_t gyro[3]); /* 单位 LSB² */

/*
 * 合加速度幅值，单位 0.01g（即 1.00g -> 100），供 FPGA 数码管显示。
 *
 * 纯整数实现（逐位整数开方），不依赖 math.h —— fall_core.c 必须保持
 * 纯 C99 并且被离线自测宿主程序直接 #include，所以这里不能用 sqrtf。
 *
 * 结果饱和到 9999（99.99g）：FPGA 侧数码管为 4 位 "XX.XX"，超出即无法表示，
 * 与其溢出回绕不如钉在上限。协议见 docs/团队协作规范与接口契约.md §4.3。
 */
uint16_t fall_acc_mag_centi_g(const int16_t acc[3]);

#endif /* FALL_CORE_H */
