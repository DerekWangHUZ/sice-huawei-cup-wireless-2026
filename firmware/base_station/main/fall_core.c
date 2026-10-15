/*
 * fall_core.c -- 跌倒检测核心实现（纯标准 C99，无平台依赖）
 *
 * 详见 fall_core.h 的协议与规则说明。
 */

#include "fall_core.h"

/* ==========================================================================
 * A. 姿态帧流式解析器
 *
 * 实现方式：18 字节线性滑动窗口。
 *   每来一个字节，窗口整体左移一位、新字节进入尾部，然后对"最新的
 *   18 个字节"做一次完整校验（帧头 + XOR + 帧尾）。
 *   - 每字节校验一次 -> 任意相位错位都能在最多 18 字节内重新对齐；
 *   - 只有窗口恰好等于一个完整合法帧时才通过 -> 不会重复计数。
 *   这就是协议要求的"滑动重同步"，保证垃圾字节流后紧跟的合法帧
 *   必然被解出。
 * ========================================================================== */

#define HDR0  0xAAu
#define HDR1  0x55u
#define TAIL  0x0Du

/*
 * 线性滑动窗口。
 * 注意：不要改回"环形固定窗口 + 固定相位校验"的方案 —— 该方案在
 * 相位错位后无法重新对齐，且会在重复帧流上把新旧混合窗口误判为
 * 合法帧（实测 60 帧被误判成 237 帧）。
 */
static uint8_t s_buf[POSTURE_FRAME_LEN];
static uint8_t s_filled;    /* 已填入的字节数（饱和到 18） */

void posture_parser_reset(void)
{
    uint8_t i;
    for (i = 0; i < POSTURE_FRAME_LEN; i++) {
        s_buf[i] = 0;
    }
    s_filled = 0;
}

/* 小端合成 int16 */
static int16_t le_i16(const uint8_t *p)
{
    return (int16_t)(uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

/* 小端合成 uint16 */
static uint16_t le_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

posture_result_t posture_parser_feed(uint8_t byte, posture_frame_t *out)
{
    uint8_t i;
    uint8_t xor8;

    /* 窗口左移一位，新字节进入尾部 */
    if (s_filled < POSTURE_FRAME_LEN) {
        /* 尚未填满：只追加，不做整体左移（等价效果） */
        s_buf[s_filled] = byte;
        s_filled++;
    } else {
        for (i = 0; i < POSTURE_FRAME_LEN - 1; i++) {
            s_buf[i] = s_buf[i + 1];
        }
        s_buf[POSTURE_FRAME_LEN - 1] = byte;
    }

    /* 未满 18 字节 -> 不可能有完整帧 */
    if (s_filled < POSTURE_FRAME_LEN) {
        return POSTURE_WAIT;
    }

    /* 帧头检查：未对齐时静默等待，不报错 */
    if (s_buf[0] != HDR0 || s_buf[1] != HDR1) {
        return POSTURE_WAIT;
    }

    /* 帧尾检查：帧头对上但帧尾错 -> 记为坏帧 */
    if (s_buf[17] != TAIL) {
        return POSTURE_BAD;
    }

    /* XOR 校验：Byte2..Byte15 共 14 字节 */
    xor8 = 0;
    for (i = 2; i <= 15; i++) {
        xor8 ^= s_buf[i];
    }
    if (xor8 != s_buf[16]) {
        return POSTURE_BAD;
    }

    /* 解析成功 */
    if (out != NULL) {
        out->seq     = le_u16(&s_buf[2]);
        out->acc[0]  = le_i16(&s_buf[4]);
        out->acc[1]  = le_i16(&s_buf[6]);
        out->acc[2]  = le_i16(&s_buf[8]);
        out->gyro[0] = le_i16(&s_buf[10]);
        out->gyro[1] = le_i16(&s_buf[12]);
        out->gyro[2] = le_i16(&s_buf[14]);
    }
    return POSTURE_OK;
}

/* ==========================================================================
 * B. 跌倒判定状态机
 * ========================================================================== */

/* 平方阈值，编译期由宏推导，避免运行时浮点开方 */
#define IMPACT_THRESH_LSB   (IMPACT_THRESH_G * ACC_LSB_PER_G)
#define IMPACT_THRESH2      (uint32_t)(IMPACT_THRESH_LSB * IMPACT_THRESH_LSB)

#define STILL_GYRO_LSB      (STILL_GYRO_DPS * GYRO_LSB_PER_DPS)
#define STILL_GYRO2         (uint32_t)(STILL_GYRO_LSB * STILL_GYRO_LSB)

#define WALK_GYRO_LSB       (WALK_GYRO_DPS * GYRO_LSB_PER_DPS)
#define WALK_GYRO2          (uint32_t)(WALK_GYRO_LSB * WALK_GYRO_LSB)

/* 重力附近的加速度平方区间：[(1-tol)*2048]^2 .. [(1+tol)*2048]^2 */
#define STILL_G_LOW_LSB     ((1.0f - STILL_G_TOL) * ACC_LSB_PER_G)
#define STILL_G_HIGH_LSB    ((1.0f + STILL_G_TOL) * ACC_LSB_PER_G)
#define STILL_G_LOW2        (uint32_t)(STILL_G_LOW_LSB  * STILL_G_LOW_LSB)
#define STILL_G_HIGH2       (uint32_t)(STILL_G_HIGH_LSB * STILL_G_HIGH_LSB)

/* 起身判据：角速度超过此值视为活动（告警解除用） */
#define GETUP_GYRO_LSB      (GETUP_GYRO_DPS * GYRO_LSB_PER_DPS)
#define GETUP_GYRO2         (uint32_t)(GETUP_GYRO_LSB * GETUP_GYRO_LSB)

/* 走动判据：|a| 偏离 1g 超过 WALK_THRESH_G
 *   等价于 a2 < (1-t)^2*LSB^2  或  a2 > (1+t)^2*LSB^2 */
#define WALK_G_LOW_LSB      ((1.0f - WALK_THRESH_G) * ACC_LSB_PER_G)
#define WALK_G_HIGH_LSB     ((1.0f + WALK_THRESH_G) * ACC_LSB_PER_G)
#define WALK_G_LOW2         (uint32_t)(WALK_G_LOW_LSB  * WALK_G_LOW_LSB)
#define WALK_G_HIGH2        (uint32_t)(WALK_G_HIGH_LSB * WALK_G_HIGH_LSB)

uint32_t fall_acc_magnitude2(const int16_t acc[3])
{
    /* 64 位中间量防溢出：2048*16=32768 每轴，平方和最大 3*32768² ≈ 3.2e9，
       超过 32 位有符号上界，故用 uint32 时需注意——此处用 uint32 累加，
       3*32768^2 = 3,221,225,472 < 4,294,967,295，仍在 uint32 范围内，安全。 */
    uint32_t x = (uint32_t)((acc[0] < 0) ? -acc[0] : acc[0]);
    uint32_t y = (uint32_t)((acc[1] < 0) ? -acc[1] : acc[1]);
    uint32_t z = (uint32_t)((acc[2] < 0) ? -acc[2] : acc[2]);
    return x * x + y * y + z * z;
}

uint32_t fall_gyro_magnitude2(const int16_t gyro[3])
{
    uint32_t x = (uint32_t)((gyro[0] < 0) ? -gyro[0] : gyro[0]);
    uint32_t y = (uint32_t)((gyro[1] < 0) ? -gyro[1] : gyro[1]);
    uint32_t z = (uint32_t)((gyro[2] < 0) ? -gyro[2] : gyro[2]);
    return x * x + y * y + z * z;
}

const char *fall_state_name(fall_state_t s)
{
    switch (s) {
        case FALL_STATE_IDLE:      return "IDLE";
        case FALL_STATE_WALK:      return "WALK";
        case FALL_STATE_PENDING:   return "PENDING";
        case FALL_STATE_CONFIRMED: return "CONFIRMED";
        default:                   return "UNKNOWN";
    }
}

void fall_detector_init(fall_detector_t *d)
{
    d->state           = FALL_STATE_IDLE;
    d->impact_time_ms  = 0;
    d->motion_start_ms = 0;
    d->calm_start_ms   = 0;
    d->confirm_time_ms = 0;
    d->last_frame_ms   = 0;
    d->last_mag2       = 0;
    d->last_gyro2      = 0;
    d->have_time       = false;
}

bool fall_detector_alarm_active(const fall_detector_t *d)
{
    return (d->state == FALL_STATE_CONFIRMED);
}

/*
 * 状态机主体。
 *
 * 状态迁移总览（每条边对应下方哪段代码，自查用）：
 *   IDLE      -> WALK       : 连续运动 MOTION_HOLD_MS        [段 2]
 *   WALK      -> IDLE       : 连续平静 CALM_HOLD_MS          [段 2]
 *   IDLE/WALK -> PENDING    : |a| > IMPACT_THRESH_G          [段 1]
 *   PENDING   -> CONFIRMED  : 窗口内持续静止达 STILL_WINDOW_MS [段 3]
 *   PENDING   -> WALK       : 窗口超时且仍运动               [段 4]
 *   PENDING   -> IDLE       : 窗口超时且已平静               [段 4]
 *   PENDING   -> PENDING    : 窗口内再次冲击 -> 重开窗口      [段 1]
 *   CONFIRMED -> IDLE       : 持续起身活动 GETUP_HOLD_MS      [段 5]
 *   CONFIRMED -> CONFIRMED  : 人仍躺着，告警保持（无定时解除）[段 5]
 *   任意      -> IDLE       : 帧超时 DATA_TIMEOUT_MS（掉线）  [段 0]
 *                  （CONFIRMED 豁免掉线：告警锁存）
 */
fall_state_t fall_detector_feed(fall_detector_t *d,
                                const int16_t acc[3],
                                const int16_t gyro[3],
                                uint32_t now_ms)
{
    uint32_t mag2  = fall_acc_magnitude2(acc);
    uint32_t gyro2 = fall_gyro_magnitude2(gyro);

    bool impact      = (mag2 > IMPACT_THRESH2);
    bool gyro_quiet  = (gyro2 < STILL_GYRO2);
    bool acc_near_g  = (mag2 >= STILL_G_LOW2) && (mag2 <= STILL_G_HIGH2);
    bool moving      = (mag2 < WALK_G_LOW2) || (mag2 > WALK_G_HIGH2) ||
                       (gyro2 > WALK_GYRO2);

    d->last_mag2  = mag2;
    d->last_gyro2 = gyro2;

    /* --- 段 0：首帧初始化 & 掉线保护 --- */
    if (!d->have_time) {
        d->have_time     = true;
        d->last_frame_ms = now_ms;
        d->calm_start_ms = now_ms;
        return d->state;
    }

    /*
     * 掉线保护。CONFIRMED 态豁免：告警是 10s 锁存，数据断了也不提前掐——
     * 否则 fall_lying 确认后仅剩 0.6s 数据，停发 3s 掉线就把告警掐成
     * 1~3 秒（实测缺陷）。CONFIRMED 只由段 5 的 ALARM_HOLD 解除。
     */
    if (d->state != FALL_STATE_CONFIRMED &&
        now_ms - d->last_frame_ms > DATA_TIMEOUT_MS) {
        /* 长时间没数据：复位到 IDLE，清掉待确认窗口 */
        d->state          = FALL_STATE_IDLE;
        d->motion_start_ms = 0;
        d->calm_start_ms   = now_ms;
        d->last_frame_ms   = now_ms;
        return d->state;
    }
    d->last_frame_ms = now_ms;

    /* --- 段 1：冲击检测（优先级最高，任何非确认态都可能被触发） --- */
    if (impact && (d->state != FALL_STATE_CONFIRMED)) {
        d->state          = FALL_STATE_PENDING;
        d->impact_time_ms = now_ms;
        d->motion_start_ms = 0;
        d->calm_start_ms   = 0;
        return d->state;
    }

    /* --- 段 5：告警保持，直到检测到持续起身活动 ---
     * 解除判据（对齐 fall_recover 的爬起语义）：
     *   角速度 > GETUP_GYRO_DPS 持续 GETUP_HOLD_MS -> 人起来了 -> IDLE。
     * 人持续躺着（静止）则一直保持告警，不定时解除。
     * 冲击不会打断告警（段 1 已排除 CONFIRMED）。
     */
    if (d->state == FALL_STATE_CONFIRMED) {
        if (gyro2 > GETUP_GYRO2) {
            if (d->motion_start_ms == 0) {
                d->motion_start_ms = now_ms;
            }
            if (now_ms - d->motion_start_ms >= GETUP_HOLD_MS) {
                d->state         = FALL_STATE_IDLE;
                d->calm_start_ms = now_ms;
                d->motion_start_ms = 0;
            }
        } else {
            /* 仍躺着：保持告警，起身计时清零 */
            d->motion_start_ms = 0;
        }
        return d->state;
    }

    /* --- 段 3：PENDING 静止确认窗口 --- */
    if (d->state == FALL_STATE_PENDING) {
        if (gyro_quiet && acc_near_g) {
            /* 姿态静止：窗口内持续静止即确认 */
            if (now_ms - d->impact_time_ms >= STILL_WINDOW_MS) {
                d->state          = FALL_STATE_CONFIRMED;
                d->confirm_time_ms = now_ms;
                d->calm_start_ms   = 0;
            }
            /* 未到窗口时间则保持 PENDING，继续观察 */
            return d->state;
        } else {
            /* 窗口内仍在运动：看是否已超时 */
            if (now_ms - d->impact_time_ms >= STILL_WINDOW_MS) {
                /* 超时未静止 -> 不是跌倒，退回运动/静止 */
                d->state = moving ? FALL_STATE_WALK : FALL_STATE_IDLE;
                d->motion_start_ms = moving ? now_ms : 0;
                d->calm_start_ms   = moving ? 0 : now_ms;
            }
            /* 未超时则保持 PENDING（人可能正在爬起/翻滚） */
            return d->state;
        }
    }

    /* --- 段 2 / 段 4：IDLE 与 WALK 之间的运动/平静消抖 --- */
    if (moving) {
        if (d->motion_start_ms == 0) {
            d->motion_start_ms = now_ms;
        }
        d->calm_start_ms = 0;

        if ((d->state == FALL_STATE_IDLE) &&
            (now_ms - d->motion_start_ms >= MOTION_HOLD_MS)) {
            d->state = FALL_STATE_WALK;
        }
    } else {
        if (d->calm_start_ms == 0) {
            d->calm_start_ms = now_ms;
        }
        d->motion_start_ms = 0;

        if ((d->state == FALL_STATE_WALK) &&
            (now_ms - d->calm_start_ms >= CALM_HOLD_MS)) {
            d->state = FALL_STATE_IDLE;
        }
    }

    return d->state;
}

/*
 * 无帧时钟：与 feed 互补，处理"没有新帧也必须推进"的超时。
 *   只管掉线保护（DATA_TIMEOUT_MS 无数据回 IDLE），且**不适用于
 *   CONFIRMED**——数据停摆时告警必须保持（人昏迷 + 设备故障恰是
 *   最需要报警的时刻），告警只由"持续起身活动"解除（feed 段 5）。
 *   历史教训：曾用 ALARM_HOLD 定时解除（人躺地上报警却停了）和
 *   停摆掉线解除（告警被 3s 掉线掐成 1~3s），均已废弃。
 * 只读 last_frame_ms（帧时间戳）不修改；被周期调用时若状态已是
 * 目标态则为幂等空操作。
 */
fall_state_t fall_detector_tick(fall_detector_t *d, uint32_t now_ms)
{
    if (!d->have_time) {
        return d->state;
    }

    /* 掉线保护：与 feed 段 0 同规则（CONFIRMED 豁免） */
    if (d->state != FALL_STATE_IDLE &&
        d->state != FALL_STATE_CONFIRMED &&
        now_ms - d->last_frame_ms > DATA_TIMEOUT_MS) {
        d->state           = FALL_STATE_IDLE;
        d->motion_start_ms = 0;
        d->calm_start_ms   = now_ms;
        return d->state;
    }

    return d->state;
}
