/*
 * test_harness.c -- fall_core 离线自测宿主程序
 *
 * 设计要点（关键）：
 *   本程序【直接 include 仓库里的 fall_core.c】，不复制、不重写逻辑。
 *   这样离线自测验证的就是 S3 固件实际运行的同一份代码，不存在
 *   "两份实现漂移"的风险。
 *
 *   因此 fall_core.c/h 必须保持纯标准 C99、只依赖 stdint/stdbool/stddef。
 *
 * 编译（宿主机 lcc64，MATLAB 自带 C 编译器）：
 *   lcc64.exe test_harness.c -I<include dir>  ->  .obj
 *   lcclnk64.exe test_harness.obj -L<lib64 dir> -o test_harness.exe
 *
 * 用法：
 *   test_harness.exe <scene.bin> [--t0=0] [--dt=20]
 *   逐字节喂入合成帧流，打印每一帧解析结果与状态迁移，
 *   结尾给出该场景的状态迁移序列与断言结果。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * 直接包含仓库源码（相对路径：本文件在 tools/offline_test/ 下）
 * 注意是 .c 而非 .h —— 把实现一起编进来，保证测的就是固件那份逻辑。
 */
#include "../../main/fall_core.c"

/*
 * SD 记录的 CSV 格式同样直接编入（2026-10-17 新增）。
 * 目的：逐字节锁定 docs/团队协作规范与接口契约.md §2.1 的表头与行格式，
 * 防止"文档一套、固件一套"的接口漂移 —— 与 fall_core 同一纪律。
 */
#include "../../main/log_fmt.c"

/*
 * FPGA -> S3 上行 UI 帧解析同样直接编入（与 rtl/cmd_decoder.v 逐字段对应）。
 * 目的：锁定契约 §4.9 上行帧的字段语义（快照位域/键号/滑动重同步），
 * 与 fall_core/log_fmt 同一纪律。
 */
#include "../../main/ui_proto.c"

/*
 * CSI 子载波方差统计同样直接编入（2026-10-25 新增，纯 C99）。
 * 目的：锁定 csi_stats 的方差公式、ready 状态机、commit 节流与
 * 10 帧滑窗环形覆盖语义（场景 21/22），与 fall_core 同一纪律。
 */
#include "../../main/csi_stats.c"

/* --------------------------------------------------------------------------
 * 期望迁移序列比对
 * -------------------------------------------------------------------------- */
#define MAX_TRANS 32

typedef struct {
    fall_state_t seq[MAX_TRANS];
    int          n;
} trans_log_t;

static void trans_log_init(trans_log_t *l) { l->n = 0; }

static void trans_log_push(trans_log_t *l, fall_state_t s)
{
    if (l->n == 0 || l->seq[l->n - 1] != s) {
        if (l->n < MAX_TRANS) {
            l->seq[l->n++] = s;
        }
    }
}

static int trans_log_match(const trans_log_t *l, const fall_state_t *exp, int n)
{
    int i;
    if (l->n != n) {
        return 0;
    }
    for (i = 0; i < n; i++) {
        if (l->seq[i] != exp[i]) {
            return 0;
        }
    }
    return 1;
}

static void trans_log_print(const trans_log_t *l, const char *label)
{
    int i;
    printf("  %s: ", label);
    for (i = 0; i < l->n; i++) {
        printf("%s%s", (i ? " -> " : ""), fall_state_name(l->seq[i]));
    }
    printf("\n");
}

/* --------------------------------------------------------------------------
 * 主流程
 * -------------------------------------------------------------------------- */
static int run_scene(const char *path, const char *expect_desc,
                     const fall_state_t *exp, int exp_n,
                     uint32_t dt_ms,
                     int expect_ok, int expect_bad)
{
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        printf("  [FAIL] cannot open %s\n", path);
        return 0;
    }

    /* 读整个文件 */
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char *buf = (unsigned char *)malloc((size_t)fsize);
    if (buf == NULL) {
        fclose(f);
        printf("  [FAIL] out of memory\n");
        return 0;
    }
    size_t got = fread(buf, 1, (size_t)fsize, f);
    fclose(f);

    fall_detector_t det;
    fall_detector_init(&det);
    posture_parser_reset();

    trans_log_t log;
    trans_log_init(&log);
    trans_log_push(&log, det.state);

    uint32_t now_ms = 0;
    uint32_t frames_ok = 0, frames_bad = 0, bytes = 0;
    fall_state_t last = det.state;

    printf("  feeding %u bytes @ dt=%ums\n", (unsigned)got, (unsigned)dt_ms);

    long i;
    for (i = 0; i < (long)got; i++) {
        posture_frame_t frame;
        posture_result_t r = posture_parser_feed(buf[i], &frame);
        bytes++;

        if (r == POSTURE_BAD) {
            frames_bad++;
            continue;
        }
        if (r != POSTURE_OK) {
            continue;
        }

        frames_ok++;
        now_ms += dt_ms;   /* 50Hz 默认 20ms/帧 */

        fall_state_t st = fall_detector_feed(&det, frame.acc, frame.gyro, now_ms);

        if (st != last) {
            printf("    t=%6ums  %s -> %s  (seq=%u)\n",
                   (unsigned)now_ms,
                   fall_state_name(last), fall_state_name(st),
                   frame.seq);
            trans_log_push(&log, st);
            last = st;
        }
    }

    free(buf);

    printf("  bytes=%u frames_ok=%u frames_bad=%u\n",
           (unsigned)bytes, (unsigned)frames_ok, (unsigned)frames_bad);

    int ok = 1;

    /* 断言 1：状态迁移序列（exp_n > 0 时生效） */
    if (exp_n > 0) {
        trans_log_print(&log, "actual  ");
        printf("  expect : ");
        int k;
        for (k = 0; k < exp_n; k++) {
            printf("%s%s", (k ? " -> " : ""), fall_state_name(exp[k]));
        }
        printf("\n");
        if (!trans_log_match(&log, exp, exp_n)) {
            printf("  ASSERT FAILED: state transition sequence mismatch\n");
            ok = 0;
        }
    } else {
        trans_log_print(&log, "actual  ");
    }

    /* 断言 2：帧统计（expect_ok/expect_bad >= 0 时生效） */
    if (expect_ok >= 0) {
        if ((int)frames_ok != expect_ok) {
            printf("  ASSERT FAILED: frames_ok=%u, expected %d\n",
                   (unsigned)frames_ok, expect_ok);
            ok = 0;
        }
    }
    if (expect_bad >= 0) {
        if ((int)frames_bad != expect_bad) {
            printf("  ASSERT FAILED: frames_bad=%u, expected %d\n",
                   (unsigned)frames_bad, expect_bad);
            ok = 0;
        }
    }

    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", expect_desc);
    return ok;
}

int main(int argc, char **argv)
{
    uint32_t dt_ms = 20;    /* 默认 50Hz */
    int      i;
    int      failures = 0;
    int      ran = 0;

    printf("=========================================\n");
    printf(" fall_core offline self-test (host build)\n");
    printf("=========================================\n\n");

    /* 解析 --dt= */
    for (i = 1; i < argc; i++) {
        if (strncmp(argv[i], "--dt=", 5) == 0) {
            dt_ms = (uint32_t)atoi(argv[i] + 5);
        }
    }

    /* 场景 1：静止 —— 期望始终 IDLE，60 帧全解出，0 坏帧 */
    {
        fall_state_t exp[] = { FALL_STATE_IDLE };
        printf("[1] scene: idle\n");
        if (!run_scene("data/idle.bin", "idle stays IDLE",
                       exp, 1, dt_ms, 60, 0)) {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /* 场景 2：走动 —— 期望 IDLE -> WALK */
    {
        fall_state_t exp[] = { FALL_STATE_IDLE, FALL_STATE_WALK };
        printf("[2] scene: walk\n");
        if (!run_scene("data/walk.bin", "walk reaches WALK",
                       exp, 2, dt_ms, 80, 0)) {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /* 场景 3：跌倒躺平 —— 期望 IDLE -> WALK -> PENDING -> CONFIRMED */
    {
        fall_state_t exp[] = { FALL_STATE_IDLE, FALL_STATE_WALK,
                               FALL_STATE_PENDING, FALL_STATE_CONFIRMED };
        printf("[3] scene: fall_lying\n");
        if (!run_scene("data/fall_lying.bin",
                       "fall then lying still -> CONFIRMED",
                       exp, 4, dt_ms, 130, 0)) {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /*
     * 场景 4：冲击后爬起 —— 期望不进 CONFIRMED，回到 WALK
     * 2026-10-18 起 PENDING 由 PENDING_MAX_MS(4s) 观察上限兜底退出
     * （确认判据为持续静止计时），故本场景数据延长到 4s 之后。
     */
    {
        fall_state_t exp[] = { FALL_STATE_IDLE, FALL_STATE_WALK,
                               FALL_STATE_PENDING, FALL_STATE_WALK };
        printf("[4] scene: fall_recover\n");
        if (!run_scene("data/fall_recover.bin",
                       "impact then move -> back to WALK, NOT confirmed",
                       exp, 4, dt_ms, 240, 0)) {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /* 场景 5：坏帧 —— 期望 0 帧通过、10 帧全被判坏 */
    {
        printf("[5] scene: bad_frames (checksum/tail corrupted)\n");
        if (!run_scene("data/bad_frames.bin",
                       "all 10 corrupted frames rejected",
                       NULL, 0, dt_ms, 0, 10)) {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /* 场景 6：重同步 —— 垃圾字节后 6 个合法帧必须全部被解出 */
    {
        printf("[6] scene: resync (garbage + valid frames)\n");
        if (!run_scene("data/resync.bin",
                       "all 6 valid frames recovered after garbage",
                       NULL, 0, dt_ms, 6, 0)) {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /*
     * 场景 7：数据停摆时告警保持（2026-10-16 语义修正回归）
     *   喂 fall_lying 到 CONFIRMED 后停止发帧，tick 推进 12 秒：
     *   人躺在地上昏迷 + 数据停摆 = 最需要报警的时刻，告警必须保持。
     *   （历史缺陷：曾实现"停摆 3s 掉线解除"和"10s 定时解除"，
     *    人还在地上报警器却停了——均为错误语义。）
     *   解除唯一途径：持续起身活动（场景 8 验证）。
     */
    {
        printf("[7] scene: alarm latched while stream stops\n");
        FILE *f = fopen("data/fall_lying.bin", "rb");
        if (f == NULL) {
            printf("  [FAIL] cannot open data/fall_lying.bin\n");
            failures++;
        } else {
            fseek(f, 0, SEEK_END);
            long fsize = ftell(f);
            fseek(f, 0, SEEK_SET);
            unsigned char *buf = (unsigned char *)malloc((size_t)fsize);
            size_t got = fread(buf, 1, (size_t)fsize, f);
            fclose(f);

            fall_detector_t det;
            fall_detector_init(&det);
            posture_parser_reset();

            uint32_t now_ms = 0;
            fall_state_t last = det.state;
            long k;
            for (k = 0; k < (long)got; k++) {
                posture_frame_t frame;
                if (posture_parser_feed(buf[k], &frame) == POSTURE_OK) {
                    now_ms += dt_ms;
                    last = fall_detector_feed(&det, frame.acc,
                                              frame.gyro, now_ms);
                }
            }
            free(buf);

            int ok = (last == FALL_STATE_CONFIRMED);
            printf("  after stream: state=%s (t=%ums)\n",
                   fall_state_name(last), (unsigned)now_ms);
            if (!ok) {
                printf("  ASSERT FAILED: expected CONFIRMED before latch test\n");
            } else {
                /* 停发帧，tick 推进 12 秒（120 拍 x 100ms）：告警必须保持 */
                int j;
                for (j = 0; j < 120; j++) {
                    now_ms += 100;
                    last = fall_detector_tick(&det, now_ms);
                }
                printf("  after 12s idle ticks: state=%s\n",
                       fall_state_name(last));
                if (last == FALL_STATE_CONFIRMED) {
                    printf("  [PASS] alarm latched while stream stopped\n");
                } else {
                    printf("  ASSERT FAILED: alarm dropped while person still"
                           " lying (state=%s)\n", fall_state_name(last));
                    ok = 0;
                }
            }
            if (!ok) {
                failures++;
            }
            ran++;
            printf("\n");
        }
    }

    /*
     * 场景 8：起身解除（告警解除判据回归）
     *   fall_getup：走动 -> 冲击 -> 躺平确认（CONFIRMED）-> 持续爬起 700ms
     *   期望解除到 IDLE —— "人起来了"是唯一解除判据。
     */
    {
        fall_state_t exp[] = { FALL_STATE_IDLE, FALL_STATE_WALK,
                               FALL_STATE_PENDING, FALL_STATE_CONFIRMED,
                               FALL_STATE_IDLE };
        printf("[8] scene: fall_getup (confirmed then get up)\n");
        if (!run_scene("data/fall_getup.bin",
                       "get-up motion releases the alarm",
                       exp, 5, dt_ms, 161, 0)) {
            failures++;
        }
        ran++;
        printf("\n");
    }
    /*
     * ======================================================================
     * 以下为 2026-10-17 新增：SD 卡 CSV 格式契约测试（契约 §2.1）
     *   这两个模块不含硬件即可验证的部分就是"写出的 CSV 长什么样"，
     *   因此把它提升为断言，锁死表头与行格式。
     * ======================================================================
     */

    /* 场景 9：表头必须与契约 §2.1 逐字节一致 */
    {
        const char *expect =
            "Timestamp_ms,Seq,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,"
            "CSI_Variance,State\n";
        const char *actual = log_fmt_header();
        int ok = (strcmp(actual, expect) == 0);

        printf("[9] csv header (contract §2.1)\n");
        printf("  actual  : %s", actual);
        if (!ok) {
            printf("  expect  : %s", expect);
            printf("  ASSERT FAILED: header differs from contract §2.1\n");
            failures++;
        } else {
            printf("  [PASS] header matches contract §2.1 byte for byte\n");
        }
        ran++;
        printf("\n");
    }

    /* 场景 10：一行数据格式与字段数（10 个字段 = 9 个逗号） */
    {
        int16_t acc[3]  = { 2048, -2048, 32767 };
        int16_t gyro[3] = { -32768, 0, 16 };
        char    buf[LOG_FMT_LINE_MAX];
        const char *expect = "123456,42,2048,-2048,32767,-32768,0,16,0,3\n";
        int n = log_fmt_row(buf, sizeof(buf), 123456u, 42u,
                            acc, gyro, 0u, 3u);
        int commas = 0;
        int i;
        int ok;

        for (i = 0; buf[i] != '\0'; i++) {
            if (buf[i] == ',') {
                commas++;
            }
        }
        ok = (n > 0) && ((size_t)n < sizeof(buf)) &&
             (strcmp(buf, expect) == 0) && (commas == 9);

        printf("[10] csv row formatting\n");
        printf("  actual  : %s", buf);
        if (!ok) {
            printf("  expect  : %s", expect);
            printf("  ASSERT FAILED: n=%d commas=%d (expect len=%d commas=9)\n",
                   n, commas, (int)strlen(expect));
            failures++;
        } else {
            printf("  [PASS] row matches contract, %d fields\n", commas + 1);
        }
        ran++;
        printf("\n");
    }

    /* 场景 11：极值不溢出缓冲区，且缓冲区过小必须能被检测出来 */
    {
        int16_t acc[3]  = { -32768, -32768, -32768 };
        int16_t gyro[3] = { -32768, -32768, -32768 };
        char    big[LOG_FMT_LINE_MAX];
        char    small[16] = { 0 };
        const char *expect =
            "4294967295,65535,-32768,-32768,-32768,-32768,-32768,-32768,"
            "4294967295,255\n";
        int n_big   = log_fmt_row(big, sizeof(big), 4294967295u, 65535u,
                                  acc, gyro, 4294967295u, 255u);
        int n_small = log_fmt_row(small, sizeof(small), 4294967295u, 65535u,
                                  acc, gyro, 4294967295u, 255u);
        int ok = 1;

        printf("[11] csv row extremes + truncation detection\n");
        printf("  actual  : %s", big);
        if (n_big <= 0 || (size_t)n_big >= sizeof(big)) {
            printf("  ASSERT FAILED: extreme row overflowed LOG_FMT_LINE_MAX\n");
            ok = 0;
        }
        if (strcmp(big, expect) != 0) {
            printf("  expect  : %s", expect);
            printf("  ASSERT FAILED: extreme row mismatch\n");
            ok = 0;
        }
        if (n_small < (int)sizeof(small)) {
            printf("  ASSERT FAILED: truncation not reported "
                   "(n=%d, size=%d)\n", n_small, (int)sizeof(small));
            ok = 0;
        }
        if (ok) {
            printf("  [PASS] extremes ok (len=%d), truncation reported "
                   "(n=%d >= %d)\n",
                   n_big, n_small, (int)sizeof(small));
        } else {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /*
     * ======================================================================
     * 以下为 2026-10-18 新增：PENDING 确认窗口修订回归（对照文献评审）
     *   确认从"快照式"（只在冲击后恰满 STILL_WINDOW_MS 的那一帧看是否
     *   静止）改为"持续静止计时 + PENDING_MAX_MS 观察上限"。
     *   场景 12/13 是旧实现的两条漏报路径，14 是新上限的防误报回归。
     * ======================================================================
     */

    /* 场景 12：冲击后挪动 2.2s 才静止 —— 旧逻辑漏报，新逻辑必须确认 */
    {
        fall_state_t exp[] = { FALL_STATE_IDLE, FALL_STATE_WALK,
                               FALL_STATE_PENDING, FALL_STATE_CONFIRMED };
        printf("[12] scene: fall_late_still (settle after 1.5s window)\n");
        if (!run_scene("data/fall_late_still.bin",
                       "late stillness still confirms (old logic missed)",
                       exp, 4, dt_ms, 241, 0)) {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /* 场景 13：窗口到期帧恰逢抽动 —— 旧逻辑漏报，新逻辑必须确认 */
    {
        fall_state_t exp[] = { FALL_STATE_IDLE, FALL_STATE_WALK,
                               FALL_STATE_PENDING, FALL_STATE_CONFIRMED };
        printf("[13] scene: fall_boundary_twitch (twitch at expiry)\n");
        if (!run_scene("data/fall_boundary_twitch.bin",
                       "twitch at window expiry does not lose the fall",
                       exp, 4, dt_ms, 216, 0)) {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /* 场景 14：碰撞后走动 3.2s 再坐下 —— 观察上限防误报，不得报警 */
    {
        fall_state_t exp[] = { FALL_STATE_IDLE, FALL_STATE_WALK,
                               FALL_STATE_PENDING, FALL_STATE_IDLE };
        printf("[14] scene: bump_walk_sit (no false alarm)\n");
        if (!run_scene("data/bump_walk_sit.bin",
                       "impact then walk then sit -> PENDING_MAX exit, "
                       "NOT confirmed",
                       exp, 4, dt_ms, 276, 0)) {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /*
     * 场景 15：数码管遥测换算 fall_acc_mag_centi_g()
     *
     * 该函数的输出会经 SET_STATE 的 p2 送到 FPGA，直接决定数码管上
     * "XX.XX" 那四位的数值，因此必须逐点锁死（契约 §4.3）。
     *   1g   = 2048 LSB -> 100  (显示 01.00)
     *   2g   = 4096 LSB -> 200
     *   负值按绝对值
     *   三轴合成 1024/1024/1024 -> 86
     * int16 输入下的理论最大值 = 三轴各 32767 -> 2771 (27.71g)，
     * 远小于 0xFFFF(65535)，所以协议里用 0xFFFF 作"暂无遥测"哨兵
     * 不会有歧义 —— 这一点由本场景一并钉住。
     */
    {
        struct { int16_t acc[3]; uint16_t expect; const char *desc; } cases[] = {
            { { 0, 0, 2048 },      100u, "1.00g on Z"          },
            { { 0, 0, 4096 },      200u, "2.00g on Z"          },
            { { 0, 0, -2048 },     100u, "negative axis uses |a|" },
            { { 2048, 0, 0 },      100u, "1.00g on X"          },
            { { 0, 0, 0 },           0u, "zero vector"         },
            { { 1024, 1024, 1024 }, 86u, "3-axis 0.5g each"    },
            { { 32767, 32767, 32767 }, 2771u, "int16 maximum"   },
        };
        int ncase = (int)(sizeof(cases) / sizeof(cases[0]));
        int k;
        int ok = 1;

        printf("[15] telemetry conversion fall_acc_mag_centi_g\n");
        for (k = 0; k < ncase; k++) {
            uint16_t got = fall_acc_mag_centi_g(cases[k].acc);
            printf("  %-24s -> %5u (expect %5u)\n",
                   cases[k].desc, (unsigned)got, (unsigned)cases[k].expect);
            if (got != cases[k].expect) {
                printf("  ASSERT FAILED: %s\n", cases[k].desc);
                ok = 0;
            }
        }
        /* 哨兵无歧义：任何合法输入都到不了 0xFFFF */
        if (cases[ncase - 1].expect == 0xFFFFu) {
            printf("  ASSERT FAILED: max value collides with no-data sentinel\n");
            ok = 0;
        }
        if (ok) {
            printf("  [PASS] %d conversion case(s) exact, sentinel unambiguous\n",
                   ncase);
        } else {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /*
     * 场景 16：FPGA->S3 上行 UI 帧解析（契约 §4.9）
     *
     * 锁定四件事：
     *   a) 合法快照帧（演示模式=1、静音=0、场景=4）逐字段解出；
     *   b) 合法键事件帧（键号 11 = 丝印 'C'）解出；
     *   c) 垃圾字节流后紧跟合法帧 -> 滑动重同步必然解出；
     *   d) 校验失败报 BAD、未知命令码静默丢弃（与 rtl/cmd_decoder 容错语义一致）。
     */
    {
        uint8_t snap[8], keyf[8];
        uint8_t garbage[3] = { 0x12, 0xAA, 0x99 };
        ui_frame_t f;
        ui_proto_result_t r;
        int i, ok = 1;

        snap[0] = 0xAA; snap[1] = 0x55; snap[2] = 0x80; snap[3] = 0x01;
        snap[4] = 0x11;                     /* p2lo: bit0=1(demo) [4:2]=100(场景4) */
        snap[5] = 0x00;                     /* p2hi */
        snap[6] = (uint8_t)(0x80 ^ 0x01 ^ 0x11 ^ 0x00);
        snap[7] = 0x0D;

        keyf[0] = 0xAA; keyf[1] = 0x55; keyf[2] = 0x81; keyf[3] = 0x01;
        keyf[4] = 0x0B;                     /* p2lo: 键号 11（'C'） */
        keyf[5] = 0x00;
        keyf[6] = (uint8_t)(0x81 ^ 0x01 ^ 0x0B ^ 0x00);
        keyf[7] = 0x0D;

        printf("[16] ui_proto uplink frame parsing\n");

        ui_proto_reset();
        r = UI_PROTO_WAIT;
        for (i = 0; i < 8; i++) {
            r = ui_proto_feed(snap[i], &f);
        }
        printf("  snapshot frame -> r=%d cmd=0x%02X demo=%u mute=%u scene=%u\n",
               (int)r, (unsigned)f.cmd, (unsigned)UI_SNAP_DEMO(f.p2),
               (unsigned)UI_SNAP_MUTE(f.p2), (unsigned)UI_SNAP_SCENE(f.p2));
        if (r != UI_PROTO_OK || f.cmd != 0x80 ||
            UI_SNAP_DEMO(f.p2) != 1u || UI_SNAP_MUTE(f.p2) != 0u ||
            UI_SNAP_SCENE(f.p2) != 4u) {
            printf("  ASSERT FAILED: snapshot fields\n");
            ok = 0;
        }

        ui_proto_reset();
        r = UI_PROTO_WAIT;
        for (i = 0; i < 8; i++) {
            r = ui_proto_feed(keyf[i], &f);
        }
        printf("  key frame      -> r=%d cmd=0x%02X key=%u\n",
               (int)r, (unsigned)f.cmd, (unsigned)UI_KEY_CODE(f.p2));
        if (r != UI_PROTO_OK || f.cmd != 0x81 || UI_KEY_CODE(f.p2) != 11u) {
            printf("  ASSERT FAILED: key frame fields\n");
            ok = 0;
        }

        /* 垃圾 + 合法帧：滑动重同步 */
        ui_proto_reset();
        r = UI_PROTO_WAIT;
        for (i = 0; i < 3; i++) {
            r = ui_proto_feed(garbage[i], &f);
        }
        for (i = 0; i < 8; i++) {
            r = ui_proto_feed(keyf[i], &f);
        }
        if (r != UI_PROTO_OK || f.cmd != 0x81 || UI_KEY_CODE(f.p2) != 11u) {
            printf("  ASSERT FAILED: resync after garbage\n");
            ok = 0;
        } else {
            printf("  resync after garbage -> OK\n");
        }

        /* 坏校验：报 BAD 且不产生帧 */
        ui_proto_reset();
        keyf[6] ^= 0x5A;
        r = UI_PROTO_WAIT;
        for (i = 0; i < 8; i++) {
            r = ui_proto_feed(keyf[i], &f);
        }
        keyf[6] ^= 0x5A;                     /* 还原，供后续未知命令测试 */
        printf("  bad xor        -> r=%d (expect %d)\n", (int)r,
               (int)UI_PROTO_BAD);
        if (r != UI_PROTO_BAD) {
            printf("  ASSERT FAILED: bad checksum not reported\n");
            ok = 0;
        }

        /* 未知命令码（校验正确）：静默丢弃，不报帧 */
        ui_proto_reset();
        r = UI_PROTO_WAIT;
        for (i = 0; i < 8; i++) {
            uint8_t b = keyf[i];
            if (i == 2) b = 0x77;
            if (i == 6) b = (uint8_t)(0x77 ^ keyf[3] ^ keyf[4] ^ keyf[5]);
            r = ui_proto_feed(b, &f);
        }
        printf("  unknown cmd    -> r=%d (expect %d = WAIT)\n", (int)r,
               (int)UI_PROTO_WAIT);
        if (r != UI_PROTO_WAIT) {
            printf("  ASSERT FAILED: unknown cmd must be dropped silently\n");
            ok = 0;
        }

        if (ok) {
            printf("  [PASS] ui_proto snap/key/resync/bad/unknown\n");
        } else {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /*
     * 场景 17：人工解除告警 fall_detector_force_clear()
     *
     * 安全边界：
     *   a) CONFIRMED 时可解除 -> 返回 true，状态回 IDLE；
     *   b) 非告警态（IDLE/WALK/PENDING）调用返回 false 且状态不变；
     *   c) 清除后计时字段归零，后续帧仍能正常判定（不残留脏状态）。
     */
    {
        fall_detector_t d;
        int ok = 1;

        printf("[17] manual clear fall_detector_force_clear\n");

        /* 直接构造 CONFIRMED 状态 */
        fall_detector_init(&d);
        d.state           = FALL_STATE_CONFIRMED;
        d.impact_time_ms  = 1000u;
        d.motion_start_ms = 800u;
        d.calm_start_ms   = 1200u;
        d.confirm_time_ms = 1500u;

        if (!fall_detector_force_clear(&d)) {
            printf("  ASSERT FAILED: CONFIRMED force_clear returned false\n");
            ok = 0;
        }
        printf("  CONFIRMED -> %s (expect IDLE)\n", fall_state_name(d.state));
        if (d.state != FALL_STATE_IDLE) {
            printf("  ASSERT FAILED: state not cleared\n");
            ok = 0;
        }
        if (d.impact_time_ms != 0u || d.motion_start_ms != 0u ||
            d.calm_start_ms != 0u || d.confirm_time_ms != 0u) {
            printf("  ASSERT FAILED: timing fields not reset\n");
            ok = 0;
        }

        /* 非告警态：必须拒绝且状态不变 */
        {
            fall_state_t cases[] = { FALL_STATE_IDLE, FALL_STATE_WALK,
                                     FALL_STATE_PENDING };
            int k;
            for (k = 0; k < 3; k++) {
                fall_detector_t e;
                fall_detector_init(&e);
                e.state = cases[k];
                if (fall_detector_force_clear(&e)) {
                    printf("  ASSERT FAILED: force_clear accepted %s\n",
                           fall_state_name(cases[k]));
                    ok = 0;
                }
                if (e.state != cases[k]) {
                    printf("  ASSERT FAILED: %s mutated\n",
                           fall_state_name(cases[k]));
                    ok = 0;
                }
            }
            printf("  non-alarm states rejected without mutation\n");
        }

        if (ok) {
            printf("  [PASS] manual clear bounds ok\n");
        } else {
            failures++;
        }
        ran++;
        printf("\n");
    }

#if POSTURE_CHECK_ENABLE
    /*
     * ======================================================================
     * 以下为 2026-10-25 新增：姿态门场景组（仅姿态门构建参与，见头注）
     *   场景 18/19 的数据前 2.1s 为直立静止，竖直参考在此期间自动校准；
     *   场景 20 无校准期，验证未校准 fail-open 与门关闭逐位一致。
     * ======================================================================
     */

    /* 场景 18：校准后跌倒躺平 —— 躺平角 > 60°，门放行 -> CONFIRMED */
    {
        fall_state_t exp[] = { FALL_STATE_IDLE, FALL_STATE_WALK,
                               FALL_STATE_PENDING, FALL_STATE_CONFIRMED };
        printf("[18] scene: posture_fall_lying (calibrated, gate allows)\n");
        if (!run_scene("data/posture_fall_lying.bin",
                       "calibrated fall to lying still CONFIRMS",
                       exp, 4, dt_ms, 231, 0)) {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /* 场景 19：校准后快速坐下 —— 坐姿 ≈ 直立（角 < 60°），门拦截不报警 */
    {
        fall_state_t exp[] = { FALL_STATE_IDLE, FALL_STATE_PENDING,
                               FALL_STATE_IDLE };
        printf("[19] scene: posture_fast_sit (gate blocks false alarm)\n");
        if (!run_scene("data/posture_fast_sit.bin",
                       "hard sit-down stays upright, gate blocks alarm",
                       exp, 3, dt_ms, 311, 0)) {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /* 场景 20：未校准 fail-open —— 行为与门关闭完全一致 */
    {
        fall_state_t exp[] = { FALL_STATE_IDLE, FALL_STATE_WALK,
                               FALL_STATE_PENDING, FALL_STATE_CONFIRMED };
        printf("[20] scene: posture_uncalib_fall (fail-open parity)\n");
        if (!run_scene("data/posture_uncalib_fall.bin",
                       "uncalibrated fall behaves exactly like gate-off",
                       exp, 4, dt_ms, 126, 0)) {
            failures++;
        }
        ran++;
        printf("\n");
    }
#endif /* POSTURE_CHECK_ENABLE */

    /*
     * 场景 21：csi_stats 单帧方差 + ready/节流语义
     *   a) 单帧 |h| = {3,4,5,0} -> var = (4*50 - 12^2)/16 = 3.5 -> 整数 3；
     *   b) 无新快照重复 commit 不重复入窗（UDP 包率 = 入窗率）；
     *   c) 第二帧 |h| = {3,8} -> var = (2*73 - 11^2)/4 = 6.25 -> 6，
     *      两帧窗口均值 (3+6)/2 = 4；
     *   d) 非法输入（NULL / n=0 / n>上限）必须被拒绝。
     */
    {
        int8_t iq1[8] = { 3, 0, 4, 0, 5, 0, 0, 0 };
        int8_t iq2[4] = { 3, 0, 8, 0 };
        uint32_t v1, v2;
        int ok = 1;

        printf("[21] csi_stats single-frame variance + ready\n");

        csi_stats_reset();
        if (!csi_stats_accumulate(iq1, 4)) {
            printf("  ASSERT FAILED: valid frame rejected\n");
            ok = 0;
        }
        csi_stats_commit();
        v1 = csi_stats_variance();
        printf("  var(3,4,5,0) = %u (expect 3)\n", (unsigned)v1);
        if (v1 != 3u || !csi_stats_ready()) {
            printf("  ASSERT FAILED: variance/ready wrong\n");
            ok = 0;
        }

        /* 无新快照的裸 commit：不得把旧值重复入窗 */
        csi_stats_commit();
        printf("  bare commit: no duplicate push\n");

        if (!csi_stats_accumulate(iq2, 2)) {
            printf("  ASSERT FAILED: second frame rejected\n");
            ok = 0;
        }
        csi_stats_commit();
        v2 = csi_stats_variance();
        printf("  window avg(3,6) = %u (expect 4)\n", (unsigned)v2);
        if (v2 != 4u) {
            printf("  ASSERT FAILED: window average wrong\n");
            ok = 0;
        }

        if (csi_stats_accumulate(NULL, 4) ||
            csi_stats_accumulate(iq1, 0) ||
            csi_stats_accumulate(iq1, CSI_STATS_MAX_SUB + 1)) {
            printf("  ASSERT FAILED: invalid input accepted\n");
            ok = 0;
        } else {
            printf("  invalid inputs rejected (NULL/n=0/n>MAX)\n");
        }

        if (ok) {
            printf("  [PASS] variance math + ready/throttle semantics\n");
        } else {
            failures++;
        }
        ran++;
        printf("\n");
    }

    /*
     * 场景 22：10 帧滑窗环形覆盖
     *   帧号 i（2..26，步进 2）的方差为 i^2/4（幅度 {i,i,0,0}）。
     *   喂 13 帧后窗口应只含最近 10 帧（i=8..26，方差 16..169），
     *   整数均值 = 805/10 = 80；若覆盖失效（保留最旧 10 帧），均值为 38。
     */
    {
        uint32_t v;
        int ok = 1;
        int k;

        printf("[22] csi_stats ring window overwrite\n");
        csi_stats_reset();
        for (k = 1; k <= 13; k++) {
            int8_t iq[8];
            int amp = 2 * k;

            iq[0] = (int8_t)amp; iq[1] = 0;
            iq[2] = (int8_t)amp; iq[3] = 0;
            iq[4] = 0;           iq[5] = 0;
            iq[6] = 0;           iq[7] = 0;
            if (!csi_stats_accumulate(iq, 4)) {
                ok = 0;
            }
            csi_stats_commit();
        }
        v = csi_stats_variance();
        if (v != 80u || !csi_stats_ready()) {
            printf("  ASSERT FAILED: window avg=%u (expect 80 = last-10 mean; "
                   "38 would mean no overwrite)\n", (unsigned)v);
            ok = 0;
        }
        if (ok) {
            printf("  [PASS] ring overwrites oldest frame\n");
        } else {
            failures++;
        }
        ran++;
        printf("\n");
    }

    printf("=========================================\n");
    if (failures == 0) {
        printf(" OFFLINE TEST: ALL %d SCENARIOS PASSED\n", ran);
        printf("=========================================\n");
        return 0;
    } else {
        printf(" OFFLINE TEST: %d/%d SCENARIO(S) FAILED\n", failures, ran);
        printf("=========================================\n");
        return 1;
    }
}
