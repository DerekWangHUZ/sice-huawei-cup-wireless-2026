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

    /* 场景 4：冲击后爬起 —— 期望不进 CONFIRMED，回到 WALK */
    {
        fall_state_t exp[] = { FALL_STATE_IDLE, FALL_STATE_WALK,
                               FALL_STATE_PENDING, FALL_STATE_WALK };
        printf("[4] scene: fall_recover\n");
        if (!run_scene("data/fall_recover.bin",
                       "impact then move -> back to WALK, NOT confirmed",
                       exp, 4, dt_ms, 130, 0)) {
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
     *    人还在地上报警器却停了——均为错误语义，已废弃。）
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
