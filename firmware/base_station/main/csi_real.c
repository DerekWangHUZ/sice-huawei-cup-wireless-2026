/*
 * csi_real.c -- 真实 CSI 提取实现（替代 csi_stub.c）
 *
 * 实现 csi_stub.h 承诺的接口；构建切换见 main/CMakeLists.txt 的
 * BS_CSI_IMPL（默认 stub，保持历史行为；real 启用本文件）。
 *
 * 数据通路（契约 §2.2 第三段的"CSI 扰动方差"判据来源）：
 *   WiFi 驱动 CSI 回调（每收到一帧携带 LLTF 的无线帧触发一次）
 *     -> csi_stats_accumulate()：只累加 Σ|h| 与 Σ|h|²（驱动上下文约束）
 *   基站每收到一个穿戴端 UDP 姿态包
 *     -> csi_feed_packet() -> csi_stats_commit()：结算单帧方差入滑窗
 *     -> csi_get_variance() / csi_ready()：窗口平均方差 / 数据就绪
 *
 * 为什么 AP 侧能采到 CSI：ESP-CSI 只对【本机收发】的帧上报 CSI
 * （不针对第三方流量，esp-csi issue #247）。本架构穿戴端 STA 以
 * 50Hz 向 SoftAP 持续发包，AP 每收一包即触发一次回调——采集率
 * 与 UDP 包率一致，恰好由 csi_feed_packet() 的调用点天然节流。
 *
 * 上机要点与已知限制：
 *   - 驱动 CSI 能力须开启（CONFIG_ESP_WIFI_CSI_ENABLED=y，或编译注入
 *     该宏），并在本文件初始化时先调 esp_wifi_set_promiscuous(true)
 *     （v6 起 CSI 与混杂模式耦合）——缺一 set_csi_config 即被以
 *     ESP_FAIL 拒绝，两者均已内置；
 *   - 初始化于首帧数据到达时执行（base_station.c 的
 *     csi_late_init_if_needed），保证 SoftAP 完全就绪且 STA 在推流；
 *   - 已知限制：IDF v6.1 SoftAP 下即便满足上述两点，set_csi_config
 *     仍被驱动以 ESP_FAIL 拒绝（LLTF-only 与 LLTF+HT-LTF 均尝试，
 *     未文档化行为）——启用判据前需先解决，跟踪见仓库 issue；
 *   - sigma_csi 阈值标定（静止/走动/跌倒躺平三状态）在采集成功后
 *     进行，取分界填 fall_core.h 的 CSI_FALL_VAR_MAX。
 */

#include "csi_stub.h"
#include "csi_stats.h"

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"

static const char *TAG = "CSI";

/* 回调（驱动上下文）与 commit/getter（任务上下文）的互斥：
 * 回调里只做常数次整数累加，临界区极短。 */
static portMUX_TYPE s_csi_lock = portMUX_INITIALIZER_UNLOCKED;

static void csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;
    if (info == NULL || info->buf == NULL || info->len < 2u) {
        return;
    }
    portENTER_CRITICAL(&s_csi_lock);
    (void)csi_stats_accumulate(info->buf, (uint16_t)(info->len / 2u));
    portEXIT_CRITICAL(&s_csi_lock);
}

esp_err_t csi_init(void)
{
    /*
     * 只取 LLTF（legacy 长训练字段）：
     *   - 穿戴端 STA 50Hz 上行是 legacy rate 帧，LLTF 每帧必在；
     *   - 关闭 HT-LTF/STBC 与相邻子载波滤波，保留子载波间独立性
     *     （滤波会抹平方差特征，恰好与判据目标相反）；
     *   - 子载波数由 info->len 动态给出（S3 LLTF 与 ESP32 classic
     *     数量不同），csi_stats 不假设固定 64。
     */
    wifi_csi_config_t cfg = {
        .lltf_en           = true,
        .htltf_en          = false,
        .stbc_htltf2_en    = false,
        .ltf_merge_en      = false,
        .channel_filter_en = false,
        .manu_scale        = false,
        .shift             = 0,
        .dump_ack_en       = false,
    };
    esp_err_t err;

    /* v6.x 起 CSI 与混杂模式耦合：CSI 系列调用的失败条件明示
     * "promiscuous mode is not enabled"（esp-csi 例程同样先开
     * promiscuous）。未开时 set_csi_config 直接被驱动拒绝。 */
    err = esp_wifi_set_promiscuous(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_promiscuous failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 两种配置组合依序尝试：A=LLTF-only（保留子载波独立性）；
     * B=LLTF+HT-LTF+合并（esp-csi 风格默认）。驱动对组合的接受度
     * 随芯片/驱动版本有差异，任一成功即用。 */
    wifi_csi_config_t try_cfg[2] = { cfg, cfg };
    try_cfg[1].htltf_en     = true;
    try_cfg[1].ltf_merge_en = true;

    for (int i = 0; i < 2; i++) {
        err = esp_wifi_set_csi_config(&try_cfg[i]);
        if (err == ESP_OK) {
            break;
        }
        ESP_LOGW(TAG, "set_csi_config try %d failed: %s", i + 1,
                 esp_err_to_name(err));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_csi_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_set_csi_rx_cb(csi_rx_cb, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_csi_rx_cb failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_wifi_set_csi(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_csi enable failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "CSI capture started (lltf only, per-rx-frame callback)");
    return ESP_OK;
}

void csi_feed_packet(void)
{
    /* 每个姿态 UDP 包结算一次：把最近一帧 CSI 快照入窗。
     * 没有新 CSI 快照时 commit 内部为空操作。 */
    portENTER_CRITICAL(&s_csi_lock);
    csi_stats_commit();
    portEXIT_CRITICAL(&s_csi_lock);
}

uint32_t csi_get_variance(void)
{
    return csi_stats_variance();
}

bool csi_ready(void)
{
    return csi_stats_ready();
}
