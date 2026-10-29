/*
 * csi_stub.c -- CSI 占位实现（默认构建，行为与历史版本一致）
 *
 * 真实实现已就绪：csi_real.c（WiFi CSI 回调 -> csi_stats 方差统计）。
 * 两者实现同一份 csi_stub.h 接口，由 main/CMakeLists.txt 的
 * BS_CSI_IMPL 在编译期选择（默认 stub；上机联调后可切 real）。
 *
 * 占位行为：
 *   - csi_init()          空操作（返回 ESP_OK）
 *   - csi_feed_packet()   仅计数（日志可验证调用链接通）
 *   - csi_get_variance()  返回常数 0
 *   - csi_ready()         恒 false
 *
 * 主程序通过 csi_ready() 判断，不会把占位值当作真实判据使用，
 * 因此不会出现"占位数据导致误判"的隐患。
 */

#include "csi_stub.h"

#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "CSI";

/* 占位统计计数器：保留字段以便真实实现直接替换，
   同时让日志能显示"已调用次数"，便于验证调用链确实接通。 */
static uint32_t s_packet_count = 0;

esp_err_t csi_init(void)
{
    /* 占位：不注册任何 WiFi CSI 回调。真实实现见 csi_real.c。 */
    ESP_LOGI(TAG, "CSI stub active (BS_CSI_IMPL=stub, variance stays 0)");
    return ESP_OK;
}

void csi_feed_packet(void)
{
    s_packet_count++;
    /* 每 5000 包打一条计数日志：验证调用链在跑（真实实现在 csi_real.c） */
    if (s_packet_count % 5000u == 0u) {
        ESP_LOGD(TAG, "stub fed %u packets", (unsigned)s_packet_count);
    }
}

uint32_t csi_get_variance(void)
{
    return 0u;
}

bool csi_ready(void)
{
    return false;
}
