/*
 * log_fmt.c -- SD 卡 CSV 记录格式实现（纯标准 C99）
 *
 * 详见 log_fmt.h。集中定义契约 §2.1 的表头与行格式，
 * 使宿主机离线自测可直接断言"固件写出的 CSV 就是契约要求的 CSV"。
 */

#include "log_fmt.h"

#include <stdio.h>

/*
 * 契约 §2.1 表头，逐字节锁定。
 * 注意：不要改动本串（含大小写与下划线），成员 C 直接用它做表头匹配。
 */
#define LOG_HEADER \
    "Timestamp_ms,Seq,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,CSI_Variance,State\n"

const char *log_fmt_header(void)
{
    return LOG_HEADER;
}

int log_fmt_row(char *buf, size_t n,
                uint32_t ts_ms, uint16_t seq,
                const int16_t acc[3], const int16_t gyro[3],
                uint32_t csi_var, uint8_t state)
{
    if (buf == NULL || n == 0u) {
        return -1;
    }

    /*
     * 显式转换：uint32 -> unsigned、int16 -> int，
     * 避免 uint16/int16 经默认提升后与格式符不匹配（-Wformat 已开）。
     */
    return snprintf(buf, n,
                    "%u,%u,%d,%d,%d,%d,%d,%d,%u,%u\n",
                    (unsigned)ts_ms,
                    (unsigned)seq,
                    (int)acc[0], (int)acc[1], (int)acc[2],
                    (int)gyro[0], (int)gyro[1], (int)gyro[2],
                    (unsigned)csi_var,
                    (unsigned)state);
}
