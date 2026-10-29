# CSI 实现选择（同一份 csi_stub.h 接口，编译期二选一）：
#   stub（默认）：csi_stub.c，占位实现，行为与历史版本完全一致；
#   real        ：csi_real.c，真实 WiFi CSI 采集 + csi_stats 方差统计。
# 切换方式：idf.py -DBS_CSI_IMPL=real build
#           （或 cmake -DBS_CSI_IMPL=real 后重新构建）
# csi_stats.c 是纯 C99 统计核心，两种实现共用，始终参与编译，
# 同时被离线自测 test_harness.c 直接 #include 验证。
set(BS_CSI_IMPL "stub" CACHE STRING "CSI implementation: real|stub")

if("${BS_CSI_IMPL}" STREQUAL "real")
    set(CSI_SRC "csi_real.c")
else()
    set(CSI_SRC "csi_stub.c")
endif()

idf_component_register(SRCS "base_station.c"
                            "fall_core.c"
                            "csi_stats.c"
                            "${CSI_SRC}"
                            "fpga_link.c"
                            "fpga_uplink.c"
                            "ui_proto.c"
                            "settings.c"
                            "ui_app.c"
                            "log_fmt.c"
                            "oled_display.c"
                            "sd_log.c"
                    INCLUDE_DIRS ".")
