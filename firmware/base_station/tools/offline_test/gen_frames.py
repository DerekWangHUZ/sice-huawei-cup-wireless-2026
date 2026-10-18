#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_frames.py -- 生成合成姿态 UDP 帧（18 字节定长），供离线自测喂给
                 fall_core.c 编译出的测试程序。

协议（README / 契约 §1.2）：
  Byte0~1  帧头 0xAA 0x55
  Byte2~3  序号 uint16 小端
  Byte4~15 六轴 int16[6] 小端 = AccX,AccY,AccZ,GyroX,GyroY,GyroZ
  Byte16   XOR 校验：Byte2..Byte15 共 14 字节
  Byte17   帧尾 0x0D

传感器换算（与 fall_core.h 保持一致）：
  加速度 ±16g   -> 1g   = 2048 LSB
  角速度 ±2000°/s -> 1°/s = 16.384 LSB

产出：
  tools/offline_test/data/<scene>.bin    合成帧流（可直接 UDP 发）
  tools/offline_test/data/<scene>.csv    同内容的 CSV，便于人眼核对

用法：
  python gen_frames.py
"""

import struct
import os

ACC_LSB_PER_G = 2048.0
GYRO_LSB_PER_DPS = 16.384

HDR0 = 0xAA
HDR1 = 0x55
TAIL = 0x0D


def build_frame(seq, acc_g, gyro_dps, bad_checksum=False, break_tail=False):
    """构造单帧。acc_g 为 (x,y,z) 单位 g，gyro_dps 为 (x,y,z) 单位 °/s。"""
    body = struct.pack('<H', seq & 0xFFFF)
    ax = int(round(acc_g[0] * ACC_LSB_PER_G))
    ay = int(round(acc_g[1] * ACC_LSB_PER_G))
    az = int(round(acc_g[2] * ACC_LSB_PER_G))
    gx = int(round(gyro_dps[0] * GYRO_LSB_PER_DPS))
    gy = int(round(gyro_dps[1] * GYRO_LSB_PER_DPS))
    gz = int(round(gyro_dps[2] * GYRO_LSB_PER_DPS))

    def clamp16(v):
        if v > 32767:
            return 32767
        if v < -32768:
            return -32768
        return v

    for v in (ax, ay, az, gx, gy, gz):
        body += struct.pack('<h', clamp16(v))

    xor8 = 0
    for b in body:
        xor8 ^= b

    if bad_checksum:
        xor8 ^= 0xFF          # 故意打乱校验和

    tail = TAIL
    if break_tail:
        tail = 0x00           # 故意破坏帧尾

    return bytes([HDR0, HDR1]) + body + bytes([xor8, tail])


# ==========================================================================
# 场景定义
#   每个场景 = (名称, [(时间戳ms, acc_g三元组, gyro三元组), ...])
#   时间戳按 50Hz（20ms 间隔）推进
# ==========================================================================

def scene_idle(n=60):
    """静止站立：合加速度≈1g，角速度≈0"""
    out = []
    for i in range(n):
        out.append((i * 20, (0.02, 0.03, 1.00), (0.5, -0.4, 0.3)))
    return out


def scene_walk(n=80):
    """走动：加速度在 0.7~1.4g 起伏，角速度明显非零"""
    out = []
    for i in range(n):
        t = i * 20
        # 周期性起伏模拟步态
        ay = 0.9 + 0.35 * (1 if (i // 4) % 2 == 0 else -1)
        az = 0.6 if (i // 4) % 2 == 0 else 1.15
        gyro = (60.0 if (i // 6) % 2 == 0 else -55.0, 25.0, -18.0)
        out.append((t, (0.15, ay, az), gyro))
    return out


def scene_fall_lying(n=130):
    """
    跌倒并躺平：
      前 25 帧  走动（500ms > MOTION_HOLD_MS 300ms，确保先进入 WALK）
      第 26 帧  冲击（合加速度 > 3g）
      之后      躺平静止（角速度低、加速度回到 1g 附近）
    期望：IDLE -> WALK -> PENDING -> (静止窗口 1500ms 后) -> CONFIRMED
    """
    out = []
    base = 0
    for i in range(25):
        out.append((base + i * 20, (0.15, 1.10, 0.75), (60.0, 25.0, -18.0)))
    base = 500
    # 冲击帧
    out.append((base, (3.4, -1.8, 2.6), (420.0, -260.0, 130.0)))
    # 躺平：静止 2 秒（100 帧 @20ms = 2000ms > STILL_WINDOW_MS 1500）
    for i in range(1, n - 25):
        out.append((base + i * 20, (0.05, 0.10, 0.98), (1.5, -1.0, 0.8)))
    return out


def scene_fall_recover(n=240):
    """
    冲击后爬起（不应判定为跌倒）：
      前 25 帧  走动
      第 26 帧  冲击
      之后      持续剧烈运动（角速度高），直到 PENDING_MAX_MS(4s)
                观察上限到期退出（2026-10-18 起上限取代旧的 1.5s 快照窗口，
                故本场景数据需延长到 4s 之后）
    期望：IDLE -> WALK -> PENDING -> (PENDING_MAX 到期且仍运动) -> WALK
          全程不进 CONFIRMED
    """
    out = []
    base = 0
    for i in range(25):
        out.append((base + i * 20, (0.15, 1.10, 0.75), (60.0, 25.0, -18.0)))
    base = 500
    out.append((base, (3.4, -1.8, 2.6), (420.0, -260.0, 130.0)))
    for i in range(1, n - 25):
        # 持续大动作：角速度高，不满足静止判据
        out.append((base + i * 20, (0.9, 1.3, 0.8), (180.0, -120.0, 90.0)))
    return out


def scene_fall_getup(n=160):
    """
    跌倒后躺 2 秒（确认告警）再爬起（应解除告警）：
      前 25 帧    走动
      第 26 帧    冲击
      之后 100 帧  躺平静止（> STILL_WINDOW_MS 1500ms -> CONFIRMED）
      最后 35 帧   持续起身活动（角速度 > GETUP_GYRO_DPS 持续 > GETUP_HOLD_MS）
    期望：IDLE -> WALK -> PENDING -> CONFIRMED -> IDLE
    验证告警解除判据 = "人起来了"，而非定时器。
    """
    out = []
    base = 0
    for i in range(25):
        out.append((base + i * 20, (0.15, 1.10, 0.75), (60.0, 25.0, -18.0)))
    base = 500
    out.append((base, (3.4, -1.8, 2.6), (420.0, -260.0, 130.0)))
    for i in range(1, 101):                        # 躺平 2 秒
        out.append((base + i * 20, (0.05, 0.10, 0.98), (1.5, -1.0, 0.8)))
    base = 500 + 100 * 20                          # 2500ms 起身
    for i in range(0, n - 125):                    # 爬起 35 帧 = 700ms
        out.append((base + i * 20, (0.9, 1.3, 0.8), (200.0, -150.0, 100.0)))
    return out


def scene_fall_late_still():
    """
    冲击后持续挪动 2.2s 才静止（2026-10-18 新增回归）：
      前 25 帧    走动
      第 26 帧    冲击
      之后 110 帧  持续运动（2.2s）—— 旧快照式确认会在此漏报
      最后 105 帧  静止 -> 静止计时满 1500ms 确认（冲击后 3.7s，
                  仍在 PENDING_MAX 4s 观察上限内）
    期望：IDLE -> WALK -> PENDING -> CONFIRMED
    """
    out = []
    base = 0
    for i in range(25):
        out.append((base + i * 20, (0.15, 1.10, 0.75), (60.0, 25.0, -18.0)))
    base = 500
    out.append((base, (3.4, -1.8, 2.6), (420.0, -260.0, 130.0)))
    for i in range(1, 111):                        # 挪动 2.2s
        out.append((base + i * 20, (0.9, 1.3, 0.8), (180.0, -120.0, 90.0)))
    for i in range(111, 216):                      # 静止 -> 确认
        out.append((base + i * 20, (0.05, 0.10, 0.98), (1.5, -1.0, 0.8)))
    return out


def scene_fall_boundary_twitch():
    """
    躺稳后恰在 1.5s 窗口到期瞬间抽动（2026-10-18 新增回归）：
      前 25 帧    走动
      第 26 帧    冲击
      71 帧       躺平静止
      5 帧        抽动（纯旋转 gyro=120°/s，恰好覆盖冲击+1500ms 的到期帧）
      之后        重新躺平 -> 静止计时重启 -> 冲击后约 3.0s 确认
    旧快照式确认会在到期帧看到"仍在运动"而退出 PENDING 漏报；
    新实现只清零静止计时，重新躺平后照常确认。
    期望：IDLE -> WALK -> PENDING -> CONFIRMED
    """
    out = []
    base = 0
    for i in range(25):
        out.append((base + i * 20, (0.15, 1.10, 0.75), (60.0, 25.0, -18.0)))
    base = 500
    out.append((base, (3.4, -1.8, 2.6), (420.0, -260.0, 130.0)))
    for i in range(1, 72):                         # 躺平 71 帧
        out.append((base + i * 20, (0.05, 0.10, 0.98), (1.5, -1.0, 0.8)))
    for i in range(72, 77):                        # 抽动 5 帧
        out.append((base + i * 20, (0.05, 0.10, 0.98), (120.0, 0.0, 0.0)))
    for i in range(77, 191):                       # 重新躺平 -> 确认
        out.append((base + i * 20, (0.05, 0.10, 0.98), (1.5, -1.0, 0.8)))
    return out


def scene_bump_walk_sit():
    """
    碰撞后继续走动再坐下（2026-10-18 新增回归，PENDING_MAX 防误报）：
      前 25 帧    走动
      第 26 帧    碰撞冲击（如撞到门框，> 3g）
      160 帧      继续走动（3.2s）
      5 帧        坐下动作（仍在运动）
      之后 85 帧   坐着静止 —— 到 PENDING_MAX(冲击+4s) 时坐下静止仅
                  约 0.7s < STILL_WINDOW_MS，必须按非跌倒退出，绝不报警
    期望：IDLE -> WALK -> PENDING -> IDLE（全程不进 CONFIRMED）
    """
    out = []
    base = 0
    for i in range(25):
        out.append((base + i * 20, (0.15, 1.10, 0.75), (60.0, 25.0, -18.0)))
    base = 500
    out.append((base, (3.4, -1.8, 2.6), (420.0, -260.0, 130.0)))
    for i in range(1, 161):                        # 继续走动 3.2s
        out.append((base + i * 20, (0.15, 1.10, 0.75), (60.0, 25.0, -18.0)))
    for i in range(161, 166):                      # 坐下动作 5 帧
        out.append((base + i * 20, (0.30, 0.80, 1.10), (90.0, -40.0, 20.0)))
    for i in range(166, 251):                      # 坐着静止
        out.append((base + i * 20, (0.05, 0.35, 0.92), (1.2, -0.9, 0.7)))
    return out


SCENES = {
    'idle':                 scene_idle(),
    'walk':                 scene_walk(),
    'fall_lying':           scene_fall_lying(),
    'fall_recover':         scene_fall_recover(),
    'fall_getup':           scene_fall_getup(),
    'fall_late_still':      scene_fall_late_still(),
    'fall_boundary_twitch': scene_fall_boundary_twitch(),
    'bump_walk_sit':        scene_bump_walk_sit(),
}


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    outdir = os.path.join(here, 'data')
    os.makedirs(outdir, exist_ok=True)

    # --- 正常场景 ---
    for name, samples in SCENES.items():
        blob = b''
        rows = []
        seq = 0
        for (t_ms, acc_g, gyro) in samples:
            frame = build_frame(seq, acc_g, gyro)
            blob += frame
            rows.append('%u,%u,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f' % (
                t_ms, seq,
                acc_g[0], acc_g[1], acc_g[2],
                gyro[0], gyro[1], gyro[2]))
            seq += 1

        bin_path = os.path.join(outdir, name + '.bin')
        csv_path = os.path.join(outdir, name + '.csv')
        with open(bin_path, 'wb') as f:
            f.write(blob)
        with open(csv_path, 'w', encoding='utf-8') as f:
            f.write('Timestamp_ms,Seq,AccX_g,AccY_g,AccZ_g,GyroX,GyroY,GyroZ\n')
            f.write('\n'.join(rows) + '\n')
        print('generated %-16s %4d frames  %s' % (name, len(samples), bin_path))

    # --- 负测试：坏校验 / 坏帧尾 ---
    bad = b''
    for i in range(5):
        bad += build_frame(100 + i, (0.0, 0.0, 1.0), (0.0, 0.0, 0.0),
                           bad_checksum=True)
        bad += build_frame(200 + i, (0.0, 0.0, 1.0), (0.0, 0.0, 0.0),
                           break_tail=True)
    with open(os.path.join(outdir, 'bad_frames.bin'), 'wb') as f:
        f.write(bad)
    print('generated %-16s %4d frames  %s' % (
        'bad_frames', 10, os.path.join(outdir, 'bad_frames.bin')))

    # --- 重同步测试：垃圾字节 + 合法帧 ---
    res = b''
    seq = 500
    for i in range(6):
        res += bytes([0x12, 0xAA, 0x99])          # 垃圾（含伪帧头字节）
        res += build_frame(seq, (0.0, 0.0, 1.0), (0.5, 0.5, 0.5))
        seq += 1
    with open(os.path.join(outdir, 'resync.bin'), 'wb') as f:
        f.write(res)
    print('generated %-16s %4d frames  %s' % (
        'resync', 6, os.path.join(outdir, 'resync.bin')))

    print('\nall synthetic data written to: %s' % outdir)


if __name__ == '__main__':
    main()
