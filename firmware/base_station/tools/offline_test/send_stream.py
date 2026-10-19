#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
send_stream.py -- 上板发假包工具（穿戴端不在场时的免等待自测手段）

用途：
  电脑连上基站热点 CSI_FALL_AP 后，用本脚本向基站 192.168.4.1:8888
  发送合成的 18 字节姿态帧，代替真实穿戴端推流，验证基站主链路。

数据源：
  复用 gen_frames.py 生成的 data/*.bin（与离线自测完全同一份数据，
  保证"离线测过的帧 = 上板发的帧"）。

用法：
  python send_stream.py walk                 # 发走动场景，默认 50Hz
  python send_stream.py fall_lying           # 发跌倒场景（会触发报警）
  python send_stream.py fall_lying --rate=50 --loop
  python send_stream.py bad_frames           # 发坏帧，观察 frames_bad 增长

期望现象（详见下载文件夹中的《上机操作说明-基站烧录与联调全流程.md》）：
  walk         -> 串口日志 STATE IDLE -> WALK；FPGA 数码管 "run" + LED 环形流水；
                  OLED 大字「行走」
  fall_lying   -> 串口日志 -> CONFIRMED；FPGA 数码管 "FALL" 整屏急闪、
                  蜂鸣器三短一停鸣响、12 颗 LED（led[0..11]）随节拍爆闪；
                  OLED 大字「跌倒告警」2Hz 反色闪烁 + 冲击幅值定格行
  bad_frames   -> 串口 STATS 中 frames_bad 增长，frames_ok 不变

数码管显示说明（契约 §4.8）：整屏完整英文词 —— 无链路 "HELLO" 慢闪 /
  静止 "rEAdy" / 走动 "run" / 待确认 "N.NNG"（右对齐带单位 G）4Hz 快闪 /
  告警 "FALL" 3.3Hz 急闪。基站在收不到有效帧时发 0xFFFF 哨兵，
  待确认画面显示 "------"。
"""

import socket
import sys
import os
import time
import argparse

BASE_IP = '192.168.4.1'
BASE_PORT = 8888
FRAME_LEN = 18


def load_scene(name):
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, 'data', name + '.bin')
    if not os.path.exists(path):
        raise SystemExit('scene data not found: %s (run gen_frames.py first)'
                         % path)
    with open(path, 'rb') as f:
        return f.read()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scene', help='scene name, e.g. idle/walk/fall_lying/'
                                  'fall_recover/bad_frames/resync')
    ap.add_argument('--ip', default=BASE_IP)
    ap.add_argument('--port', type=int, default=BASE_PORT)
    ap.add_argument('--rate', type=int, default=50,
                    help='frames per second (default 50, i.e. one UDP packet '
                         'per frame)')
    ap.add_argument('--loop', action='store_true',
                    help='repeat the scene forever')
    args = ap.parse_args()

    blob = load_scene(args.scene)
    n_frames = len(blob) // FRAME_LEN
    print('scene=%s  %d frames  ->  %s:%d  rate=%dHz%s'
          % (args.scene, n_frames, args.ip, args.port, args.rate,
             '  LOOP' if args.loop else ''))

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    interval = 1.0 / args.rate

    sent = 0
    try:
        while True:
            i = 0
            while i < n_frames:
                # 一帧一个 UDP 包，与真实穿戴端 50Hz 推流行为一致
                chunk = blob[i * FRAME_LEN:(i + 1) * FRAME_LEN]
                s.sendto(chunk, (args.ip, args.port))
                sent += 1
                i += 1
                if args.rate > 0:
                    time.sleep(interval)
            if not args.loop:
                break
            print('  -- loop restart (total frames sent: %d) --' % sent)
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        s.close()
    print('done, %d frames sent' % sent)


if __name__ == '__main__':
    main()
