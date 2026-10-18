import socket
import struct
import time

UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(("0.0.0.0", UDP_PORT))

print("=" * 50)
print(f"UDP接收器已经启动，正在监听端口 {UDP_PORT}")
print("等待ESP32-CAM发送数据……")
print("=" * 50)

packet_count = 0
rate_start_time = time.time()

while True:
    data, address = sock.recvfrom(1024)

    if len(data) != 18:
        print(f"收到错误长度：{len(data)} 字节")
        continue

    if data[0] != 0xAA or data[1] != 0x55:
        print("帧头错误")
        continue

    if data[17] != 0x0D:
        print("帧尾错误")
        continue

    checksum = 0

    for value in data[2:16]:
        checksum ^= value

    if checksum != data[16]:
        print("校验失败")
        continue

    sequence, ax, ay, az, gx, gy, gz = struct.unpack_from(
        "<Hhhhhhh", data, 2
    )

    packet_count += 1

    # 每10包显示一次具体数据
    if sequence % 10 == 0:
        print(
            f"序号={sequence:5d}  "
            f"加速度=({ax:6d},{ay:6d},{az:6d})  "
            f"角速度=({gx:6d},{gy:6d},{gz:6d})"
        )

    now = time.time()

    # 每秒显示一次实际接收频率
    if now - rate_start_time >= 1.0:
        rate = packet_count / (now - rate_start_time)

        print(
            f"接收速率：{rate:.1f} 包/秒  "
            f"来源：{address[0]}:{address[1]}"
        )

        packet_count = 0
        rate_start_time = now