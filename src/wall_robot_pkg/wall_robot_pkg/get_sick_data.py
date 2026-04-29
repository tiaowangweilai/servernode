import socket
import struct
import time

UDP_IP = "0.0.0.0"
UDP_PORT = 5010
BUFFER_SIZE = 1024

def parse_pose_packet(data):
    if len(data) < 56:
        print("数据包长度不足！")
        return None
    payload = data[16:56]  # 跳过前16字节报文头，仅取报文体

    fmt = '>QQqqiBBH'  # 注意！这里用大端格式：>，否则结果错误
    unpacked = struct.unpack(fmt, payload)
    telegram_count    = unpacked[0]
    timestamp_us      = unpacked[1]
    x_mm              = unpacked[2]
    y_mm              = unpacked[3]
    yaw_mdeg          = unpacked[4]
    localization_stat = unpacked[5]
    mapmatching_stat  = unpacked[6]
    reserved          = unpacked[7]
    yaw_deg = yaw_mdeg / 1000.0
    return {
        "telegram_count": telegram_count,
        "timestamp_us": timestamp_us,
        "x_mm": x_mm,
        "y_mm": y_mm,
        "yaw_deg": yaw_deg,
        "localization_stat": localization_stat,
        "mapmatching_stat": mapmatching_stat,
        "reserved": reserved
    }

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"监听UDP {UDP_IP}:{UDP_PORT} 等待位姿数据...")

    while True:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        print(f"[UDP] Received {len(data)} bytes from {addr}: {data[:16].hex()} ...")
        pose = parse_pose_packet(data)
        if pose:
            print(f"[POSE] X={pose['x_mm']} mm, Y={pose['y_mm']} mm, Yaw={pose['yaw_deg']:.2f}° | 状态={pose['localization_stat']} | 地图匹配={pose['mapmatching_stat']}")
        time.sleep(0.01)

if __name__ == "__main__":
    main()
