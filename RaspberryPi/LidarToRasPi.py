import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import json
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# ✅ UDP 설정
UDP_IP = "192.168.1.167"  # 🔹 라즈베리파이 호스트 IP (수정 가능)
UDP_PORT = 5005
MAX_PACKET_SIZE = 8000  # ✅ 패킷 크기 제한

class LidarUDPServer(Node):
    def __init__(self):
        super().__init__('lidar_udp_server')
        self.get_logger().info("📡 Lidar UDP Server Started!")

        # ✅ Best Effort QoS 설정
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # ✅ UDP 소켓 생성
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ✅ /scan 토픽 구독
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile)

    def scan_callback(self, msg):
        """ Lidar 데이터를 받아 UDP로 전송 (패킷 크기 제한 적용) """
        scan_data = []
        angle = msg.angle_min

        for i in range(len(msg.ranges)):
            scan_data.append({
                "angle": round(math.degrees(angle), 2),
                "distance": round(msg.ranges[i], 3)
            })
            angle += msg.angle_increment

        json_data = json.dumps(scan_data)
        json_bytes = json_data.encode()

        # ✅ 패킷이 너무 크면 나눠서 전송
        if len(json_bytes) > MAX_PACKET_SIZE:
            self.get_logger().warning("🚨 Message too large, splitting into smaller packets...")
            num_packets = (len(json_bytes) // MAX_PACKET_SIZE) + 1

            for i in range(num_packets):
                start = i * MAX_PACKET_SIZE
                end = start + MAX_PACKET_SIZE
                packet = json_bytes[start:end]

                # ✅ 마지막 패킷에 "END" 추가하여 패킷 종료 알림
                if i == num_packets - 1:
                    packet += b'END'

                self.sock.sendto(packet, (UDP_IP, UDP_PORT))
                self.get_logger().info(f"📡 Sent packet {i+1}/{num_packets}")

        else:
            self.sock.sendto(json_bytes + b'END', (UDP_IP, UDP_PORT))  # ✅ 단일 패킷도 종료 표시
            self.get_logger().info(f"📡 Sent {len(scan_data)} points via UDP")

def main(args=None):
    rclpy.init(args=args)
    node = LidarUDPServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()