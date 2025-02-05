import socket
import json
import serial
import time

# ✅ UDP 설정
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# ✅ 아두이노 시리얼 설정
ser = serial.Serial('/dev/ttyAMA3', 9600)  # 아두이노와 연결된 시리얼 포트

print("📡 Listening for Lidar UDP messages...")

buffer = b""  # ✅ 패킷 조립을 위한 버퍼

while True:
    data, addr = sock.recvfrom(8192)  # ✅ UDP 패킷 수신
    buffer += data

    # ✅ "END" 문자열이 포함된 경우, 데이터 조립 완료
    if b'END' in buffer:
        buffer = buffer.replace(b'END', b'')  # ✅ "END" 제거

        try:
            lidar_data = json.loads(buffer.decode())  # ✅ JSON 디코딩
            print(f"✅ Received {len(lidar_data)} Lidar points from {addr}")

            # ✅ -90°~0° 및 0°~90°에서 0m을 제외한 가장 작은 거리 찾기
            distances_negative_90_to_0 = [point["distance"] for point in lidar_data if -90.0 <= point["angle"] <= 0 and point["distance"] > 0]
            distances_0_to_90 = [point["distance"] for point in lidar_data if 0 < point["angle"] <= 90 and point["distance"] > 0]

            min_negative_90_to_0 = min(distances_negative_90_to_0) if distances_negative_90_to_0 else None
            min_0_to_90 = min(distances_0_to_90) if distances_0_to_90 else None

            print(f"-90도부터 0도까지 가장 가까운 장애물 거리: {min_negative_90_to_0:.3f}m" if min_negative_90_to_0 else "-90도~0도 사이 장애물 없음")
            print(f"0도부터 90도까지 가장 가까운 장애물 거리: {min_0_to_90:.3f}m" if min_0_to_90 else "0도~90도 사이 장애물 없음")

            # ✅ 장애물이 0.5m 이하로 감지되면 아두이노로 신호 전송
            if (min_negative_90_to_0 and min_negative_90_to_0 <= 0.5) or (min_0_to_90 and min_0_to_90 <= 0.5):
                if not obstacle_detected:
                    print("⚠️ 장애물 감지: 0.5m 이하 → 아두이노로 [1,3] 신호 전송")
                    ser.write(bytes([1, 3]))  # 🚀 1,3 신호 전송

                    print("🔄 즉시 → 아두이노로 [1,2] 신호 전송")
                    ser.write(bytes([1, 2]))  # 🚀 1,2 신호 전송

                    time.sleep(2)  # 10초 대기

                    obstacle_detected = True  # 장애물 감지 상태 유지

                print("🔄 장애물이 계속 0.5m 이하 → 아두이노로 [255,0] 신호 전송")
                ser.write(bytes([255, 0]))  # 🚀 255,0 신호 반복 전송

            else:
                if obstacle_detected:
                    print("✅ 장애물 벗어남 → 동작 초기화")
                obstacle_detected = False  # 장애물 감지 해제

            # ✅ 1도 간격으로 필터링
            filtered_data = []
            last_angle = None

            for point in lidar_data:
                angle = round(point["angle"])  # ✅ 소수점 버리고 정수 각도 사용
                if angle != last_angle:  # ✅ 1도 차이 나는 값만 추가
                    filtered_data.append(point)
                    last_angle = angle

            # ✅ 필터링된 데이터 출력 (처음 180개만)
            for i, point in enumerate(filtered_data[:180]):
                print(f"  [{i}] Angle: {point['angle']}°, Distance: {point['distance']}m")

        except json.JSONDecodeError as e:
            print(f"❌ JSON Decode Error: {e}")

        # ✅ 버퍼 초기화 (다음 데이터 수신을 위해)
        buffer = b""