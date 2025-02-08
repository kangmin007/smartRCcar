import socket
import json
import serial
import time
import threading
# ✅ UDP 설정
UDP_IP = "0.0.0.0"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# ✅ 아두이노 시리얼 설정
ser = serial.Serial('/dev/ttyAMA3', 9600)  # 아두이노와 연결된 시리얼 포트

# PI 제어 변수
Kp = 2.5  # 비례 게인
Ki = 0.7  # 적분 게인
integral_error = 0  # 적분 항 초기화
previous_time = time.time()

# ✅ 속도 조절 쓰레드 플래그 (전역 변수로 선언)
speed_control_active = False
speed_thread = None  # 쓰레드 객체 초기화
pi_control_active = False  # ✅ 장애물 감지 여부
print("📡 Listening for Lidar UDP messages...")

buffer = b""  # ✅ 패킷 조립을 위한 버퍼
obstacle_detected = False  # ✅ 장애물 감지 여부 (초기값 False)

# ✅ (1,1), (1,0) 반복 송신을 위한 쓰레드 함수
def speed_control():
    global speed_control_active
    while speed_control_active:
        ser.write(bytes([1, 1]))  # 속도 조절 신호 (1,1)
        print("🚀 속도 감소: (1,1) 전송")
        time.sleep(0.5)  # 0.5초 대기

        ser.write(bytes([1, 0]))  # 속도 유지 신호 (1,0)
        print("🚀 속도 유지: (1,0) 전송")
        time.sleep(0.5)  # 0.5초 대기

while True:
    data, addr = sock.recvfrom(8192)  # ✅ UDP 패킷 수신
    buffer += data

    # ✅ "END" 문자열이 포함된 경우, 데이터 조립 완료
    if b'END' in buffer:
        buffer = buffer.replace(b'END', b'')  # ✅ "END" 제거

        try:
            lidar_data = json.loads(buffer.decode())  # ✅ JSON 디코딩

            # ✅ 8개 영역으로 분할
            angle_ranges = [
                (-60, -45), (-45, -30), (-30, -15), (-15, 0),
                (0, 15), (15, 30), (30, 45), (45, 60)
            ]

            min_distances = []
            for angle_range in angle_ranges:
                distances = [point["distance"] for point in lidar_data
                             if angle_range[0] <= point["angle"] <= angle_range[1] and point["distance"] > 0]
                min_distances.append(min(distances) if distances else float('inf'))

            # ✅ 각 영역의 최소 거리 출력
            print("\n📊 Lidar Distance Readings:")
            for i, dist in enumerate(min_distances):
                print(f"  Zone {i} ({angle_ranges[i][0]}° ~ {angle_ranges[i][1]}°): {dist:.3f}m")

            # ✅ 0번과 7번 영역 거리 확인
            left_obstacle_distance = min_distances[0]  # -60° ~ -45°
            right_obstacle_distance = min_distances[7]  # 45° ~ 60°
            # ✅ PI 제어 적용 (0.5m 이내에서 작동)
            setpoint = 0.5  # 목표 거리 (0.5m)
            current_time = time.time()
            dt = current_time - previous_time

            if left_obstacle_distance < setpoint or right_obstacle_distance < setpoint:
                # ✅ 장애물 감지 시작
                pi_control_active = True  

                error = setpoint - min(left_obstacle_distance, right_obstacle_distance)  # ✅ 변경: setpoint - distance
                integral_error += error * dt  # 적분 항 누적

                # ✅ 적분 항 제한 (Windup 방지)
                integral_error = max(-0.1, min(0.1, integral_error))

                # ✅ 조향 방향 설정 (왼쪽 장애물이 가까우면 오른쪽으로 조향, 반대도 동일)
                if left_obstacle_distance < right_obstacle_distance:
                    steering_signal = int((Kp * error + Ki * integral_error) * 150)
                else:
                    steering_signal = int(-(Kp * error + Ki * integral_error) * 150)


                # ✅ 조향값 제한 (45 ~ 135도)
                final_steering = max(45, min(135, 90 + steering_signal))

                print(f"⚠️ 장애물 감지 → 조향 값: {final_steering}")
                ser.write(bytes([0, int(final_steering)]))  # 아두이노에 조향값 전송

            # ✅ 0.5m 이내일 경우 속도 조절 시작
            if left_obstacle_distance < setpoint or right_obstacle_distance < setpoint:
                if not speed_control_active:
                    print("⚠️ 0.5m 이내 접근 → 속도 조절 시작")
                    speed_control_active = True
                    speed_thread = threading.Thread(target=speed_control)
                    speed_thread.start()
            else:
                if speed_control_active:
                    print("✅ 0.5m 이상 거리 확보 → 속도 조절 중지")
                    speed_control_active = False

            # ✅ 장애물을 피한 후 0.5m 이상 거리 확보하면 (255,0) 전송
            if pi_control_active and left_obstacle_distance >= 0.5 and right_obstacle_distance >= 0.5:
                print("✅ 0.5m 이상 벗어남 → (255,0) 전송")
                ser.write(bytes([255, 0]))  # 장애물 피한 후 신호
                pi_control_active = False  # 다시 정상 상태로 전환
            previous_time = current_time  # 이전 시간 업데이트

        except json.JSONDecodeError:
            pass

        # ✅ 버퍼 초기화 (다음 데이터 수신을 위해)
        buffer = b""