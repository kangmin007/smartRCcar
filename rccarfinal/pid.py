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
ser = serial.Serial('/dev/ttyAMA3', 9600)

# ✅ PID 제어 변수
Kp = 2.5
Ki = 0.6
Kd = 1.2
integral_error = 0
previous_error = 0
current_time = time.time()
previous_time = time.time()
previous_steering = 90  

# ✅ 속도 조절 및 조향 관련 변수
speed_control_active = False
speed_thread = None
buffer = b""
last_steering_time = 0  
steer_cooldown = 0.5  

# ✅ 현재 상태 (BLUETOOTH, SPEED_CONTROL, PID_CONTROL)
current_state = "BLUETOOTH"  
previous_state = None  # 이전 상태 저장
# ✅ 속도 조절 쓰레드 함수
def speed_control():
    while speed_control_active:
        ser.write(bytes([1, 1]))  
        print("🚀 속도 감소: (1,1) 전송")
        time.sleep(0.3)

        if speed_control_active:
            ser.write(bytes([1, 0]))  
            print("🚀 속도 유지: (1,0) 전송")
            time.sleep(0.3)

# ✅ 메인 루프
while True:
    data, addr = sock.recvfrom(8192)
    buffer += data

    if b'END' in buffer:
        buffer = buffer.replace(b'END', b'')

        try:
            lidar_data = json.loads(buffer.decode())

            # ✅ 각 구간별 최단 거리 계산
            angle_ranges = [
                (-60, -45), (-45, -30), (-30, -15), (-15, 0),
                (0, 15), (15, 30), (30, 45), (45, 60)
            ]
            min_distances = [
                min([point["distance"] for point in lidar_data
                     if angle_range[0] <= point["angle"] <= angle_range[1] and point["distance"] > 0], 
                    default=float('inf'))
                for angle_range in angle_ranges
            ]

            # ✅ 8개 구간 거리 출력
            for i, dist in enumerate(min_distances):
                print(f"  Zone {i} ({angle_ranges[i][0]}° ~ {angle_ranges[i][1]}°): {dist:.3f}m")
            # ✅ 장애물 감지 상태 업데이트
            front_obstacle_detected = any(dist < 0.8 for dist in min_distances[1:6])
            left_obstacle_distance = min_distances[0]
            right_obstacle_distance = min_distances[7]
            side_safe = left_obstacle_distance > 0.6 and right_obstacle_distance > 0.6
            side_obstacle_detected = left_obstacle_distance < 0.5 or right_obstacle_distance < 0.5

            # ✅ 현재 상태에 따른 동작 결정
            if current_state == "BLUETOOTH":
                if side_obstacle_detected:
                    print("⚠️ 측면 장애물 감지 → PID 제어 진입")
                    current_state = "PID_CONTROL"
                elif front_obstacle_detected and not side_obstacle_detected:
                    print("⚠️ 전방 장애물 감지 → 속도 제어 활성화 & 선제적 조향")
                    current_state = "SPEED_CONTROL"
            elif current_state == "SPEED_CONTROL":
                if side_obstacle_detected:
                    print("⚠️ 측면 장애물 감지 → PID 제어 진입")
                    current_state = "PID_CONTROL"
                elif not front_obstacle_detected:
                    print("✅ 전방 장애물 없음 → 블루투스 제어 전환")
                    current_state = "BLUETOOTH"

            elif current_state == "PID_CONTROL":
                if side_safe:
                    print("✅ 측면 장애물 벗어남 → 블루투스 or 속도 제어 결정")

                    # ✅ 측면 장애물 벗어났을 때 즉시 다음 상태 결정
                    if front_obstacle_detected:
                        print("⚠️ 전방 장애물 감지 → 속도 제어 활성화 & 선제적 조향")
                        current_state = "SPEED_CONTROL"
                    else:
                        print("✅ 장애물 없음 → 블루투스 제어 전환")
                        current_state = "BLUETOOTH"

            # ✅ 상태에 따른 시리얼 통신
            if current_state == "BLUETOOTH":
                if previous_state != "BLUETOOTH":  # 블루투스 상태로 처음 전환될 때만 전송
                    print("✅ 블루투스 제어 전환 → (255,0) 전송")
                    ser.write(bytes([255, 0]))  # ✅ 블루투스 신호 전송
                    previous_state = "BLUETOOTH"
                if speed_control_active:
                    print("🛑 블루투스 모드 전환 → speed_control 스레드 종료")
                    speed_control_active = False  # ✅ 스레드 종료 신호

                    if speed_thread is not None and speed_thread.is_alive():
                        speed_thread.join()  # ✅ 실행 중이면 종료 대기

            elif current_state == "SPEED_CONTROL":
                if previous_state != "SPEED_CONTROL":
                    print("⚠️ 속도 제어 및 선제적 조향 실행")
                    previous_state = "SPEED_CONTROL"
                if not speed_control_active:
                    print("⚠️ 속도 제어 및 선제적 조향 실행")

                    # ✅ 속도 제어 활성화
                    speed_control_active = True
                    if speed_thread is None or not speed_thread.is_alive():
                        speed_thread = threading.Thread(target=speed_control)
                        speed_thread.start()

                # ✅ 선제적 조향 결정
                left_sum = sum(dist for dist in min_distances[0:3] if dist != float('inf'))
                right_sum = sum(dist for dist in min_distances[4:7] if dist != float('inf'))

                if left_sum == 0: left_sum = 10
                if right_sum == 0: right_sum = 10

                if left_sum > right_sum:
                    steer_direction = "🔄 왼쪽 회피 (스티어링 +30)"
                    pre_steering = 120
                else:
                    steer_direction = "🔄 오른쪽 회피 (스티어링 -30)"
                    pre_steering = 60

                print(f"⚠️ 선제적 조향 실행 → {steer_direction}")
                ser.write(bytes([0, pre_steering]))
                previous_steering = pre_steering
            elif current_state == "PID_CONTROL":
                if previous_state != "PID_CONTROL":
                    print("⚠️ PID 제어 시작")
                    previous_state = "PID_CONTROL"
                setpoint = 0.5
                current_time = time.time()
                dt = current_time - previous_time
                error = setpoint - min(left_obstacle_distance, right_obstacle_distance)

                integral_error += error * dt
                integral_error = max(-0.2, min(0.2, integral_error))
                derivative_error = (error - previous_error) / dt if dt > 0 else 0
                previous_error = error

                steering_signal = int((Kp * error + Ki * integral_error + Kd * derivative_error) * 100)
                final_steering = max(55, min(125, previous_steering + (-steering_signal if left_obstacle_distance < right_>

                print(f"⚠️ PID 제어 → 조향 값: {final_steering}")
                ser.write(bytes([0, int(final_steering)]))
                previous_steering = final_steering  

            previous_time = current_time

        except json.JSONDecodeError:
            pass

        buffer = b""