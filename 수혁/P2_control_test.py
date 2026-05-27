import socket
import threading
import sys
import time

# ==========================================
# [설정] ESP32의 고정 IP 주소와 포트
# ==========================================
ESP32_IP = '192.168.0.9'
PORT = 8080

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2.0)
    sock.connect((ESP32_IP, PORT))
    print(f"✅ [{ESP32_IP}:{PORT}] 실전 자율주행 무선 세션 활성화 성공!")
    sock.settimeout(None)
except Exception as e:
    print(f"❌ 접속 에러. 로봇의 전원과 Wi-Fi 연결을 확인하세요.\n상세: {e}")
    sys.exit()

# 초기 상태는 '정지(s)'로 설정
active_command = 's'
running = True

# 1. ESP32 피드백 실시간 수신 스레드
def read_from_socket():
    global running
    while running:
        try:
            data = sock.recv(1024).decode('utf-8')
            if data:
                lines = data.split('\n')
                for line in lines:
                    if line.strip():
                        # 화면 밀림 방지를 위해 캐리지 리턴(\r) 사용
                        sys.stdout.write(f"\r[로봇 피드백] {line.strip()}                                ")
                        sys.stdout.flush()
            else:
                raise Exception("서버 연결 끊김.")
        except Exception:
            running = False
            break

read_thread = threading.Thread(target=read_from_socket)
read_thread.daemon = True
read_thread.start()

# 2. 하트비트 스레드 (통신 유지 및 세이프가드 방어)
# ESP32의 2초 타임아웃을 막기 위해 0.2초마다 현재 active_command를 계속 쏴줍니다.
def send_heartbeat():
    global running
    while running:
        try:
            # 꺾쇠(<>)로 감싸서 패킷 전송
            command_packet = f"<{active_command}>"
            sock.sendall(command_packet.encode())
        except:
            running = False
            break
        time.sleep(0.2)

heartbeat_thread = threading.Thread(target=send_heartbeat)
heartbeat_thread.daemon = True
heartbeat_thread.start()

print("\n" + "="*60)
print(" 🚀 [RobotCore 실전 자율주행 통제 단말기]")
print(" - g [속도] : 지정한 속도(0~255)로 직진 주행 (예: g 150)")
print(" - t [각도] : 지정한 각도로 제자리 회전 (예: t -90)")
print(" - s        : 즉시 긴급 정지 (E-STOP)")
print(" - q        : 프로그램 안전 종료")
print("="*60 + "\n")

try:
    while running:
        time.sleep(0.05)
        user_input = input("\n>> 명령 입력: ").strip().lower()
        
        if not user_input:
            continue
            
        if user_input == 'q':
            running = False
            print("\n프로그램을 안전하게 종료합니다.")
            break
            
        elif user_input == 's':
            active_command = "s"
            print(">> [명령 갱신] 긴급 정지 패킷 송신 중")
            
        elif user_input.startswith('g '):
            try:
                # 속도는 정수형으로 파싱
                val = int(user_input.split()[1])
                active_command = f"g,{val}"
                print(f">> [명령 갱신] 속도 {val} 직진 주행 기동")
            except (IndexError, ValueError):
                print("❌ 속도 값을 정확히 입력하세요. (예: g 150)")
                
        elif user_input.startswith('t '):
            try:
                # 각도는 소수점 입력이 가능하므로 실수형으로 파싱
                val = float(user_input.split()[1])
                active_command = f"t,{val}"
                print(f">> [명령 갱신] 목표 각도 {val}도 제자리 회전 기동")
            except (IndexError, ValueError):
                print("❌ 각도 값을 정확히 입력하세요. (예: t -90)")
        else:
            print("❌ 지원하지 않는 명령어입니다. (g [속도] / t [각도] / s / q 중 입력)")

except KeyboardInterrupt:
    running = False
finally:
    try:
        # 프로그램 종료 전 로봇의 폭주를 막기 위해 확실하게 정지 패킷 전송
        sock.sendall(b"<s>")
    except:
        pass
    sock.close()
    print("\n무선 연결이 안전하게 해제되었습니다.")