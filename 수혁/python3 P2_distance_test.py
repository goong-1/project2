import socket
import threading
import sys
import tty
import termios
import time

# ==========================================
# [설정] 할당받은 ESP32 IP 주소를 정확히 입력하세요
# ==========================================
ESP32_IP = '192.168.0.9' 
PORT = 8080

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2.0)
    sock.connect((ESP32_IP, PORT))
    print(f"[{ESP32_IP}:{PORT}] 소켓 통신 시스템 활성화 성공!")
    sock.settimeout(None)
except Exception as e:
    print(f"ESP32 접속 에러. IP 주소를 확인하세요.\n상세: {e}")
    sys.exit()

active_command = 'x'
running = True

# 1. 수신 스레드 (엔코더 실시간 갱신 및 스크롤 출력)
def read_from_socket():
    global running
    while running:
        try:
            data = sock.recv(1024).decode('utf-8')
            if data:
                lines = data.split('\n')
                for line in lines:
                    if line.strip():
                        sys.stdout.write(f"\r[로봇 실시간 상태] {line.strip()}\n")
                        sys.stdout.flush()
            else:
                raise Exception("서버 점검 또는 연결 끊김.")
        except Exception as e:
            print(f"\n🚨 통신 장애 발생: {e}")
            running = False
            break

read_thread = threading.Thread(target=read_from_socket)
read_thread.daemon = True
read_thread.start()

# 2. 하트비트 송신 스레드
def send_heartbeat():
    global running
    while running:
        try:
            command_packet = f"<{active_command}>"
            sock.sendall(command_packet.encode())
        except:
            running = False
            break
        time.sleep(0.2)

heartbeat_thread = threading.Thread(target=send_heartbeat)
heartbeat_thread.daemon = True
heartbeat_thread.start()

# 3. 터미널 ECHO 및 버퍼 비활성화 제어
def get_char():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        new_settings = termios.tcgetattr(fd)
        new_settings[3] = new_settings[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(fd, termios.TCSANOW, new_settings)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

print("\n" + "="*40)
print(" 🚀 [363cm 자율 주행 모드] 통제 단말기 (WSL)")
print(" - [g] : 363cm 목표 주행 출발")
print(" - [x] : 긴급 정지 (E-STOP)")
print(" - [q] : 프로그램 종료")
print("="*40 + "\n")

try:
    while running:
        char = get_char().lower()
        if char == 'q':
            running = False
            print("\n프로그램을 종료합니다.")
            break
        elif char in ['g', 'x']:
            active_command = char

except KeyboardInterrupt:
    running = False
    print("\n강제 종료 처리됨.")
finally:
    try:
        sock.sendall(b"<x>")
    except:
        pass
    sock.close()
