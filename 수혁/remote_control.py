import serial
import threading
import sys
import tty
import termios
import time

SERIAL_PORT = '/dev/ttyS0' 
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"[{SERIAL_PORT}] 시리얼 포트 연결 성공!")
except Exception as e:
    print(f"시리얼 포트 열기 실패.\n에러: {e}")
    sys.exit()

# 현재 로봇이 수행해야 할 명령 (초기값: 정지)
active_command = 'x'
running = True

# ---------------------------------------------------------
# 1. 수신 스레드: ESP32 데이터를 읽어와서 예쁘게 출력
# ---------------------------------------------------------
def read_from_port():
    while running:
        try:
            if ser.in_waiting > 0:
                reading = ser.readline().decode('utf-8', errors='ignore').strip()
                if reading:
                    # 화면을 지우고 맨 왼쪽부터 깔끔하게 출력
                    sys.stdout.write(f"\r[ESP32] {reading}\n")
                    sys.stdout.flush()
        except Exception:
            break

read_thread = threading.Thread(target=read_from_port)
read_thread.daemon = True
read_thread.start()

# ---------------------------------------------------------
# 2. 하트비트 스레드: 현재 명령을 0.2초마다 알아서 전송
# ---------------------------------------------------------
def send_heartbeat():
    while running:
        try:
            command_packet = f"<{active_command}>"
            ser.write(command_packet.encode())
        except:
            break
        time.sleep(0.2) # 1초에 5번만 안전하게 전송 (버퍼 오버플로우 방지)

heartbeat_thread = threading.Thread(target=send_heartbeat)
heartbeat_thread.daemon = True
heartbeat_thread.start()

# ---------------------------------------------------------
# 3. 키보드 입력 처리 (ECHO Off 적용하여 화면 깨짐 방지)
# ---------------------------------------------------------
def get_char():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        # 사용자가 누른 키가 화면에 보이지 않도록 터미널 설정 변경
        new_settings = termios.tcgetattr(fd)
        new_settings[3] = new_settings[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(fd, termios.TCSANOW, new_settings)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# 메인 UI 출력
print("\n" + "="*40)
print(" 🚀 [버전 2.0] 원터치 하트비트 무선 조종기")
print(" - [w] : 전진 (한 번만 누르세요)")
print(" - [s] : 후진 (한 번만 누르세요)")
print(" - [x] : 정지 (브레이크)")
print(" - [q] : 프로그램 종료")
print("="*40 + "\n")

# 메인 루프: 키보드 입력 시 active_command 변수만 바꿔줍니다.
try:
    while True:
        char = get_char().lower()
        
        if char == 'q':
            running = False
            print("\n프로그램을 종료합니다.")
            break
        elif char in ['w', 's', 'x']:
            active_command = char # 하트비트 스레드가 이 값을 가져가서 보냄

except KeyboardInterrupt:
    running = False
    print("\n강제 종료됨.")
finally:
    # 프로그램 종료 시 안전을 위해 강제 정지 신호 한 번 발송
    try:
        ser.write(b"<x>")
    except:
        pass
    ser.close()
