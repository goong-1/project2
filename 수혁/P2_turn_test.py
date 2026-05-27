import socket
import threading
import sys
import tty
import termios
import time

ESP32_IP = '192.168.0.9' 
PORT = 8080

try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2.0)
    sock.connect((ESP32_IP, PORT))
    print(f"[{ESP32_IP}:{PORT}] 제자리 회전 제어 세션 활성화 성공!")
    sock.settimeout(None)
except Exception as e:
    print(f"ESP32 접속 에러. IP 주소를 확인하세요.\n상세: {e}")
    sys.exit()

active_command = 'x'
running = True

def read_from_socket():
    global running
    while running:
        try:
            data = sock.recv(1024).decode('utf-8')
            if data:
                lines = data.split('\n')
                for line in lines:
                    if line.strip() and "Yaw" in line:
                        sys.stdout.write(f"\r[실시간 데이터] {line.strip()}                    ")
                        sys.stdout.flush()
            else:
                raise Exception("서버 연결 끊김.")
        except Exception:
            running = False
            break

read_thread = threading.Thread(target=read_from_socket)
read_thread.daemon = True
read_thread.start()

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

print("\n" + "="*50)
print(" 🔄 [자이로 실시간 피드백 정밀 회전 단말기]")
print(" - [1] : 시계 방향으로 90도 회전")
print(" - [2] : 시계 방향으로 180도 회전")
print(" - [3] : 반시계 방향으로 90도 회전 (-90도)")
print(" - [x] : 긴급 제동 (E-STOP)")
print(" - [q] : 프로그램 종료")
print("="*50 + "\n")

try:
    while running:
        char = get_char().lower()
        if char == 'q':
            running = False
            print("\n프로그램을 안전하게 종료합니다.")
            break
        elif char == 'x':
            active_command = "x"
            print("\n>> [EMERGENCY] 즉시 정지 명령 송신.")
        elif char == '1':
            active_command = "t,90"
            print("\n>> 시계 방향 90도 타겟 바인딩.")
        elif char == '2':
            active_command = "t,180"
            print("\n>> 시계 방향 180도 타겟 바인딩.")
        elif char == '3':
            active_command = "t,-90"
            print("\n>> 반시계 방향 90도 타겟 바인딩.")

except KeyboardInterrupt:
    running = False
finally:
    try: sock.sendall(b"<x>")
    except: pass
    sock.close()