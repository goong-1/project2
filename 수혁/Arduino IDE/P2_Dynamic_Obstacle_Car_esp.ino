#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
unsigned long lastHeartbeatTime = 0;
bool wasConnected = false;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17); 
  SerialBT.begin("ESP32_RC_Car"); 
  Serial.println("블루투스 기기가 시작되었습니다.");
}

void loop() {
  bool isConnected = SerialBT.hasClient();

  // [신규] 블루투스가 연결되어 있다가 끊긴 순간 예외 처리
  if (wasConnected && !isConnected) {
    Serial.println("블루투스 연결 끊김! 아두이노에 정지 명령 전송");
    Serial2.print('S'); // 즉시 정지 신호 전송
  }
  wasConnected = isConnected;

  // 1. 스마트폰에서 명령을 받아 아두이노로 전달
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    if (cmd == 'G' || cmd == 'L' || cmd == 'R' || cmd == 'S') {
      Serial2.print(cmd);
    }
  }

  // [신규] 연결 상태 유지 중일 때, 200ms마다 아두이노에 하트비트('H') 전송
  if (isConnected && (millis() - lastHeartbeatTime > 200)) {
    Serial2.print('H'); 
    lastHeartbeatTime = millis();
  }

  // 2. 아두이노에서 오는 피드백 전달
  if (Serial2.available()) {
    char c = Serial2.read();
    if (isConnected) SerialBT.print(c);
  }
}