#include <Arduino.h>
#include <WiFi.h>
#include <cmath>


const char* ssid = "asia-edu_2G";
const char* password = "12345678";

WiFiServer server(8080); // 통신을 위한 8080 포트 개방

// ==========================================
// [수정 완료] 하드웨어 좌우 배선 교차 해결을 위한 핀 리매핑
// ==========================================
const uint16_t AENCA = 35; 
const uint16_t AENCB = 34; 
const uint16_t BENCA = 27; 
const uint16_t BENCB = 16; 

volatile long A_wheel_pulse_count = 0;
volatile long B_wheel_pulse_count = 0;
volatile int current_dir_A = 1;
volatile int current_dir_B = 1;

void IRAM_ATTR A_wheel_pulse() { A_wheel_pulse_count += current_dir_A; }
void IRAM_ATTR B_wheel_pulse() { B_wheel_pulse_count += current_dir_B; }

// 기존의 B 모터 핀들을 A 모터 제어용으로 할당
const uint16_t PWMA = 26; 
const uint16_t AIN1 = 22; 
const uint16_t AIN2 = 23; 

// 기존의 A 모터 핀들을 B 모터 제어용으로 할당
const uint16_t PWMB = 25; 
const uint16_t BIN1 = 21; 
const uint16_t BIN2 = 17; 

const int freq = 20000;      
const int resolution = 8;    

// 핀 리매핑으로 꼬인 것을 풀었으므로 가중치 다시 1:1 순정화
const float MOTOR_A_WEIGHT = 1.01; 
const float MOTOR_B_WEIGHT = 0.99; 

String current_command = "x"; 
String inputBuffer = "";      
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 500; 

void setup() {
  Serial.begin(115200);
  
  pinMode(AENCA, INPUT_PULLUP); pinMode(AENCB, INPUT_PULLUP);
  pinMode(BENCA, INPUT_PULLUP); pinMode(BENCB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(AENCB), A_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(BENCB), B_wheel_pulse, RISING);
  
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  ledcAttach(PWMA, freq, resolution); ledcAttach(PWMB, freq, resolution);

  Serial.print("\nWi-Fi 연결 중: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\n[Wi-Fi 연결 성공!]");
  Serial.print(">> ESP32 IP 주소: ");
  Serial.println(WiFi.localIP()); 

  server.begin();
  lastCommandTime = millis();
}

void controlMotor(char motor, int speed) {
  uint16_t pwmPin = (motor == 'A') ? PWMA : PWMB;
  uint16_t in1Pin = (motor == 'A') ? AIN1 : BIN1;
  uint16_t in2Pin = (motor == 'A') ? AIN2 : BIN2;
  
  float weight = (motor == 'A') ? MOTOR_A_WEIGHT : MOTOR_B_WEIGHT;
  int balanced_speed = (int)(speed * weight);

  if (balanced_speed == 0) {
    digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, LOW); ledcWrite(pwmPin, 0);
  } 
  else if (balanced_speed > 0) {
    // [수정 완료] 전진 방향 극성 뒤집기 (LOW, HIGH 로 변경)
    digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, HIGH); ledcWrite(pwmPin, min(balanced_speed, 255));
    if (motor == 'A') current_dir_A = 1; else current_dir_B = 1;
  } 
  else {
    // [수정 완료] 후진 방향 극성 뒤집기 (HIGH, LOW 로 변경)
    digitalWrite(in1Pin, HIGH); digitalWrite(in2Pin, LOW); ledcWrite(pwmPin, min(abs(balanced_speed), 255));
    if (motor == 'A') current_dir_A = -1; else current_dir_B = -1;
  }
}

void loop() {
  WiFiClient client = server.available(); 

  if (client) {
    Serial.println("노트북이 접속했습니다!");
    inputBuffer = "";
    lastCommandTime = millis();

    while (client.connected()) {
      while (client.available() > 0) {
        char c = client.read();
        if (c == '<') {
          inputBuffer = ""; 
        } else if (c == '>') {
          if (inputBuffer == "w" || inputBuffer == "s" || inputBuffer == "x") {
            lastCommandTime = millis(); 
            
            if (current_command != inputBuffer) {
              current_command = inputBuffer;
              noInterrupts(); A_wheel_pulse_count = 0; B_wheel_pulse_count = 0; interrupts();

              if (current_command == "w") {
                controlMotor('A', 127); controlMotor('B', 127);
              } else if (current_command == "s") {
                controlMotor('A', -127); controlMotor('B', -127);
              } else if (current_command == "x") {
                controlMotor('A', 0); controlMotor('B', 0);
              }
            }
          }
        } else {
          inputBuffer += c;
        }
      }

      if (millis() - lastCommandTime > TIMEOUT_MS) {
        if (current_command != "x") {
          current_command = "x";
          controlMotor('A', 0); controlMotor('B', 0);
        }
      }

      client.print("CMD: "); client.print(current_command);
      client.print(" | Count A: "); client.print(A_wheel_pulse_count);
      client.print(" | Count B: "); client.println(B_wheel_pulse_count);
      
      delay(100);
    }
    
    Serial.println("노트북 접속 끊김. 강제 정지.");
    current_command = "x";
    controlMotor('A', 0); controlMotor('B', 0);
  }
}