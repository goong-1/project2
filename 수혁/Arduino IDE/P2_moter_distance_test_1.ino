#include <Arduino.h>
#include <WiFi.h>
#include <cmath>

const char* ssid = "asia-edu_2G";
const char* password = "12345678";

WiFiServer server(8080); 

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

const uint16_t PWMA = 26; const uint16_t AIN1 = 22; const uint16_t AIN2 = 23; 
const uint16_t PWMB = 25; const uint16_t BIN1 = 21; const uint16_t BIN2 = 17; 

const int freq = 20000;      
const int resolution = 8;    

const float MOTOR_A_WEIGHT = 1.01; 
const float MOTOR_B_WEIGHT = 0.99; 

// ==========================================
// [거리 영점 조절 완료] 193cm 주행 실측 데이터 반영
// ==========================================
const float TARGET_DISTANCE_CM = 363.0;
const float TICKS_PER_CM = 379.27; // 73,200틱 / 193cm 기반 실측 상수 (A모터 누락 상태 기준)
const long TARGET_TICKS = (long)(TARGET_DISTANCE_CM * TICKS_PER_CM); // 약 137,675 틱

String current_command = "x"; 
String inputBuffer = "";      
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 500; 

bool is_target_mode = false;
bool target_reached = false;

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
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.print("\n>> ESP32 IP 주소: "); Serial.println(WiFi.localIP()); 

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
  } else if (balanced_speed > 0) {
    digitalWrite(in1Pin, LOW); digitalWrite(in2Pin, HIGH); ledcWrite(pwmPin, min(balanced_speed, 255));
    if (motor == 'A') current_dir_A = 1; else current_dir_B = 1;
  } else {
    digitalWrite(in1Pin, HIGH); digitalWrite(in2Pin, LOW); ledcWrite(pwmPin, min(abs(balanced_speed), 255));
    if (motor == 'A') current_dir_A = -1; else current_dir_B = -1;
  }
}

void loop() {
  WiFiClient client = server.available(); 

  if (client) {
    inputBuffer = "";
    lastCommandTime = millis();

    while (client.connected()) {
      while (client.available() > 0) {
        char c = client.read();
        if (c == '<') {
          inputBuffer = ""; 
        } else if (c == '>') {
          if (inputBuffer == "g") {
            lastCommandTime = millis(); 
            if (!is_target_mode) {
              is_target_mode = true;
              target_reached = false;
              current_command = "g";
              noInterrupts(); A_wheel_pulse_count = 0; B_wheel_pulse_count = 0; interrupts();
              controlMotor('A', 127); controlMotor('B', 127); 
            }
          } 
          else if (inputBuffer == "x") {
            lastCommandTime = millis();
            is_target_mode = false;
            target_reached = false;
            current_command = "x";
            controlMotor('A', 0); controlMotor('B', 0); 
          }
        } else {
          inputBuffer += c;
        }
      }

      if (is_target_mode && !target_reached) {
        long avg_count = (abs(A_wheel_pulse_count) + abs(B_wheel_pulse_count)) / 2;
        if (avg_count >= TARGET_TICKS) {
          controlMotor('A', 0); controlMotor('B', 0); 
          target_reached = true;
          current_command = "ARRIVED"; 
        }
      }

      if (millis() - lastCommandTime > TIMEOUT_MS) {
        controlMotor('A', 0); controlMotor('B', 0);
        current_command = "TIMEOUT_STOP";
        is_target_mode = false;
        target_reached = false;
      }

      client.print("STATE: "); client.print(current_command);
      client.print(" | Count A: "); client.print(A_wheel_pulse_count);
      client.print(" | Target: "); client.print(TARGET_TICKS);
      client.print(" | Count B: "); client.println(B_wheel_pulse_count);
      
      delay(100);
    }
    
    is_target_mode = false; target_reached = false;
    controlMotor('A', 0); controlMotor('B', 0);
  }
}