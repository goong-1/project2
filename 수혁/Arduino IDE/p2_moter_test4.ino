//"기존 명령과 완전히 다른 새로운 명령이 들어왔을 때만 카운트를 0으로 초기화"

#include <Arduino.h>
#include <cmath>

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

const uint16_t PWMA = 25; 
const uint16_t AIN1 = 21; 
const uint16_t AIN2 = 17; 
const uint16_t PWMB = 26; 
const uint16_t BIN1 = 22; 
const uint16_t BIN2 = 23; 

const int freq = 20000;      
const int resolution = 8;    

const float MOTOR_A_WEIGHT = 1.00; 
const float MOTOR_B_WEIGHT = 1.00; 

String current_command = "x"; 
String inputBuffer = "";      

unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 500; 

void setup() {
  Serial.begin(115200);
  while(!Serial) {}

  pinMode(AENCA, INPUT_PULLUP);
  pinMode(AENCB, INPUT_PULLUP);
  pinMode(BENCA, INPUT_PULLUP);
  pinMode(BENCB, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(AENCB), A_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(BENCB), B_wheel_pulse, RISING);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  ledcAttach(PWMA, freq, resolution);
  ledcAttach(PWMB, freq, resolution);

  lastCommandTime = millis();
}

void controlMotor(char motor, int speed) {
  uint16_t pwmPin = (motor == 'A') ? PWMA : PWMB;
  uint16_t in1Pin = (motor == 'A') ? AIN1 : BIN1;
  uint16_t in2Pin = (motor == 'A') ? AIN2 : BIN2;
  
  float weight = (motor == 'A') ? MOTOR_A_WEIGHT : MOTOR_B_WEIGHT;
  int balanced_speed = (int)(speed * weight);

  if (balanced_speed == 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    ledcWrite(pwmPin, 0);
  } 
  else if (balanced_speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    ledcWrite(pwmPin, min(balanced_speed, 255));
    if (motor == 'A') current_dir_A = 1; else current_dir_B = 1;
  } 
  else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    ledcWrite(pwmPin, min(abs(balanced_speed), 255));
    if (motor == 'A') current_dir_A = -1; else current_dir_B = -1;
  }
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '<') {
      inputBuffer = ""; 
    } else if (c == '>') {
      if (inputBuffer == "w" || inputBuffer == "s" || inputBuffer == "x") {
        
        // 하트비트가 들어오면 무조건 시간은 갱신 (안전장치용)
        lastCommandTime = millis();
        
        // [핵심 수정] 이전 명령과 다를 때(바뀔 때)만 카운트 초기화 및 동작 수행
        if (current_command != inputBuffer) {
          current_command = inputBuffer;
          
          noInterrupts();
          A_wheel_pulse_count = 0;
          B_wheel_pulse_count = 0;
          interrupts();

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

  // 0.5초 동안 아무 명령이 없으면 강제 브레이크
  if (millis() - lastCommandTime > TIMEOUT_MS) {
    if (current_command != "x") {
      current_command = "x";
      controlMotor('A', 0);
      controlMotor('B', 0);
    }
  }

  Serial.print("CMD: "); Serial.print(current_command);
  Serial.print(" | Count A: "); Serial.print(A_wheel_pulse_count);
  Serial.print(" | Count B: "); Serial.println(B_wheel_pulse_count);
  
  delay(100);
}