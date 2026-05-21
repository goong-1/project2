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

int interval = 100; // 출력 주기 (100ms)

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

// ==========================================
// [수정] 배터리 충전 후를 위해 가중치를 다시 1:1 순정 상태로 초기화했습니다.
// ==========================================
const float MOTOR_A_WEIGHT = 1.00; 
const float MOTOR_B_WEIGHT = 1.00; 

void setup() {
  Serial.begin(115200);
  while(!Serial) {}
  Serial.println("--- 배터리 완충용 순정 밸런스(1.00 / 1.00) 세팅 ---");

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
    if (motor == 'A') current_dir_A = 1;
    else current_dir_B = 1;
  } 
  else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    ledcWrite(pwmPin, min(abs(balanced_speed), 255));
    if (motor == 'A') current_dir_A = -1;
    else current_dir_B = -1;
  }
}

void loop() {
  Serial.println("\n[주행] 양쪽 모터 정방향 주행");
  noInterrupts();
  A_wheel_pulse_count = 0;
  B_wheel_pulse_count = 0;
  interrupts();
  
  controlMotor('A', 127);
  controlMotor('B', 127);
  
  for (int i = 0; i < 20; i++) {
    Serial.print("Forward  -> Count A: "); Serial.print(A_wheel_pulse_count);
    Serial.print(" | Count B: "); Serial.println(B_wheel_pulse_count);
    delay(interval);
  }

  controlMotor('A', 0);
  controlMotor('B', 0);
  delay(1500);

  Serial.println("\n[주행] 양쪽 모터 역방향 주행");
  noInterrupts();
  A_wheel_pulse_count = 0;
  B_wheel_pulse_count = 0;
  interrupts();
  
  controlMotor('A', -127);
  controlMotor('B', -127);
  
  for (int i = 0; i < 20; i++) {
    Serial.print("Backward -> Count A: "); Serial.print(A_wheel_pulse_count);
    Serial.print(" | Count B: "); Serial.println(B_wheel_pulse_count);
    delay(interval);
  }

  controlMotor('A', 0);
  controlMotor('B', 0);
  delay(1500);
}