#include <Wire.h>
#include <MPU6050_tockn.h>

// --- [핀 및 상수 설정] ---
const byte IN1 = 6; const byte IN2 = 5; 
const byte IN3 = 10; const byte IN4 = 9; 
const byte ENCODER_L = 2; const byte ENCODER_R = 3; 
const float WHEEL_D = 6.5; 
const int TICKS_REV = 40; 

// 출력 범위 설정
const int MIN_PWM = 90; 
const int MAX_PWM = 150; 
const byte MANUAL_BASE_SPD = 110;

MPU6050 mpu(Wire); 

// PID 설정
float Kp = 25.0, Ki = 0.05, Kd = 1.0; 
float err_int = 0, last_err = 0; 

// 제어 변수
float targetYaw = 0; 
bool isMovingAuto = false;  
bool isMovingManual = false; 
bool isOriented = false;
bool isTurnOnly = false; // 제자리 90도 회전 후 정지하기 위한 플래그
int manualDirection = 1; 

volatile long lTick = 0, rTick = 0; 
volatile unsigned long lTime = 0, rTime = 0; 
unsigned long pTime = 0; 

void cL() { unsigned long c = micros(); if (c - lTime > 500) { lTick++; lTime = c; } } 
void cR() { unsigned long c = micros(); if (c - rTime > 500) { rTick++; rTime = c; } } 

void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  mpu.begin(); 
  mpu.calcGyroOffsets(false); 
  
  pinMode(ENCODER_L, INPUT_PULLUP); 
  pinMode(ENCODER_R, INPUT_PULLUP); 
  attachInterrupt(0, cL, RISING);
  attachInterrupt(1, cR, RISING); 
  
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); 
  stopAll(); 
}

void loop() {
  mpu.update(); 

  if (Serial.available()) {
    char cmd = Serial.read(); 
    
    if (cmd == 'G') {
      // 직진 명령 (자이로 PID를 활용해 현재 방향을 꽉 잡고 전진)
      lTick = 0; rTick = 0; err_int = 0; last_err = 0; 
      pTime = millis(); 
      isOriented = true; 
      isTurnOnly = false;
      isMovingManual = true; 
      isMovingAuto = false; 
      targetYaw = mpu.getAngleZ(); // 직진 시 현재 방향 유지 
      manualDirection = 1; 
      Serial.println("STATUS: GO_STRAIGHT");
    } 
    else if (cmd == 'S') {
      // 정지 명령
      stopAll();
      Serial.println("STATUS: STOPPED");
    }
    else if (cmd == 'L') {
      // 90도 좌회전 (현재 각도 기준 +90도)
      updateTargetTurn(90.0);
      Serial.println("STATUS: TURN_LEFT_90");
    }
    else if (cmd == 'R') {
      // 90도 우회전 (현재 각도 기준 -90도)
      updateTargetTurn(-90.0);
      Serial.println("STATUS: TURN_RIGHT_90");
    }
  }

  if (isMovingAuto || isMovingManual) {
    applyPIDDrive(); 
  }
}

// 정확한 제자리 회전을 위한 목표치 설정 함수
void updateTargetTurn(float angle_err) {
  targetYaw = mpu.getAngleZ() + angle_err;
  isOriented = false; 
  isTurnOnly = true;  // 회전 완료 후 즉시 멈춤을 지시
  err_int = 0; 
  
  isMovingAuto = true; 
  isMovingManual = false; 
}

void applyPIDDrive() {
  unsigned long c = millis(); 
  float dt = (c - pTime) / 1000.0; 
  if (dt <= 0) return; 
  pTime = c; 

  float currentYaw = mpu.getAngleZ(); 
  float err = targetYaw - currentYaw; 
  
  if (err > 180) err -= 360; 
  else if (err < -180) err += 360; 
  
  if (abs(err) < 10.0) { 
    err_int += err * dt; 
    err_int = constrain(err_int, -50, 50); 
  } else {
    err_int = 0; 
  }

  float correction = (Kp * err) + (Ki * err_int) + (Kd * (err - last_err) / dt); 
  last_err = err; 

  int base = 0; 
  if (isMovingAuto) { 
    if (!isOriented) {
      base = 0; // 제자리 회전 모드 
      if (abs(err) > 2.5) {  // 2.5도 오차 밖일 때
        if (correction > 0 && correction < MIN_PWM) correction = MIN_PWM;
        else if (correction < 0 && correction > -MIN_PWM) correction = -MIN_PWM; 
      }
      if (abs(err) <= 2.5) {  // 목표 각도 도달
        isOriented = true; 
        err_int = 0; 
        if (isTurnOnly) {
          stopAll(); // 90도 회전용인 경우 회전이 끝나면 정지
          return;
        }
      }
    } else {
      base = 100; // 자동 직진용 기본 베이스 
    }
  } else {
    base = MANUAL_BASE_SPD; // 수동 제어(직진)용 
  }

  base *= manualDirection; 
  int leftSpd = base + (int)correction; 
  int rightSpd = base - (int)correction; 

  leftSpd = applyRange(leftSpd); 
  rightSpd = applyRange(rightSpd); 

  moveRaw(leftSpd, rightSpd); 
}

int applyRange(int speed) {
  if (speed == 0) return 0; 
  bool positive = speed > 0;
  int absSpeed = abs(speed); 
  if (absSpeed < MIN_PWM) absSpeed = MIN_PWM; 
  if (absSpeed > MAX_PWM) absSpeed = MAX_PWM;
  return positive ? absSpeed : -absSpeed; 
}

void moveRaw(int left, int right) {
  if (left >= 0) { analogWrite(IN1, left); digitalWrite(IN2, LOW); } 
  else { digitalWrite(IN1, LOW); analogWrite(IN2, abs(left)); } 
  if (right >= 0) { analogWrite(IN3, right); digitalWrite(IN4, LOW); } 
  else { digitalWrite(IN3, LOW); analogWrite(IN4, abs(right)); } 
}

void stopAll() {
  isMovingAuto = false; isMovingManual = false; 
  isTurnOnly = false;
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); 
  err_int = 0; 
  analogWrite(IN1, 0); analogWrite(IN2, 0);
  analogWrite(IN3, 0); analogWrite(IN4, 0);
}