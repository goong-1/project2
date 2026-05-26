#include <Wire.h>
#include <MPU6050_tockn.h>

// --- [핀 및 상수 설정] ---
const byte IN1 = 6; const byte IN2 = 5;
const byte IN3 = 10; const byte IN4 = 9;
const byte ENCODER_L = 2; const byte ENCODER_R = 3;

const float WHEEL_D = 6.5;
const int TICKS_REV = 40;

// 출력 범위 설정
const int MIN_PWM = 90;  // 최소 출력
const int MAX_PWM = 150; // 최대 출력
const byte MANUAL_BASE_SPD = 110;

MPU6050 mpu(Wire);

// PID 설정
float Kp = 25.0, Ki = 0.05, Kd = 1.0;
float err_int = 0, last_err = 0;

// 제어 변수
long targetTick = 0;
float targetYaw = 0;
bool isMovingAuto = false;  
bool isMovingManual = false;
bool isOriented = false;    
int manualDirection = 1;     // 1: 전진, -1: 후진, 2: 수동 좌회전(PID 없음), 3: 수동 우회전(PID 없음)

volatile long lTick = 0, rTick = 0;
volatile unsigned long lTime = 0, rTime = 0;
unsigned long pTime = 0;
unsigned long lastStatusTime = 0;

void cL() { unsigned long c = micros(); if (c - lTime > 500) { lTick++; lTime = c; } }
void cR() { unsigned long c = micros(); if (c - rTime > 500) { rTick++; rTime = c; } }

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5); // 시리얼 파싱 지연 방지를 위한 타임아웃 설정
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
    if (cmd == 'D') {
      float dist = Serial.parseFloat();
      if (Serial.read() == ',') {
        if (Serial.read() == 'A') {
          float angle_err = Serial.parseFloat();
          
          // 레이턴시 측정을 위한 T 파라미터 처리 (추가된 부분)
          if (Serial.read() == ',') {
            if (Serial.read() == 'T') {
              String timestamp = Serial.readStringUntil('\n');
              Serial.println(timestamp); // 받은 타임스탬프 즉시 에코
            }
          }
          
          updateTarget(dist, angle_err);
        }
      }
    } else {
      handleManualCommand(cmd);
    }
  }

  if (isMovingAuto || isMovingManual) {
    if (isMovingAuto && ((lTick + rTick) / 2 >= targetTick)) {
      stopAll();
      Serial.println("STATUS:ARRIVED");
    } else {
      // a, d로 인한 제자리 회전 시에도 이 함수 안에서 PID 우회 처리됨
      applyPIDDrive();
    }
  }

  if (millis() - lastStatusTime > 100) {
    lastStatusTime = millis();
    reportStatus();
  }
}

void updateTarget(float dist, float angle_err) {
  targetYaw = mpu.getAngleZ() + angle_err;
  long currentAvg = (lTick + rTick) / 2;
  long addedTick = (long)((dist / (WHEEL_D * 3.14159)) * TICKS_REV);
  targetTick = currentAvg + addedTick;
  
  if (abs(angle_err) > 5.0) {
    isOriented = false;
    err_int = 0;
  }
  
  isMovingAuto = true;
  isMovingManual = false;
}

void applyPIDDrive() {
  // --- [수정 핵심: 수동 좌회전(2) 또는 우회전(3)일 때 PID 연산 없이 즉시 정속 구동] ---
  if (isMovingManual && (manualDirection == 2 || manualDirection == 3)) {
    // 2(좌회전)이면 왼쪽 바퀴 후진(-), 오른쪽 바퀴 전진(+) / 3(우회전)이면 반대
    int leftSpd = (manualDirection == 2) ? -MANUAL_BASE_SPD : MANUAL_BASE_SPD;
    int rightSpd = (manualDirection == 2) ? MANUAL_BASE_SPD : -MANUAL_BASE_SPD;

    leftSpd = applyRange(leftSpd);
    rightSpd = applyRange(rightSpd);

    moveRaw(leftSpd, rightSpd);
    return; // 아래의 자이로 자조 및 PID 로직을 타지 않고 탈출합니다.
  }
  // -----------------------------------------------------------------------------

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
      base = 0;
      if (abs(err) > 2.5) {
        if (correction > 0 && correction < MIN_PWM) correction = MIN_PWM;
        else if (correction < 0 && correction > -MIN_PWM) correction = -MIN_PWM;
      }
      if (abs(err) < 2.5) {
        isOriented = true;
        err_int = 0;
      }
    } else {
      base = 100;
    }
  } else {
    base = MANUAL_BASE_SPD;
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

void reportStatus() {
  if (!isMovingAuto && !isMovingManual) Serial.println("STATUS:IDLE");
  else if (isMovingAuto) {
    if (!isOriented) Serial.println("STATUS:TURNING");
    else Serial.println("STATUS:DRIVING");
  }
}

void moveRaw(int left, int right) {
  if (left >= 0) { analogWrite(IN1, left); digitalWrite(IN2, LOW); }
  else { digitalWrite(IN1, LOW); analogWrite(IN2, abs(left)); }
  if (right >= 0) { analogWrite(IN3, right); digitalWrite(IN4, LOW); }
  else { digitalWrite(IN3, LOW); analogWrite(IN4, abs(right)); }
}

void stopAll() {
  isMovingAuto = false; isMovingManual = false;
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  err_int = 0;
}

// [수정된 부분] a, d 명령 시 PID를 우회하도록 manualDirection 변수에 독립 플래그 부여
void handleManualCommand(char cmd) {
  if (cmd == 'w' || cmd == 's' || cmd == 'a' || cmd == 'd') {
    lTick = 0; rTick = 0; err_int = 0; last_err = 0;
    pTime = millis();
    isOriented = true;
    isMovingManual = true;
    isMovingAuto = false;

    if (cmd == 'w' || cmd == 's') {
      targetYaw = mpu.getAngleZ(); // 현재 방향 유지하며 직진 보정 PID 활성화
      manualDirection = (cmd == 'w') ? 1 : -1;
    }
    else if (cmd == 'a' || cmd == 'd') {
      // 💡 원래 있던 고정 15도 추가 연산을 지우고, PID 우회용 상태 값으로 변경 (2: 좌회전, 3: 우회전)
      manualDirection = (cmd == 'a') ? 2 : 3; 
    }
  }
  else if (cmd == 'x' || cmd == ' ') {
    stopAll();
  }
}
