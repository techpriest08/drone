#include <Servo.h>
#include <Wire.h>

// MPU9250 I2C 주소
#define MPU9250_ADDRESS 0x68
#define ACCEL_ZOUT_H 0x3F  // Z축 가속도 레지스터

// 핀 및 변수 설정
Servo esc;
int escPin = 9;           // ESC 신호 핀 (D9)
int buttonPin = 2;        // 푸시 버튼 핀 (D2)
volatile int throttle = 0;         // ESC 스로틀 값 (0~180, 다이어그램의 J_p, J_ep)
float targetPosition = 2.0;        // 목표 위치 2m (다이어그램의 w*)
float currentPosition = 0.0;       // 현재 위치 (다이어그램의 w)
float velocity = 0.0;              // 현재 속도 (적분으로 계산)
float Kp = 30.0;                   // 비례 이득 (다이어그램의 Kp, 1kg 드론에 맞게 증가)
float Ki = 0.3;                    // 적분 이득 (다이어그램의 PI 블록, 진동 방지 조정)
float integral = 0.0;              // 적분 항 (다이어그램의 PI 적분 부분)
float accelZ = 0.0;                // Z축 가속도
long lastTime = 0;                 // 이전 시간
bool buttonPressed = false;        // 버튼 상태 플래그

void setup() {
  // ESC 초기화
  esc.attach(escPin, 1000, 2000);  // PWM 범위 설정 (1000~2000μs)
  esc.write(0);                    // 초기 정지 (스로틀 0)
  delay(5000);                     // ESC 초기화 대기 (5초)
  Serial.begin(9600);              // 디버깅용 시리얼 시작

  // 버튼 핀 설정 (내부 풀업 저항 사용)
  pinMode(buttonPin, INPUT_PULLUP);  // 내부 풀업 저항 활성화

  // MPU9250 초기화
  Wire.begin();
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x6B);  // PWR_MGMT_1 레지스터
  Wire.write(0x00);  // 슬립 모드 해제
  Wire.endTransmission(true);

  lastTime = millis();  // 초기 시간 설정
}

void loop() {
  // 버튼 상태 확인 (내부 풀업: 누르면 LOW, 안 누르면 HIGH)
  if (digitalRead(buttonPin) == LOW && !buttonPressed) {
    buttonPressed = true;

    // 5초 동안 스로틀 0에서 180으로 가속
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
      throttle = map(millis() - startTime, 0, 5000, 0, 180);  // 0~180 선형 증가
      esc.write(throttle);  // ESC에 스로틀 값 전달 (10ms 이내 반영 가정)
      delay(10);  // 부드러운 가속
    }
    throttle = 180;  // 최대 속도 유지
    esc.write(throttle);
  }

  // MPU9250에서 가속도 데이터 읽기
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(ACCEL_ZOUT_H);  // Z축 가속도 레지스터
  Wire.endTransmission(false);
  Wire.requestFrom(MPU9250_ADDRESS, 2, true);
  int16_t accelZRaw = (Wire.read() << 8) | Wire.read();  // Z축 가속도 원시 데이터
  accelZ = accelZRaw / 16384.0 * 9.81;  // m/s² 단위로 변환 (감도: ±2g)

  // 시간 간격 계산
  long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // 초 단위
  lastTime = currentTime;

  // 속도와 위치 계산 (적분)
  velocity += (accelZ - 9.81) * dt;  // 속도 = 가속도 적분 (중력 보정, m/s)
  currentPosition += velocity * dt;  // 위치 = 속도 적분 (m)

  // PI 제어 (다이어그램 기반)
  if (buttonPressed) {
    float error = targetPosition - currentPosition;  // 오차 (다이어그램의 e = w* - w)
    integral += error * dt;  // 적분 항 누적 (다이어그램의 PI 적분)
    float output = (Kp * error) + (Ki * integral);  // PI 출력 (다이어그램의 PI 블록)

    // 스로틀 값 조정 (0~180 범위 유지, 다이어그램의 J_p, J_ep)
    throttle = constrain(output, 0, 180);
    esc.write(throttle);  // ESC에 스로틀 값 전달 (10ms 이내 반영 가정)
  }

  // 디버깅 출력
  Serial.print("Position: "); Serial.print(currentPosition);
  Serial.print(" Throttle: "); Serial.println(throttle);

  delay(10);  // 제어 주기 (10ms)
}