#include <Servo.h>

// 핀 및 변수 설정
Servo esc;
int escPin = 9;           // ESC 신호 핀
int throttle = 0;         // ESC 스로틀 값

void setup() {
  esc.attach(escPin, 1000, 2000);  // PWM 범위 설정
  esc.write(0);                    // 초기 정지
  delay(5000);                     // 전원 연결 후 5초 대기
  Serial.begin(9600);              // 디버깅용 시리얼
}

void loop() {
  // 30초 작동 (최대 속도)
  throttle = 180;                  // 최대 속도
  esc.write(throttle);
  Serial.println("Motor ON - Max Speed");
  delay(30000);                    // 30초 작동

  // 10초 휴식
  throttle = 0;                    // 정지
  esc.write(throttle);
  Serial.println("Motor OFF - 10s Rest");
  delay(10000);                    // 10초 휴식
}