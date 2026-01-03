#include <Servo.h>

// 핀 및 변수 설정
Servo esc;
int escPin = 9;           // ESC 신호 핀 (D9)
int throttle = 0;         // ESC 스로틀 값 (0~180)

void setup() {
  // ESC 초기화
  esc.attach(escPin, 1000, 2000);  // PWM 범위 설정 (1000~2000μs)
  esc.write(0);                    // 초기 정지 (스로틀 0)
  delay(5000);                     // 전원 연결 후 5초 대기 (ESC 초기화)
  Serial.begin(9600);              // 디버깅용 시리얼 시작
}

void loop() {
  // 10초 작동 (최대 속도, 스로틀 180)
  throttle = 180;                  // 최대 속도 (100% 반영, 10ms 이내)
  esc.write(throttle);             // ESC에 스로틀 값 전달
  Serial.println("Motor ON - Max Speed (Throttle 180)");
  delay(10000);                    // 10초 작동

  // 20초 휴식
  throttle = 0;                    // 정지 (스로틀 0)
  esc.write(throttle);             // ESC에 정지 신호 전달
  Serial.println("Motor OFF - 20s Rest");
  delay(20000);                    // 20초 휴식
}