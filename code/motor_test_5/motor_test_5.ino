#include <Servo.h>

// 핀 및 변수 설정
Servo esc;
int escPin = 9;           // ESC 신호 핀 (D9)
int switchPin1 = 2;       // 스위치 1 (속도 감소 및 정지, 인터럽트 핀 D2)
int switchPin2 = 3;       // 스위치 2 (속도 증가 및 복귀, 인터럽트 핀 D3)
volatile int throttle = 90;  // 초기 스로틀 값 (중간, 0~180)
volatile int previousThrottle = 90;  // 정지 전 스로틀 값 저장
volatile bool isStopped = false;  // 정지 상태 플래그
volatile bool decreaseSpeed = false;  // 속도 감소 플래그 (인터럽트용)
volatile bool increaseSpeed = false;  // 속도 증가 플래그 (인터럽트용)

// 타이머 인터럽트 플래그
volatile bool timerFlag = false;

void setup() {
  // ESC 초기화
  esc.attach(escPin, 1000, 2000);  // PWM 범위 설정 (1000~2000μs)
  esc.write(throttle);             // 초기 중간 속도 (100% 반영, 10ms 이내)
  delay(5000);                     // ESC 초기화 대기 (5초)
  Serial.begin(9600);              // 디버깅용 시리얼 시작

  // 스위치 핀 설정 (풀다운 저항 사용)
  pinMode(switchPin1, INPUT);
  pinMode(switchPin2, INPUT);

  // 인터럽트 설정 (스위치 1, 2에 대해 RISING 트리거)
  attachInterrupt(digitalPinToInterrupt(switchPin1), decreaseSpeedISR, RISING);  // 스위치 1 인터럽트
  attachInterrupt(digitalPinToInterrupt(switchPin2), increaseSpeedISR, RISING);  // 스위치 2 인터럽트

  // 타이머 인터럽트 설정 (Timer1, 100ms 주기)
  cli();  // 인터럽트 비활성화
  TCCR1A = 0;  // Timer1 레지스터 초기화
  TCCR1B = 0;
  TCNT1 = 0;   // 타이머 카운터 초기화
  OCR1A = 15624;  // 100ms 주기 (16MHz, 1024 분주율: (16,000,000 / 1024) * 0.1 = 15625)
  TCCR1B |= (1 << WGM12);  // CTC 모드
  TCCR1B |= (1 << CS12) | (1 << CS10);  // 1024 분주율
  TIMSK1 |= (1 << OCIE1A);  // 타이머 비교 인터럽트 활성화
  sei();  // 인터럽트 활성화
}

// 스위치 1 인터럽트 서비스 루틴 (속도 감소 트리거)
void decreaseSpeedISR() {
  if (!isStopped) {
    decreaseSpeed = true;
  }
}

// 스위치 2 인터럽트 서비스 루틴 (속도 증가 트리거)
void increaseSpeedISR() {
  if (isStopped) {
    increaseSpeed = true;
  }
}

// 타이머 인터럽트 서비스 루틴 (100ms 주기)
ISR(TIMER1_COMPA_vect) {
  timerFlag = true;  // 스로틀 조정 트리거
}

void loop() {
  // 속도 감소 처리 (인터럽트로 트리거)
  if (decreaseSpeed) {
    previousThrottle = throttle;  // 현재 스로틀 값 저장
    while (throttle > 0) {
      throttle -= 5;  // 5 단위로 감소 (10ms 이내 반영 가정)
      throttle = max(throttle, 0);  // 0 이하로 안 떨어지게
      esc.write(throttle);  // ESC에 스로틀 값 전달
      delay(100);  // 감소 속도 조절 (100ms마다 감소)
      if (digitalRead(switchPin1) == LOW) break;  // 스위치 떼면 중단
    }
    if (throttle == 0) isStopped = true;  // 정지 상태로
    decreaseSpeed = false;  // 플래그 리셋
  }

  // 속도 증가 처리 (인터럽트로 트리거)
  if (increaseSpeed) {
    while (throttle < previousThrottle) {
      throttle += 5;  // 5 단위로 증가 (10ms 이내 반영 가정)
      throttle = min(throttle, previousThrottle);  // 이전 속도 초과 안 함
      esc.write(throttle);  // ESC에 스로틀 값 전달
      delay(100);  // 증가 속도 조절
      if (digitalRead(switchPin2) == LOW) break;  // 스위치 떼면 중단
    }
    if (throttle == previousThrottle) isStopped = false;  // 정상 상태로
    increaseSpeed = false;  // 플래그 리셋
  }

  // 타이머 인터럽트로 주기적 상태 유지 (100ms)
  if (timerFlag) {
    esc.write(throttle);  // 현재 스로틀 값 유지 (100% 반영, 10ms 이내)
    Serial.print("Throttle: "); Serial.println(throttle);  // 디버깅 출력
    timerFlag = false;  // 플래그 리셋
  }
}