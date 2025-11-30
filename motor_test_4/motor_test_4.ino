#include <Servo.h>

// 핀 및 변수 설정
Servo esc;
int escPin = 9;           // ESC 신호 핀 (D9)
int sensorPin = A0;       // 속도 센서 핀 (아날로그 입력, 다이어그램의 w 측정)
int switchPin1 = 2;       // 스위치 1 (속도 감소 및 정지, 인터럽트 핀 D2)
int switchPin2 = 3;       // 스위치 2 (속도 증가 및 복귀, 인터럽트 핀 D3)
volatile float setpoint = 1000.0;  // 목표 속도 (RPM, 다이어그램의 w*)
volatile float previousSetpoint = 1000.0;  // 정지 전 속도 저장
float Kp = 0.5;           // 비례 이득 (다이어그램의 Kp)
float Ki = 0.01;          // 적분 이득 (다이어그램의 PI 블록)
float prevError = 0.0;    // 이전 오차 (미분 항 제거, PI 제어로 수정)
float integral = 0.0;     // 적분 항 (다이어그램의 PI 적분 부분)
volatile int throttle = 0;         // ESC 스로틀 값 (0~180, 다이어그램의 J_p, J_ep)
volatile bool isStopped = false;   // 정지 상태 플래그
volatile bool decreaseSpeed = false;  // 속도 감소 플래그 (인터럽트용)
volatile bool increaseSpeed = false;  // 속도 증가 플래그 (인터럽트용)

// 타이머 인터럽트 플래그
volatile bool timerFlag = false;

void setup() {
  // ESC 초기화
  esc.attach(escPin, 1000, 2000);  // PWM 범위 설정 (1000~2000μs)
  esc.write(0);                    // 초기 정지 (스로틀 0)
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
  timerFlag = true;  // PID 계산 트리거
}

void loop() {
  // 속도 감소 처리 (인터럽트로 트리거)
  if (decreaseSpeed) {
    previousSetpoint = setpoint;  // 현재 setpoint 저장 (다이어그램의 w* 저장)
    while (setpoint > 0) {
      setpoint -= 50.0;  // 50 RPM씩 감소
      setpoint = max(setpoint, 0.0);  // 0 이하로 안 떨어지게
      delay(100);  // 감소 속도 조절 (100ms마다 감소)
      if (digitalRead(switchPin1) == LOW) break;  // 스위치 떼면 중단
    }
    if (setpoint == 0) isStopped = true;  // 정지 상태로
    decreaseSpeed = false;  // 플래그 리셋
  }

  // 속도 증가 처리 (인터럽트로 트리거)
  if (increaseSpeed) {
    while (setpoint < previousSetpoint) {
      setpoint += 50.0;  // 50 RPM씩 증가
      setpoint = min(setpoint, previousSetpoint);  // 이전 속도 초과 안 함
      delay(100);  // 증가 속도 조절
      if (digitalRead(switchPin2) == LOW) break;  // 스위치 떼면 중단
    }
    if (setpoint == previousSetpoint) isStopped = false;  // 정상 상태로
    increaseSpeed = false;  // 플래그 리셋
  }

  // 타이머 인터럽트로 주기적 PID 계산 (100ms)
  if (timerFlag) {
    // 센서에서 실제 속도 측정 (RPM 단위, 다이어그램의 w)
    int sensorValue = analogRead(sensorPin);  // 센서 값 읽기 (0~1023)
    float actualSpeed = map(sensorValue, 0, 1023, 0, 2000);  // 0~2000 RPM으로 매핑 (조정 필요)

    // PI 제어 계산 (다이어그램의 PI 블록)
    float error = setpoint - actualSpeed;  // 오차 (다이어그램의 e = w* - w)
    integral += error;                     // 적분 항 누적 (다이어그램의 PI 적분)
    float output = (Kp * error) + (Ki * integral);  // PI 출력 (D항 제거)

    // 스로틀 값 조정 (0~180 범위 유지, 다이어그램의 J_p, J_ep)
    throttle = constrain(throttle + output, 0, 180);
    esc.write(throttle);  // ESC에 스로틀 값 전달

    // 디버깅 출력
    Serial.print("Setpoint: "); Serial.print(setpoint);
    Serial.print(" Actual: "); Serial.print(actualSpeed);
    Serial.print(" Throttle: "); Serial.println(throttle);

    // 상태 저장
    prevError = error;

    timerFlag = false;  // 플래그 리셋
  }
}