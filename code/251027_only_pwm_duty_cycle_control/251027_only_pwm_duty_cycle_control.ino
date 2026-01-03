#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>

// -------------------- LCD --------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C 주소 0x27, 1602 LCD

// -------------------- BMP280 센서 (사용 안 함: 삭제해도 됨) --------------------
// PI 제어 삭제로 인해 더 이상 고도 측정에 사용되지 않지만, 라이브러리 선언은 유지합니다.
Adafruit_BMP280 bmp;

// -------------------- ESC (9번 핀) 및 제어 --------------------
Servo esc;
int escPin = 9;

int escSwitchDecPin = 2; // ESC Duty Cycle 감소 스위치 (10% 감소)
int escSwitchPowerPin = 3; // ESC 전원 스위치
int escSwitchIncPin = 4; // ESC Duty Cycle 증가 스위치 (10% 증가)

int escDutyCycle = 0; // 0 ~ 100 (%)
bool escRunning = false;
int escPowerCount = 0; // 3번 스위치 누름 횟수

// -------------------- 새로운 PWM 제어 (10번 핀) --------------------
int newPwmPin = 10;
int newPwmDutyCycle = 0; // 0 ~ 100 (%)
bool newPwmRunning = false;
int newPwmSwitchDecPin = 5; // Duty Cycle 감소 스위치 (10% 감소)
int newPwmSwitchPowerPin = 6; // 전원 스위치 (켜기: 10%, 끄기: 0%)
int newPwmSwitchIncPin = 7; // Duty Cycle 증가 스위치 (10% 증가)
int newPwmPowerCount = 0; // 6번 스위치 누름 횟수
// --------------------------------------------------------

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // LCD 초기화
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("System Initializing");

  // ESC 초기화
  esc.attach(escPin, 1000, 2000);
  esc.writeMicroseconds(1000);
  delay(3000);
  
  // -------------------- 핀 설정 --------------------
  pinMode(escSwitchDecPin, INPUT_PULLUP);
  pinMode(escSwitchPowerPin, INPUT_PULLUP);
  pinMode(escSwitchIncPin, INPUT_PULLUP);
  
  pinMode(newPwmPin, OUTPUT);
  pinMode(newPwmSwitchDecPin, INPUT_PULLUP);
  pinMode(newPwmSwitchPowerPin, INPUT_PULLUP);
  pinMode(newPwmSwitchIncPin, INPUT_PULLUP);
  // --------------------------------------------------

  // BMP280 초기화 (PI 제어에 사용되지 않으나, 초기화 코드는 그대로 둡니다.)
  Serial.println("BMP280 초기화 시도...");
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 연결 실패!");
  } else {
    Serial.println("BMP280 연결 성공!");
  }
  
  lcd.clear();
  lcd.print("Ready");
}

void loop() {
  // -------------------- ESC 제어 (9번 핀, 기존 2, 3, 4 스위치) --------------------
  // 2번 스위치 (Duty Cycle 10% 감소)
  if (digitalRead(escSwitchDecPin) == LOW) {
    delay(50);
    if (digitalRead(escSwitchDecPin) == LOW) {
      if (escRunning) {
        escDutyCycle = max(escDutyCycle - 10, 0);
        Serial.print("ESC Duty Cycle 감소: "); Serial.println(escDutyCycle);
      }
      while (digitalRead(escSwitchDecPin) == LOW);
    }
  }

  // 4번 스위치 (Duty Cycle 10% 증가)
  if (digitalRead(escSwitchIncPin) == LOW) {
    delay(50);
    if (digitalRead(escSwitchIncPin) == LOW) {
      if (escRunning) {
        escDutyCycle = min(escDutyCycle + 10, 100);
        Serial.print("ESC Duty Cycle 증가: "); Serial.println(escDutyCycle);
      }
      while (digitalRead(escSwitchIncPin) == LOW);
    }
  }

  // 3번 스위치 (전원 스위치)
  if (digitalRead(escSwitchPowerPin) == LOW) {
    delay(50);
    if (digitalRead(escSwitchPowerPin) == LOW) {
      escPowerCount++;
      if (escPowerCount == 1) {
        escRunning = true;
        escDutyCycle = 10; // 10%로 시작
        Serial.println("ESC ON, Duty Cycle 10%");
      } else if (escPowerCount == 2) {
        escRunning = false;
        escDutyCycle = 0; // 0%로 종료
        escPowerCount = 0; // 카운트 초기화
        Serial.println("ESC OFF, Duty Cycle 0%");
      }
      while (digitalRead(escSwitchPowerPin) == LOW);
    }
  }
  
  // ESC 출력 (0% = 1000us, 100% = 2000us)
  int escMicroseconds = map(escDutyCycle, 0, 100, 1000, 2000); 
  esc.writeMicroseconds(escMicroseconds); 


  // -------------------- 새로운 PWM 제어 (10번 핀, 5, 6, 7 스위치) --------------------
  // 5번 스위치 (Duty Cycle 10% 감소)
  if (digitalRead(newPwmSwitchDecPin) == LOW) {
    delay(50);
    if (digitalRead(newPwmSwitchDecPin) == LOW) {
      if (newPwmRunning) {
        newPwmDutyCycle = max(newPwmDutyCycle - 10, 0);
        Serial.print("새 PWM Duty Cycle 감소: "); Serial.println(newPwmDutyCycle);
      }
      while (digitalRead(newPwmSwitchDecPin) == LOW);
    }
  }

  // 7번 스위치 (Duty Cycle 10% 증가)
  if (digitalRead(newPwmSwitchIncPin) == LOW) {
    delay(50);
    if (digitalRead(newPwmSwitchIncPin) == LOW) {
      if (newPwmRunning) {
        newPwmDutyCycle = min(newPwmDutyCycle + 10, 100);
        Serial.print("새 PWM Duty Cycle 증가: "); Serial.println(newPwmDutyCycle);
      }
      while (digitalRead(newPwmSwitchIncPin) == LOW);
    }
  }

  // 6번 스위치 (전원 스위치)
  if (digitalRead(newPwmSwitchPowerPin) == LOW) {
    delay(50);
    if (digitalRead(newPwmSwitchPowerPin) == LOW) {
      newPwmPowerCount++;
      if (newPwmPowerCount == 1) {
        newPwmRunning = true;
        newPwmDutyCycle = 10; // 10%로 시작
        Serial.println("새 PWM ON, Duty Cycle 10%");
      } else if (newPwmPowerCount == 2) {
        newPwmRunning = false;
        newPwmDutyCycle = 0; // 0%로 종료
        newPwmPowerCount = 0; // 카운트 초기화
        Serial.println("새 PWM OFF, Duty Cycle 0%");
      }
      while (digitalRead(newPwmSwitchPowerPin) == LOW);
    }
  }

  // 새로운 PWM 출력 (10번 핀, 0~255 범위)
  int pwmValue = map(newPwmDutyCycle, 0, 100, 0, 255); 
  analogWrite(newPwmPin, pwmValue); 

  // -------------------- 상태 출력 --------------------
  Serial.print("\nESC 듀티: "); Serial.print(escDutyCycle); Serial.print(" % ("); Serial.print(escRunning ? "ON" : "OFF"); Serial.print(")");
  Serial.print(" | 새 PWM 듀티: "); Serial.print(newPwmDutyCycle); Serial.print(" % ("); Serial.print(newPwmRunning ? "ON" : "OFF"); Serial.println(")");

  // -------------------- LCD 출력 --------------------
  lcd.setCursor(0, 0);
  lcd.print("ESC:");
  lcd.print(escDutyCycle);
  lcd.print("% ");
  lcd.print(escRunning ? "ON " : "OFF");
  lcd.print("     ");  // 여백

  lcd.setCursor(0, 1);
  lcd.print("PWM:");
  lcd.print(newPwmDutyCycle);
  lcd.print("% ");
  lcd.print(newPwmRunning ? "ON " : "OFF");
  lcd.print("     ");  // 여백

  delay(100);  // 10Hz 주기
}