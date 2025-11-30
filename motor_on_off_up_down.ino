#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

Servo esc;
int escPin = 9;
int switchLeftPin = 2;
int switchRightPin = 4;
int switchMiddlePin = 3;
int targetAltitude = 1;
int throttleValue = 0;                 // 0 ~ 180
bool motorRunning = false;
int middlePressCount = 0;

const int throttleFor1m = 120;
const int throttleFor2m = 140;

// LCD 주소 0x27 또는 0x3F 등, 16x2 디스플레이 기준
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD 인스턴스 생성

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("PWM Altitude Control with Internal Pullup Starting...");

  esc.attach(escPin, 1000, 2000);
  esc.writeMicroseconds(1000);        // ESC 초기화
  delay(5000);

  pinMode(switchLeftPin, INPUT_PULLUP);
  pinMode(switchRightPin, INPUT_PULLUP);
  pinMode(switchMiddlePin, INPUT_PULLUP);

  lcd.init();            // LCD 초기화
  lcd.backlight();       // 백라이트 켜기
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Throttle: 0%");
}

void loop() {
  Serial.print("\e[2J\e[H");  // 화면 초기화

  if (digitalRead(switchLeftPin) == LOW) {
    delay(50);
    if (digitalRead(switchLeftPin) == LOW) {
      throttleValue = min(throttleValue + 10, 180);
      if (motorRunning) esc.writeMicroseconds(map(throttleValue, 0, 255, 1000, 2000));
      Serial.print("Throttle increased to: "); Serial.println(throttleValue);
      while (digitalRead(switchLeftPin) == LOW);
    }
  }

  if (digitalRead(switchRightPin) == LOW) {
    delay(50);
    if (digitalRead(switchRightPin) == LOW) {
      throttleValue = max(throttleValue - 10, 0);
      if (motorRunning) esc.writeMicroseconds(map(throttleValue, 0, 255, 1000, 2000));
      Serial.print("Throttle decreased to: "); Serial.println(throttleValue);
      while (digitalRead(switchRightPin) == LOW);
    }
  }

  if (digitalRead(switchMiddlePin) == LOW) {
    delay(50);
    if (digitalRead(switchMiddlePin) == LOW) {
      middlePressCount++;
      if (middlePressCount == 1) {
        motorRunning = true;
        throttleValue = 120;
        esc.writeMicroseconds(map(throttleValue, 0, 255, 1000, 2000));
        Serial.println("Motor ON, Throttle set to 120");
      } else if (middlePressCount == 2) {
        motorRunning = false;
        while (throttleValue > 0) {
          throttleValue = max(throttleValue - 20, 0);
          esc.writeMicroseconds(map(throttleValue, 0, 255, 1000, 2000));
          Serial.print("Decelerating, Throttle: "); Serial.println(throttleValue);
          delay(500);
        }
        throttleValue = 0;
        esc.writeMicroseconds(1000);
        middlePressCount = 0;
        Serial.println("Motor OFF");
      }
      while (digitalRead(switchMiddlePin) == LOW);
    }
  }

  Serial.print("Target Altitude: "); Serial.print(targetAltitude); Serial.println("m");
  Serial.print("Throttle: "); Serial.println(throttleValue);

  // --- LCD 업데이트 ---
  lcd.setCursor(0, 0);
  lcd.print("Throttle:       ");  // 초기화
  lcd.setCursor(10, 0);
  int throttlePercent = map(throttleValue, 0, 180, 0, 100);
  lcd.print(throttlePercent);
  lcd.print("% ");

  delay(100);
}

void updateThrottle() {
  if (!motorRunning) return;
  switch (targetAltitude) {
    case 1:
      throttleValue = throttleFor1m;
      break;
    case 2:
      throttleValue = throttleFor2m;
      break;
    default:
      throttleValue = 0;
  }
  esc.writeMicroseconds(map(throttleValue, 0, 180, 1000, 2000));
}
