#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// -------------------- BMP280 센서 --------------------
Adafruit_BMP280 bmp;

float pressure = 0.0;
float altitude = 0.0;
float initialPressure = 0.0;
float sumPressure = 0.0;
int sampleCount = 0;
const int FILTER_SAMPLES = 10;

// -------------------- ESC 및 제어 --------------------
Servo esc;
int escPin = 9;

int switchLeftPin = 2;
int switchRightPin = 4;
int switchMiddlePin = 3;

float targetAltitude = 1;         // 목표 고도 (0 ~ 2 m)
int throttleValue = 0;          // 스로틀 값 (0 ~ 180)
bool motorRunning = false;
int middlePressCount = 0;

// PI 제어 상수
float Kp = 10.0;
float Ki = 1.0;
float integral = 0;
float lastError = 0;

// 기준 스로틀 값
const int throttleBase1m = 120;
const int throttleBase2m = 140;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // ESC 초기화
  esc.attach(escPin, 1000, 2000);
  esc.writeMicroseconds(1000);
  delay(5000);

  // I2C 설정
  Wire.begin();
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
  Wire.setClock(100000);
  delay(2000);

  // BMP280 초기화
  Serial.println("BMP280 초기화 시도...");
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 연결 실패!");
    while (1) {
      delay(2000);
      if (bmp.begin(0x76)) {
        Serial.println("BMP280 연결 성공!");
        break;
      }
    }
  }
  Serial.println("BMP280 연결 성공!");

  // 초기 기압 측정 → 0m 기준
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    float p = bmp.readPressure() / 100.0;
    if (p > 0.0) {
      sumPressure += p;
      sampleCount++;
    }
    delay(100);
  }
  if (sampleCount > 0) {
    initialPressure = sumPressure / sampleCount;
    Serial.print("초기 기준 기압(hPa): ");
    Serial.println(initialPressure, 2);
  } else {
    initialPressure = 1013.25; // 예외 시 기본값
    Serial.println("초기 기압 읽기 실패. 기본값 사용.");
  }
  sumPressure = 0.0;
  sampleCount = 0;

  // 스위치 설정
  pinMode(switchLeftPin, INPUT_PULLUP);
  pinMode(switchRightPin, INPUT_PULLUP);
  pinMode(switchMiddlePin, INPUT_PULLUP);
}

void loop() {
  // -------------------- 고도 측정 --------------------
  pressure = bmp.readPressure() / 100.0;
  if (pressure == 0.0) {
    Serial.println("기압 데이터 오류!");
    delay(1000);
    return;
  }

  // 이동 평균 필터 적용
  sumPressure += pressure;
  sampleCount++;
  if (sampleCount > FILTER_SAMPLES) {
    sumPressure -= (sumPressure / sampleCount);
    sampleCount = FILTER_SAMPLES;
  }
  float avgPressure = sumPressure / sampleCount;
  altitude = 44330.77 * (1.0 - pow(avgPressure / initialPressure, 0.190263));  // 상대 고도

  // -------------------- PI 제어 --------------------
  float Altitudeerror = targetAltitude - altitude;
  integral += Altitudeerror * 0.1;
  float output = Kp * Altitudeerror + Ki * integral;
  lastError = Altitudeerror;

  if (motorRunning) {
    float baseThrottle = (targetAltitude == 1) ? throttleBase1m :
                       (targetAltitude == 2) ? throttleBase2m : 0;
    throttleValue = constrain(baseThrottle + (int)output, 0, 180);
    esc.writeMicroseconds(map(throttleValue, 0, 180, 1000, 2000));
  }

  // -------------------- 스위치 입력 --------------------
  if (digitalRead(switchLeftPin) == LOW) {
    delay(50);
    if (digitalRead(switchLeftPin) == LOW) {
      targetAltitude = max(targetAltitude - 0.2, 0);
      Serial.print("목표 고도 감소: "); Serial.println(targetAltitude);
      while (digitalRead(switchLeftPin) == LOW);
    }
  }

  if (digitalRead(switchRightPin) == LOW) {
    delay(50);
    if (digitalRead(switchRightPin) == LOW) {
      targetAltitude = min(targetAltitude + 0.2, 2);
      Serial.print("목표 고도 증가: "); Serial.println(targetAltitude);
      while (digitalRead(switchRightPin) == LOW);
    }
  }

  if (digitalRead(switchMiddlePin) == LOW) {
    delay(50);
    if (digitalRead(switchMiddlePin) == LOW) {
      middlePressCount++;
      if (middlePressCount == 1) {
        // 모터 ON 및 3초 가속 (최대 스로틀 80)
        motorRunning = true;
        throttleValue = 0;
        unsigned long startTime = millis();
        while (throttleValue < 80 && millis() - startTime < 3000) {
          throttleValue = min(throttleValue + 20, 80);
          esc.writeMicroseconds(map(throttleValue, 0, 180, 1000, 2000));
          Serial.print("가속 중, 스로틀: "); Serial.println(throttleValue);
          delay(500);
        }
        throttleValue = 80;
        esc.writeMicroseconds(map(throttleValue, 0, 180, 1000, 2000));
        targetAltitude = 1;
        Serial.println("모터 ON, 목표 고도 1m, 초기 스로틀 80");
      } else if (middlePressCount == 2) {
        // 모터 OFF 및 착지
        motorRunning = false;
        while (throttleValue > 0) {
          throttleValue = max(throttleValue - 20, 0);
          esc.writeMicroseconds(map(throttleValue, 0, 180, 1000, 2000));
          Serial.print("감속 중, 스로틀: "); Serial.println(throttleValue);
          delay(500);
          if (altitude <= 0.05) break;
        }
        throttleValue = 0;
        esc.writeMicroseconds(1000);
        middlePressCount = 0;
        integral = 0;
        Serial.println("모터 OFF, 0m 착지 완료");
      }
      while (digitalRead(switchMiddlePin) == LOW);
    }
  }

  // -------------------- 상태 출력 --------------------
  Serial.print("\n목표 고도: "); Serial.print(targetAltitude); Serial.print(" m");
  Serial.print(" | 현재 고도: "); Serial.print(altitude, 2); Serial.print(" m");
  Serial.print(" | 스로틀: "); Serial.println(throttleValue);

  delay(100);
}
