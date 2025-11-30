#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_asukiaaa.h>

#define MPU9250_ADDRESS 0x68 // MPU-9250 I2C 주소
#define BMP280_ADDRESS 0x76  // BMP280 I2C 주소

Adafruit_BMP280 bmp; // BMP280 객체
MPU9250_asukiaaa mpu; // MPU-9250 객체

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // BMP280 초기화
  if (!bmp.begin(BMP280_ADDRESS)) {
    Serial.println("BMP280 센서 연결 실패! 배선 확인");
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, // 운영 모드
                    Adafruit_BMP280::SAMPLING_X2,   // 온도 오버샘플링
                    Adafruit_BMP280::SAMPLING_X16,  // 기압 오버샘플링
                    Adafruit_BMP280::FILTER_X16,    // 필터링
                    Adafruit_BMP280::STANDBY_MS_500); // 대기 시간

  // MPU-9250 초기화
  mpu.setWire(&Wire);
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();
  Serial.println("MPU9250 초기화 시도..."); // 추가: 초기화 시도 메시지
  Serial.println("GY-91 초기화 완료!");
}

void loop() {
  // BMP280 데이터 읽기
  Serial.println("=== BMP280 ===");
  Serial.print("온도: "); Serial.print(bmp.readTemperature()); Serial.println(" °C");
  Serial.print("기압: "); Serial.print(bmp.readPressure() / 100.0); Serial.println(" hPa");
  Serial.print("고도: "); Serial.print(bmp.readAltitude(1013.25)); Serial.println(" m");

  // MPU-9250 데이터 읽기
  Serial.println("=== MPU-9250 ===");
  if (mpu.accelUpdate() == 0) {
    Serial.print("가속도 X: "); Serial.print(mpu.accelX()); Serial.print(" g, ");
    Serial.print("Y: "); Serial.print(mpu.accelY()); Serial.print(" g, ");
    Serial.print("Z: "); Serial.println(mpu.accelZ()); Serial.println(" g");
  }
  if (mpu.gyroUpdate() == 0) {
    Serial.print("자이로 X: "); Serial.print(mpu.gyroX()); Serial.print(" °/s, ");
    Serial.print("Y: "); Serial.print(mpu.gyroY()); Serial.print(" °/s, ");
    Serial.print("Z: "); Serial.println(mpu.gyroZ()); Serial.println(" °/s");
  }
  if (mpu.magUpdate() == 0) {
    Serial.print("자력 X: "); Serial.print(mpu.magX()); Serial.print(" μT, ");
    Serial.print("Y: "); Serial.print(mpu.magY()); Serial.print(" μT, ");
    Serial.print("Z: "); Serial.println(mpu.magZ()); Serial.println(" μT");
  }

  Serial.println();
  delay(1000); // 1초 대기
}