#include <Servo.h>
Servo esc;
int escPin = 9;  // 또는 44
int throttle = 0;
void setup() {
  esc.attach(escPin, 1000, 2000);
  esc.write(0);
  delay(5000);
}
void loop() {
  for (throttle = 0; throttle <= 80; throttle += 5) {
    esc.write(throttle);
    delay(20);
  }
  esc.write(80);
  delay(5000);
  for (throttle = 80; throttle >= 0; throttle -= 5) {
    esc.write(throttle);
    delay(20);
  }
  esc.write(0);
  delay(1000);
}