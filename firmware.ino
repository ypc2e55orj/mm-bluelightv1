#include "src/driver/encoder.h"
#include "src/driver/imu.h"

void setup() {
  Serial.begin(115200);
  driver::encoder::init();
  driver::imu::init();
}

void loop() {
#if 0
  uint16_t left, right;
  driver::encoder::angle(left, right);
  Serial.printf("\r\n%d, %d", left, right);

  delay(1);
#endif


}
