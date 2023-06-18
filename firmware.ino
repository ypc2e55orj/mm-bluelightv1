#include "src/driver/encoder.h"
#include "src/driver/imu.h"

void setup()
{
  Serial.begin(115200);
  driver::encoder::init();
  driver::imu::init();

  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);
  buzzer();
}

void buzzer()
{
  for (int i = 0; i < 100; i++)
  {
    digitalWrite(21, HIGH);
    delayMicroseconds(500);
    digitalWrite(21, LOW);
    delayMicroseconds(500);
  }
}

void loop()
{
  uint16_t left, right;
  driver::encoder::angle(left, right);
  Serial.printf("\r\n%d, %d", left, right);

  delay(1);
}
