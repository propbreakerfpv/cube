#include "io/blheli32.h"

Servo ESC_left;
Servo ESC_right;

void blh32_init(int pin, int min, int max){
  ESC_left.attach(ESC_LEFT_PIN, min, max);
  ESC_right.attach(ESC_RIGHT_PIN, min, max);
  ESC_left.writeMicroseconds(ESC_ARM_SIGNAL);
  ESC_right.writeMicroseconds(ESC_ARM_SIGNAL);

  unsigned long now = millis();
  while (millis() < now + ESC_ARM_TIME){}

}

void BLH_set_left(int val){
  ESC_left.writeMicroseconds(val);
}

void BLH_set_right(int val){
  ESC_right.writeMicroseconds(val);
}
