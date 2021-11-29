#include <Arduino.h>
#include "io/blheli32.h"
#include "flight/pid.h"
#include "flight/imu.h"

float pid;
float PID_right;
float PID_left;

float throttle = 1200;

void setup() {
  Serial.begin(9600);
  blh32_init(9, 1070, 2000);
  setupMPU();
}

void loop() {
  pid = pid_loop(0);

  PID_right = throttle - pid;
  PID_left = throttle + pid;

  BLH_set_left(PID_left);
  BLH_set_right(PID_right);
}
