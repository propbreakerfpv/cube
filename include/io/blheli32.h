#pragma once
#include <Arduino.h>
#include <Servo.h>


#define ESC_ARM_SIGNAL 1000
#define ESC_ARM_TIME 2000

#define ESC_LEFT_PIN 9
#define ESC_RIGHT_PIN 11

void blh32_init(int pin, int min, int max);

void BLH_set_left(int val);
void BLH_set_right(int val);
