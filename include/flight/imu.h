#pragma once
#include <Arduino.h>


void setupMPU();
void recordAccelRegisters();
void recordGyroRegisters();
float get_gyro_datax();
float get_gyro_datay();
float get_gyro_dataz();
float get_accelx();
float get_accely();
float get_accelz();

void printData_temp();
