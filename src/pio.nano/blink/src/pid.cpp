#include "flight/pid.h"
#include "flight/imu.h"


float kp = 1;
float ki = 1;
float kd = 1;

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float PID, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

float rad_to_deg = 180/3.141592654;

float elapsedTime, time, timePrev;

float acc_rawX, acc_rawY, acc_rawZ,gyr_rawX, gyr_rawY, gyr_rawZ;




float pid_loop(float setpoit){

    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000;


    // get the accel data
    recordAccelRegisters();

    acc_rawX = get_accelx();
    acc_rawY = get_accely();
    acc_rawZ = get_accelz();



    // sum randum filters
    Acceleration_angle[0] = atan((acc_rawY/16384.0)/sqrt(pow((acc_rawX/16384.0),2) + pow((acc_rawZ/16384.0),2)))*rad_to_deg;

    Acceleration_angle[1] = atan(-1*(acc_rawX/16384.0)/sqrt(pow((acc_rawY/16384.0),2) + pow((acc_rawZ/16384.0),2)))*rad_to_deg;


    // get the gyro data
    recordGyroRegisters();

    gyr_rawX = get_gyro_datax();
    gyr_rawY = get_gyro_datay();


    Gyro_angle[0] = gyr_rawX/131.0;

    Gyro_angle[1] = gyr_rawY/131.0;

    // sum randum filters
    Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];

    Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];


    error = Total_angle[0] - setpoit;

    pid_p = kp*error;


    pid_i = pid_i+(ki*error);


    pid_d = kd*((error - previous_error)/elapsedTime);

    PID = pid_p + pid_i + pid_d;

    if(PID < -1000){
    PID=-1000;
    }
    if(PID > 1000){
    PID=1000;
    }

    previous_error = error;


    Serial.print(Total_angle[0]);
    Serial.print(" ");
    Serial.println(PID);
    return PID;

}


















//void PIDController_Init(PIDController *pid) {
//
//  /* Clear controller variables */
//  pid->integrator = 0.0f;
//  pid->prevError  = 0.0f;
//
//  pid->differentiator  = 0.0f;
//  pid->prevMeasurement = 0.0f;
//
//  pid->out = 0.0f;
//
//    pid->Kp = 1;
//    pid->Ki = 1;
//    pid->Kd = 1;
//}
//
//float PIDloop(PIDController *pid, float measurement,float setpoit){
//    // calculate error
//    float error = setpoit - measurement;
//
//    // calculate p term
//    float proportional = pid->Kp * error;
//
//    // calculate i term
//    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);
//
//    // calculate i term Anti-wind-up limits
//    if (pid->limMax > proportional){
//        pid->limMaxInt = pid->limMax - proportional;
//    }else{
//        pid->limMaxInt = 0.0f;
//    }
//    if (pid->limMin > proportional){
//        pid->limMinInt = pid->limMin - proportional;
//    }else{
//        pid->limMinInt = 0.0f;
//    }
//
//    // use i term Anti-wind-up limitd
//    if (pid->integrator > pid->limMaxInt) {
//
//        pid->integrator = pid->limMaxInt;
//
//    } else if (pid->integrator < pid->limMinInt) {
//
//        pid->integrator = pid->limMinInt;
//
//    }
//
//    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement) /* Note:
//    derivative on measurement, therefore minus sign in front of equation! */
//                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
//                        / (2.0f * pid->tau + pid->T);
//
//
//
//    pid->out = proportional + pid->integrator + pid->differentiator;
//
//
//    // cap the output
//    if (pid->out > pid->limMax){
//        pid->out = pid->limMax;
//    }else if (pid->out < pid->limMin){
//        pid->out = pid->limMin;
//    }
//
//    pid->prevError = error;
//    pid->prevMeasurement = measurement;
//
//    return pid->out;
//}
