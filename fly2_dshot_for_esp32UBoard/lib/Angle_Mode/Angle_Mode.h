#ifndef _AngleMode_H
#define _AngleMode_H
#include <Arduino.h>
#include <EEPROM.h>
#define ANGLE 60
#define SAMPLE_ACCE 4000
#define SAMPLE_GYRO 4000
// #define THROTTLE_MIN       0//102
// #define THROTTLE_MAX       1900
extern float q0,q1,q2,q3;
extern float exInt,eyInt,ezInt;
extern float x_error,y_error;
extern float x_error_integral,y_error_integral;
extern float target_x,target_y;
// extern float pitch0,roll0,yaw0;
extern float A[3][3];
extern float norm0,ACC_ERROR_THRESHOLD;
// extern float alfa,thita;
// extern int number_acce;
// extern int number_gyro;
// extern int gyro_stable_number,acce_stable_number;
// extern int gx_fix,gy_fix,gz_fix;
// extern float gx_offset,gy_offset,gz_offset;
// extern float previous_gx_2 , previous_gy_2 , previous_gz_2 ;
// extern float previous_targetgx_2, previous_targetgy_2, previous_targetgz_2;
// extern float gx_error_integral_2, gy_error_integral_2 ,gz_error_integral_2 ;
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float norm,float error,double T);
void calculateEulerAngles(float* pitch, float* roll, float* yaw);
// void adjustMotors_Angle(float gx,float gy,float gz);
void getTargetGyro(float pitch,float roll,float* target_gx,float* target_gy);
float InputMapAngle(uint16_t rawSpeed,float minrate,float maxrate);
float getAngleBaserate(uint16_t rawSpeed,float maxrate,float rcRate,float Rate,float rcExpo,float mid);
void calibrateAngle();
void AverageAcceleration(float ax,float ay,float az,int* flag_acce,float* ax0,float* ay0,float* az0);
void AverageGyro(float gx,float gy,float gz,int* flag_gyro,float* gx0,float* gy0,float* gz0);
#endif