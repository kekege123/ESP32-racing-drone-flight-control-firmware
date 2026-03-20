#include <Arduino.h>
#include "Angle_Mode.h"
// #include "CommonControl.h"
#include "ICM42688.h"
// extern float target_gx ;// 声明，告诉编译器变量在其他地方定义
// extern float target_gy ;
// extern float target_gz ;
// extern int base_pwm;
// extern int throttles[4];
// extern float d_weight,alpha;
//加速度更正矩阵
// float ax0=0.0,ay0=0.0,az0=9.55;
float sum_ax0=0,sum_ay0=0,sum_az0=0;
int number_acce=0;
// float gx_offset =0.0f, gy_offset = 0.0f, gz_offset = 0.0f;//esp32U_1
// int number_gyro=0;
// int gyro_stable_number=0;
int acce_stable_number=0;
int gx_fix=0,gy_fix=0,gz_fix=0;

// float thita=acos(az0/sqrt(ax0*ax0+ay0*ay0+az0*az0));
// float s1=(ax0*ax0+ay0*ay0)==0 ? sqrt(2)/2 : ay0/sqrt(ax0*ax0+ay0*ay0),s2=sin(thita);
// float c1=(ax0*ax0+ay0*ay0)==0 ? sqrt(2)/2 : ax0/sqrt(ax0*ax0+ay0*ay0),c2=cos(thita);//s1=sin(alfa),s2=sin(thita),c1=cos(alfa)
// float A[3][3] = {{c2+s1*s1*(1-c2),-s1*c1*(1-c2),s2*c1},{-s1*c1*(1-c2),c2+c1*c1*(1-c2),s2*s1},{-s2*c1,-s2*s1,c2}};
// float thita;
// float s1,s2;
// float c1,c2;//s1=sin(alfa),s2=sin(thita),c1=cos(alfa)
float A[3][3];
// 四元数相关变量
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数
float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;   // 积分误差
float last_ex = 0, last_ey = 0, last_ez = 0; // 存储上一次的误差
// PI 控制器参数，1.6，0.0002
// float Kp = 3.0f, Ki = 0.0001f,Kd = 0.0f; 
float Kp = 30.0f , Ki=0.0f; 
// float Ki_p=0.00f,Ki_r=0.000f,Ki_yaw = 0.000f;
// float halfT = 0.00007576f;                             // 采样周期的一半，0.0002
float spin_rate_limit = 20.0 * (M_PI / 180.0);
float norm0=0.0f;
float ACC_ERROR_THRESHOLD=0.0f;
//角速度
float sum_gx=0,sum_gy=0,sum_gz=0;
int number_gyro=0;
//角度校准值
// float roll0=0,pitch0=0,yaw0=0;
//角度PID参数
float target_x = 0.0f; // 单位rad
float target_y = 0.0f;  
float target_turnpt=2*M_PI/180.0;
float angle_turnpt=10.0;
float Kp_x = 10.0f, Ki_x =0.0f,Kd_x=0.0f;//kp<25,kd=5~10
float Kp_y = 10.0f, Ki_y =0.0f,Kd_y=0.0f;//kp<25,kd=5~10
// float Kp1=0.5,Kp2=5.0;
float x_error = 0.0f, y_error = 0.0f;
float x_error_integral = 0.0f, y_error_integral = 0.0f;
float last_x_error = 0.0f, last_y_error = 0.0f;
float x_error_derivative = 0.0f, y_error_derivative = 0.0f;
//角速度PID参数，和手动模式用不同的一套参数，避免串参
// float Kp_gx_2 = 15.0f, Ki_gx_2  =33.0f, Kd_gx_2  =30.0f, FF_gainx_2 =120.0f;//Ki_gx = 80.0f,kp=0.2； Ki_gx = 0.004f,kp=0.5
// float Kp_gy_2  =16.0f, Ki_gy_2  =33.0f, Kd_gy_2  =34.0f, FF_gainy_2 =125.0f;//Ki_gx = 84.0f,kp=0.2；Ki_gy = 0.004f,kp=0.5
// float Kp_gz_2  =15.0f, Ki_gz_2  =35.0f, Kd_gz_2  = 0.0f, FF_gainz_2 =120.0f;
// float gx_error_2 = 0.0f, gy_error_2 = 0.0f,gz_error_2 = 0.0f;
// float gx_error_integral_2 = 0.0f, gy_error_integral_2 = 0.0f,gz_error_integral_2 = 0.0f;
// float gx_error_derivative_2 = 0.0f, gy_error_derivative_2 = 0.0f,gz_error_derivative_2 = 0.0f;
// float previous_gx_2 = 0.0f, previous_gy_2 = 0.0f, previous_gz_2 = 0.0f;
// float previous_targetgx_2 = 0.0f, previous_targetgy_2 = 0.0f, previous_targetgz_2 = 0.0f;
// float dt_2=6000.0;
// 四元数更新函数
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az ,float norm,float error,double T) {
    // float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    //将陀螺仪数据先转换为弧度每秒再输入姿态解算函数：
    gx = gx * M_PI / 180.0f;
    gy = gy * M_PI / 180.0f;
    gz = gz * M_PI / 180.0f;
    float spin_rate = sqrt(gx * gx + gy * gy + gz * gz);
    // 归一化加速度计数据
    // az=sqrt(norm0*norm0-ax * ax - ay * ay);
    // Serial.println(az);
    // norm = sqrt(ax * ax + ay * ay + az * az);
    // float error=fabs(norm-norm0);
    if(error>ACC_ERROR_THRESHOLD){
      Kp=0.1;
      Kp_x = 0.3f;
      Kp_y = 0.3f;
    }
    else{
      Kp=30.0;
      Kp_x = 50.0f;
      Kp_y = 50.0f;
    }
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
  
    // 计算重力向量
    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
  
    // 计算误差向量
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
  
    
    // float dex = ex - last_ex;
    // float dey = ey - last_ey;
    // float dez = ez - last_ez;
    // // 更新上一次误差
    // last_ex = ex;
    // last_ey = ey;
    // last_ez = ez;
    // 修正陀螺仪数据
    // 仅在角速度低于限制时积分
    if (Ki!=0 && spin_rate < spin_rate_limit) {
      // 积分误差
      exInt += ex ;
      eyInt += ey ;
      ezInt += ez ;
      gx = gx + Kp * ex + Ki*exInt;
      gy = gy + Kp * ey + Ki*eyInt;
      gz = gz + Kp * ez + Ki*ezInt;
    }
    else{
      gx = gx + Kp * ex;
      gy = gy + Kp * ey;
      gz = gz + Kp * ez;
    }
   
    // 四元数微分方程
    double halfT=T/2;
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
  
    // 归一化四元数
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
}
  
// 计算欧拉角
void calculateEulerAngles(float* pitch, float* roll, float* yaw) {
// *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) / M_PI * 180.0f; // 俯仰角
// *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) / M_PI * 180.0f; // 横滚角
  float roll_sin = -2 * q1 * q3 + 2 * q0 * q2;
// 关键步骤：防止因浮点数舍入误差导致asin输入超出[-1, 1]的范围
  if (roll_sin > 1.0f) {
      roll_sin = 1.0f;
  } else if (roll_sin < -1.0f) {
      roll_sin= -1.0f;
  }
  *roll = asin(roll_sin) / M_PI * 180.0f; // 俯仰角
  *pitch = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) / M_PI * 180.0f; // 横滚角
  *yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) / M_PI * 180.0f; // 航向角
  // Serial.printf("%f \n",(2 * q2 * q3 + 2 * q0 * q1));
}
/*
void adjustMotors_Angle(float gx,float gy,float gz){
  gx = gx * M_PI / 180.0f;
  gy = gy * M_PI / 180.0f;
  gz = gz * M_PI / 180.0f;
  // gz_p = gz_p * M_PI / 180.0f;
  // 计算误差
  gx_error_2 = target_gx - gx;
  gy_error_2 = target_gy - gy;
  gz_error_2 = target_gz - gz;
  // gz_error_p = target_gz - gz_p;
  // float gx_error_P=constrain(gx_error,-2.22,2.22);
  // float gy_error_P=constrain(gy_error,-2.12,2.12);
  // float gz_error_P=constrain(gz_error,-2.22,2.22);
  gx_error_integral_2 += gx_error_2;
  gy_error_integral_2 += gy_error_2;
  gz_error_integral_2 += gz_error_2;
  float gx_diff = gx - previous_gx_2;
  float gy_diff = gy - previous_gy_2;
  float gz_diff = gz - previous_gz_2;
  // filtered_gx_diff = alpha * gx_diff + (1 - alpha) * filtered_gx_diff;
  // filtered_gy_diff = alpha * gy_diff + (1 - alpha) * filtered_gy_diff;
  // filtered_gz_diff = alpha * gz_diff + (1 - alpha) * filtered_gz_diff;
  gx_error_derivative_2 = d_weight*(target_gx-previous_targetgx_2)-gx_diff;
  gy_error_derivative_2 = d_weight*(target_gy-previous_targetgy_2)-gy_diff;
  gz_error_derivative_2 = d_weight*(target_gz-previous_targetgz_2)-gz_diff;
  gx_error_derivative_2 = constrain(gx_error_derivative_2,-1.0,1.0);
  gy_error_derivative_2 = constrain(gy_error_derivative_2,-1.0,1.0);

 
  // float FF_termx=FF_gainx*(previous_inputx-channels.roll)* M_PI / 180.0;
  // float FF_termy=FF_gainy*(previous_inputy-channels.pitch)* M_PI / 180.0;
  // float FF_termz=FF_gainz*(previous_inputz-channels.yaw)* M_PI / 180.0;
  // FF_termx=constrain(FF_termx,-40,40);
  // FF_termy=constrain(FF_termy,-46,46);
  // FF_termz=constrain(FF_termz,-40,40);
  // float gx_output = Kp_gx * gx_error + Ki_gx * gx_error_integral/9000.0+ Kd_gx * gx_error_derivative +FF_termx;
  // float gy_output = Kp_gy * gy_error + Ki_gy * gy_error_integral/9000.0 + Kd_gy * gy_error_derivative +FF_termy;
  // float gz_output = Kp_gz * gz_error + Ki_gz * gz_error_integral/8000.0 +FF_termz;
  
  float gx_output = Kp_gx_2 * gx_error_2 + Ki_gx_2 * gx_error_integral_2/dt_2;
  float gy_output = Kp_gy_2 * gy_error_2 + Ki_gy_2 * gy_error_integral_2/dt_2;
  // float gz_output = Kp_gz * gz_error_p + Ki_gz * gz_error_integral/9000.0;
  float gz_output = Kp_gz_2 * gz_error_2 + Ki_gz_2 * gz_error_integral_2/dt_2;

  // Serial.printf(" %f , %f,%f\n",FF_termx, FF_termy,FF_termz);
  // 更新前一次误差
  previous_gx_2 = gx;
  previous_gy_2 = gy;
  previous_gz_2 = gz;
  previous_targetgx_2 = target_gx;
  previous_targetgy_2 = target_gy;
  previous_targetgz_2 = target_gz;
  // previous_inputx_2=channels.roll;
  // previous_inputy_2=channels.pitch;
  // previous_inputz_2=channels.yaw;
  // 调整电机转速
  //gx_error若小于0，gx要往小调，gx增大一侧的两个电机转速往上调，应该减去gx_error
  //gy_error若小于0，gy要往小调，gy增大一侧的两个电机转速往上调，应该减去gy_error
  //gz_error若小于0，gz正向过大，要往小调，与gz增大同旋向的两个桨叶对应的电机1、4转速往上调，应该减gz_error，因为反扭距会克服gz的过大
  if(base_pwm<1350){
   
    throttles[0] = base_pwm - gx_output-gy_output+gz_output; // 电机1
    throttles[1]  = base_pwm + gx_output-gy_output-gz_output; // 电机2
    throttles[2]  = base_pwm - gx_output +gy_output-gz_output; // 电机3
    throttles[3] = base_pwm +gx_output +gy_output +gz_output; // 电机4
  }
  else{
    float TPA_multiplier=1-(base_pwm-1350.0)/650.0*0.65;
    throttles[0] = base_pwm + (-gx_output-gy_output+gz_output)*TPA_multiplier; // 电机1
    throttles[1]  = base_pwm + (+gx_output-gy_output-gz_output)*TPA_multiplier; // 电机2
    throttles[2]  = base_pwm +(- gx_output +gy_output-gz_output)*TPA_multiplier; // 电机3
    throttles[3] = base_pwm +(+gx_output +gy_output +gz_output)*TPA_multiplier; // 电机4
  }
  // 限制PWM值在有效范围内
  throttles[0] = constrain(throttles[0], THROTTLE_MIN , THROTTLE_MAX );
  throttles[1] = constrain(throttles[1], THROTTLE_MIN , THROTTLE_MAX);
  throttles[2] = constrain(throttles[2], THROTTLE_MIN , THROTTLE_MAX);
  throttles[3] = constrain(throttles[3], THROTTLE_MIN ,THROTTLE_MAX);
}
*/
void getTargetGyro(float pitch,float roll,float* target_gx,float* target_gy){
  // if(abs(pitch)<angle_turnpt && abs(target_x)<target_turnpt){
  //   Kp_x=Kp1;
  // }
  // else{
  //   Kp_x=Kp2;
  // }
  // if(abs(roll)<angle_turnpt && abs(target_y)<target_turnpt){
  //   Kp_y=Kp1;
  // }
  // else{
  //   Kp_y=Kp2;
  // }
  pitch=pitch* M_PI / 180.0f;
  roll=roll* M_PI / 180.0f;
  x_error = target_x - pitch;
  y_error = target_y - roll;
  
  // x_error_integral += x_error;
  // y_error_integral += y_error;
  // x_error_derivative=x_error-last_x_error;
  // y_error_derivative=y_error-last_y_error;
  // 计算PID输出
   //如果x_output>0,说明要向roll增大的方向转，target_gx为正而与target_gy无关
  *target_gx = Kp_x * x_error  ;
  *target_gy = Kp_y * y_error  ;
  // last_x_error=x_error;
  // last_y_error=y_error;
  // Serial.printf("%f %f\n",target_gx,target_gy);
 
}
float InputMapAngle(uint16_t rawSpeed,float minrate,float maxrate){
  float baseAngle;
  baseAngle=map(rawSpeed,minrate,maxrate,ANGLE,-ANGLE);
  return baseAngle;
}
float getAngleBaserate(uint16_t rawSpeed,float maxrate,float rcRate,float Rate,float rcExpo,float mid){
  int sign;
  float stickInput;
  float response;
  float baseRate;
  // stickInput=map(rawSpeed, 1811, 174, -1, 1);
  stickInput=-1+2*((rawSpeed-0)/(maxrate-0));
  if(stickInput>=0){
    sign=1;
  }
  else{
    sign=-1;
  }
  if(abs(stickInput)<=mid){
    response=pow(stickInput,3)*rcExpo+stickInput*(1-rcExpo);
  }
  else{
    response=stickInput*Rate+(pow(stickInput,2)-pow(mid,2))*sign*(Rate/mid-1)*(Rate/mid);
  }
  baseRate=ANGLE*rcRate*response;
  return baseRate;
}
/*
void calibrateAngle(){
  float ax,ay,az,gx,gy,gz;
  float roll,pitch,yaw;
  q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数
  exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;   // 积分误差
  for (int i = 0; i < 100; i++) {
        digitalWrite(ICM_CS_PIN, LOW);
        SPI.transfer(0x1F | 0x80); // 最高位=1（读操作）
        // 读取加速度计数据
        int16_t rawAX = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
        int16_t rawAY = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
        int16_t rawAZ = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
        // 读取陀螺仪原始数据
        int16_t rawGX = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
        int16_t rawGY = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
        int16_t rawGZ = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
        digitalWrite(ICM_CS_PIN, HIGH);
        // ±4g量程转换 (8192 LSB/g)
        const float accelScale = 0.00119695f; // 9.80665 / 8192
        ax= rawAX * accelScale;
        ay= rawAY * accelScale;
        az= rawAZ * accelScale;
        // float alfa=atan2(0.2,9.8);
        // ax=0.2*cos(alfa)-9.8*sin(alfa);
        // az=0.2*sin(alfa)+9.8*cos(alfa);

        ax=ax*A[0][0]+ay*A[1][0]+az*A[2][0];
        ay=ax*A[0][1]+ay*A[1][1]+az*A[2][1];
        az=ax*A[0][2]+ay*A[1][2]+az*A[2][2];
        float a=sqrt(ax*ax+ay*ay+az*az);
        pitch=asin(-ax/a)/ M_PI * 180.0f;
        roll=asin(ay/a)/ M_PI * 180.0f;
        // const float gyroScale = 0.015267176f;
        // gx = (rawGX * gyroScale) - gx_offset;
        // gy = (rawGY * gyroScale) - gy_offset;
        // gz = (rawGZ * gyroScale) - gz_offset;
        // IMUupdate(gx, gy, gz, ax, ay, az);
        // // 可选：计算欧拉角
        // calculateEulerAngles(&pitch, &roll, &yaw);
        // if(i>=50){
        roll0+=roll;
        pitch0+=pitch;
        // yaw0+=yaw;
        // }
        
    }
    roll0=roll0/100;
    pitch0=pitch0/100;
    // yaw0=yaw0/50;
    
    // q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数
    // exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;   // 积分误差
}
*/
void AverageAcceleration(float ax,float ay,float az,int* flag_acce,float* ax0,float* ay0,float* az0){
  // while(number_acce<SAMPLE_ACCE){
    /*
    float ax, ay, az;
      
    int16_t rawAX,rawAY,rawAZ;
    // digitalWrite(AT7456_CS, HIGH); 
    // if(xSemaphoreTake(spiMutex,portMAX_DELAY)==pdTRUE){
    digitalWrite(ICM_CS_PIN, LOW);

    SPI.transfer(0x1F | 0x80); // 最高位=1（读操作）
    // 读取加速度计数据
    rawAX = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
    rawAY = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
    rawAZ = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
    
  
    digitalWrite(ICM_CS_PIN, HIGH);
  
    // ±4g量程转换 (8192 LSB/g)
    const float accelScale = 0.00119695f; // 9.80665 / 8192
    ax = (rawAX * accelScale) ;
    ay = (rawAY * accelScale) ;
    az = (rawAZ * accelScale) ;
    */
  sum_ax0+=ax;
  sum_ay0+=ay;
  sum_az0+=az;
  number_acce+=1;
  if(number_acce==SAMPLE_ACCE){
    *ax0=sum_ax0/SAMPLE_ACCE;
    *ay0=sum_ay0/SAMPLE_ACCE;
    *az0=sum_az0/SAMPLE_ACCE;
    float Ax0=*ax0;
    float Ay0=*ay0;
    float Az0=*az0;
   
    // thita=acos(Az0/sqrt(Ax0*Ax0+Ay0*Ay0+Az0*Az0));
    // s1=(Ax0*Ax0+Ay0*Ay0)==0 ? sqrt(2)/2 : Ay0/sqrt(Ax0*Ax0+Ay0*Ay0),s2=sin(thita);
    // c1=(Ax0*Ax0+Ay0*Ay0)==0 ? sqrt(2)/2 : Ax0/sqrt(Ax0*Ax0+Ay0*Ay0),c2=cos(thita);
    // A[0][0]=c2+s1*s1*(1-c2),A[0][1]=-s1*c1*(1-c2),A[0][2]=s2*c1;
    // A[1][0]=-s1*c1*(1-c2),A[1][1]=c2+c1*c1*(1-c2),A[1][2]=s2*s1;
    // A[2][0]=-s2*c1,A[2][1]=-s2*s1,A[2][2]=c2;
    *flag_acce=1;
    EEPROM.put(0,Ax0);
    EEPROM.put(4,Ay0);
    EEPROM.put(8,Az0);
    EEPROM.commit();
    norm0=sqrt(Ax0*Ax0+ Ay0 * Ay0 + Az0 * Az0);
    ACC_ERROR_THRESHOLD=0.02*norm0;
  }
  // }
}
void AverageGyro(float gx,float gy,float gz,int* flag_gyro,float* gx0,float* gy0,float* gz0){
 
  sum_gx+=gx;
  sum_gy+=gy;
  sum_gz+=gz;
  number_gyro+=1;
  if(number_gyro==SAMPLE_GYRO){
    *gx0=sum_gx/SAMPLE_GYRO;
    *gy0=sum_gy/SAMPLE_GYRO;
    *gz0=sum_gz/SAMPLE_GYRO;
    *flag_gyro=1;
    EEPROM.put(12,*gx0);
    EEPROM.put(16,*gy0);
    EEPROM.put(20,*gz0);
    EEPROM.commit();
  }
  // }
}
/*
void AverageGyro(float gx,float gy,float gz){
  float gyro_con=0.5f;
 
  if(number_gyro==0){
    while(abs(gx)>=1.0 || abs(gy)>=1.0 || abs(gz)>=1.0 || gyro_stable_number<=20){
      gx_fix=abs(gx)<1.0 ? 1 : 0;
      gy_fix=abs(gy)<1.0 ? 1 : 0;
      gz_fix=abs(gz)<1.0 ? 1 : 0;
      
      if(abs(gx)<1.0 && abs(gy)<1.0 && abs(gz)<1.0){
        gyro_stable_number++;
        // Serial.printf("%f %f\n",pitch,roll);
      }
      else{
        gyro_stable_number=0;
  
        number_gyro+=1;
        if(gx_fix==0){
        gx_offset=gx;
        }
        if(gy_fix==0){
        gy_offset=gy;
        }
        if(gz_fix==0){
        gz_offset=gz;
        }
        Serial.printf("%f %f %f\n",gx_offset,gy_offset,gz_offset);
      }
    }
  }
}
*/
/*
void AverageAcceleration(){
 
  float ax, ay, az;
    
  int16_t rawAX,rawAY,rawAZ;
  // digitalWrite(AT7456_CS, HIGH); 
  // if(xSemaphoreTake(spiMutex,portMAX_DELAY)==pdTRUE){
  digitalWrite(ICM_CS_PIN, LOW);

  SPI.transfer(0x1F | 0x80); // 最高位=1（读操作）
  // 读取加速度计数据
  rawAX = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
  rawAY = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
  rawAZ = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
  

  digitalWrite(ICM_CS_PIN, HIGH);

  // ±4g量程转换 (8192 LSB/g)
  const float accelScale = 0.00119695f; // 9.80665 / 8192
  ax = (rawAX * accelScale) ;
  ay = (rawAY * accelScale) ;
  az = (rawAZ * accelScale) ;
  // sum_ax0+=ax;
  // sum_ay0+=ay;
  // sum_az0+=az;
  // number_acce+=1;
  
  ax0=ax;
  ay0=ay;
  az0=az;
  // ax0=0.0;
  // alfa=atan2(ay0,ax0);
  // thita=atan2(sqrt(ax0*ax0+ay0*ay0),az0);
  thita=acos(az0/sqrt(ax0*ax0+ay0*ay0+az0*az0));
  s1=(ax0*ax0+ay0*ay0)==0 ? sqrt(2)/2 : ay0/sqrt(ax0*ax0+ay0*ay0),s2=sin(thita);
  c1=(ax0*ax0+ay0*ay0)==0 ? sqrt(2)/2 : ax0/sqrt(ax0*ax0+ay0*ay0),c2=cos(thita);
  A[0][0]=c2+s1*s1*(1-c2),A[0][1]=-s1*c1*(1-c2),A[0][2]=s2*c1;
  A[1][0]=-s1*c1*(1-c2),A[1][1]=c2+c1*c1*(1-c2),A[1][2]=s2*s1;
  A[2][0]=-s2*c1,A[2][1]=-s2*s1,A[2][2]=c2;
  

}
*/
  