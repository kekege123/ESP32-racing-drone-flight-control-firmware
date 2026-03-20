#include <Arduino.h>
#include <Wire.h>
// #include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
// #include <Adafruit_BMP280.h>
// #include <Adafruit_VL53L0X.h>
#include <WiFi.h>
#include <SPI.h> 
#include "soc/rtc_wdt.h"    // RTC看门狗
#include "esp_task_wdt.h"   // 任务看门狗
#include <SPI.h>
#include "OSD_ESP32.h"
#include <ICM42688.h>
#include <math.h>
#include <ESP32_DSHOT.h>
#include "Angle_Mode.h"
#include "SD_SPI.h"
#include <EEPROM.h>
// #include "cdbus_gui.h"
// CDBusGUI gui;
#pragma pack(push, 1)  // 1字节对齐，确保无填充字节
typedef struct {
    uint8_t sync_byte;     // 同步头：0xAA
    float gyro_x;          // X轴角速度 (rad/s)
    float gyro_y;          // Y轴角速度 (rad/s)
    float gyro_z;          // Z轴角速度 (rad/s)
} imu_frame_t;
#pragma pack(pop)         // 恢复默认对齐
imu_frame_t imu_data = {0xAA, 0.0f, 0.0f, 0.0f}; 
SPIClass hspi(HSPI);
#define HSPI_SCK   14
#define HSPI_MISO  12
#define HSPI_MOSI  13
MySD mySD(hspi);
MyAT7456 myAT7456(hspi);
/*

// 陀螺仪数据文件
File dataFile;

// 定义挂载点，默认为"/sdcard"
#define MOUNT_POINT "/sdcard"
*/
//互斥锁
SemaphoreHandle_t spiMutex;
SemaphoreHandle_t spiMutex_icm;
// 定义接收机，gps，灯引脚

#define RX_PIN 33        // 重映射后的UART RX引脚
#define TX_PIN 22       // 重映射后的UART TX引脚
#define beep_pin 4
// 定义GPS使用的串口（ESP32通常使用Serial1或Serial2）
#define GpsSerial Serial2  // 使用Serial2连接GPS模块
#define DebugSerial Serial // 使用Serial用于调试输出
// #define RATE 595
#define RATE 595
#define SAMPLE_VOL 100
#define DATA_VERSION 0//等于1代表重新校准并写入存储器，等于0代表不校准直接读取存储器
// CRGB leds[NUM_LEDS];
int beep_act=0;
bool cali_finish=false;
int beep_seq[5]={0};//[0][2][4]为启用蜂鸣器
struct GpsDataStruct//定义了一个匿名结构体，直接实例化为Save_data，全局变量，是合法的
{
  uint16_t year;
  uint8_t month,day,hour,minute,second;
  float height_m,gspeed_kmh,heading_deg;
  double log,lat;
  double hAcc,cAcc,speedAcc,headAcc;//水平精度（m），垂直精度（m），速度精度（km/h），朝向精度（deg）
  int numSV;
  char timeStr[20]= {0};
};
GpsDataStruct Gps_Data;
struct HomePoint{
  double lat,log;
  bool isSet=false;
};
HomePoint home;
const int MIN_SATELLITES=5;
const double MAX_hAcc=2.0;//小于2.0表示精度较好
const double EARTH_RADIUS=6371000.0;//地球半径，米
double distToHome=0;//米
double arrowAngle=0;//0~360°
double last_lat=0,last_log=0;
int arrow_valid=0;


unsigned long last_video_check;
const unsigned int gpsRxBufferLength = 600;
char gpsRxBuffer[gpsRxBufferLength];
// unsigned int ii = 0;
const int LED_PIN = 2;    // ESP32通常使用GPIO2作为内置LED
// const int FlowingLightPin=33;
// int brightness = 0;
bool increasing = true;
const int PilotLightPin=32;
const int VOLTAGE_PIN = 34; // 电压检测模块连接的引脚是D34/GPIO34)
const float VOLTAGE_DIVIDER_RATIO = 5.0;  // 分压比为1:5
const float ADC_REF_VOLTAGE = 3.3;       // ESP32 ADC参考电压(通常3.3V)
const int ADC_RESOLUTION = 4095;         // ESP32 ADC为12位(0-4095)
float voltage=25.0;
float vols[SAMPLE_VOL]={25.0};
float sumvol=voltage*SAMPLE_VOL;
int num_vol=0;
int vol_lock=0;
//char voltageStr[10];
void setupHSPI();
void calibrateGyro();
void calibrateAccel();
void adjustMotors(float gx, float gy,float gz);
int findMin(int arr[], int size);
// void adjustMotors_Angle(float gx,float gy,float gz);
float getExpo(float Rate,float mid);
float getBaserate(uint16_t rawSpeed,float stick_min,float stick_max,float rcRate,float Rate,float rcExpo,float mid);
// float max(float a,float b);
// void errorLog(int num);
// void printGpsBuffer();
// void parseGpsBuffer();
// void gpsRead();
// void clrGpsRxBuffer();
void parseNAV_PVT(uint8_t* buffer);
double calculateDistance(double lat1,double log1,double lat2,double log2);
double calculateBearing(double lat1,double log1,double lat2,double log2);
double convertToDecimalDegrees(const char* degreeMinute, char direction);
float readPreciseVoltage();
void rainbowWave() ;
// void runningLight(CRGB color, int speed);
// void breathingEffect(CRGB color);
void randomTwinkle();
void handlePairingSequence();
void readReceiverData();
uint8_t crc8(uint8_t *data, int len);
void parseFrame(uint8_t* frame, int length);
void processChannelData();
void initAT7456E();
void clearOSD();
void writeChar(uint8_t row, uint8_t col, char c);
void writeString(uint8_t row, uint8_t col, const char* str);
void cleanUpSD();

const char* ssid = "NodeMCU_flight_AP";
const char* password = "12345678";
const char* host = "192.168.4.1";
const uint16_t port = 8080;
WiFiServer server(port);
String lock="lock"; 


byte version=0;
float gx0=0.0f,gy0=0.0f,gz0=0.0f;
int flag_gyro=0;

const int calibrationSamples = 100;

// 定义PWM通道、引脚和频率
#define LEDC_CHANNEL_0     0  // PWM通道0
#define LEDC_CHANNEL_1     1  // PWM通道1
#define LEDC_CHANNEL_2     2  // PWM通道2
#define LEDC_CHANNEL_3     3  // PWM通道3
// #define LEDC_TIMER_BIT     8  // 8位分辨率
// #define LEDC_BASE_FREQ     400 // 50 Hz（电调通常使用50 Hz的PWM信号）

// 定义四个电机的引脚
#define MOTOR_1_PIN        25 // 原GPIO 27（连接电调1的信号线）
#define MOTOR_2_PIN        26 // 原GPIO 14（连接电调2的信号线）
#define MOTOR_3_PIN        27 // 原GPIO 12（连接电调3的信号线）
#define MOTOR_4_PIN        32 // 原GPIO 13（连接电调4的信号线）
DSHOT motor1;
DSHOT motor2;
DSHOT motor3;
DSHOT motor4;

// PWM占空比范围
#define THROTTLE_MIN       0//102
#define THROTTLE_MAX       1900
// #define THROTTLE_MIN       50//102
// #define THROTTLE_MAX       1900
// #define PRE_PWM            139
#define SAMPLE_COUNT 20
#define SAMPLE_COUNT2 20
#define SAMPLE_COUNT3 5
#define GYRO_CYCLE 40
#define GPS_CYCLE 6


// 通道数据结构
struct {
  uint16_t throttle;
  uint16_t yaw;
  uint16_t pitch;
  uint16_t roll;
  uint16_t aux1;
  uint16_t aux2;
  uint16_t aux3;
  uint16_t aux4;
} channels;
float ax0=0.0,ay0=0.0,az0=9.55;
float norm,error;
int flag_acce=0;
int throttles[4]={0,0,0,0};
//角速度文件变量
float gx_record[GYRO_CYCLE]={0},gy_record[GYRO_CYCLE]={0},gz_record[GYRO_CYCLE]={0};
int gyro_record_count=0;
int gyro_record_index=0;
GpsDataStruct gps_record[GPS_CYCLE]{};
int gps_record_count=0;
int gps_record_index=0;
// const float gx_max = 180.0f; // 最大横滚角速度（°/s）
// const float gy_max = 180.0f; // 最大俯仰角速度（°/s）
// const float gz_max = 180.0f; // 最大偏航角速度（°/s）
float rcRate_x=1.0f,rcRate_y=1.0f,rcRate_z=1.0f;
float Rate_x=0.7f,Rate_y=0.7f,Rate_z=0.7f;
float rcExpo_x=0.0f,rcExpo_y=0.0f,rcExpo_z=0.0f;
float mid=0.5;
const float aux_min = 0.0f;   // 辅助通道最小值
const float aux_max = 1023.0f; // 辅助通道最大值
const uint8_t UNLOCK_AUX_CHANNEL = 5;   // 解锁通道（对应AUX1）
const uint8_t RESET_INTEGRAL_AUX_CHANNEL = 6; // 重置积分通道（对应AUX2）
int mode=1;
String temp_lock="lock";
String temp="";
int flag_unlock=1;
int flag_calibrate=0;
unsigned long lastPrint = 0;
// const uint32_t cycle = 10; // 10ms周期
// uint32_t lastRun = millis();

// const TickType_t xFrequency = pdMS_TO_TICKS(1);
// float distances[SAMPLE_COUNT] = {0}; // 初始化为0
// int validSamples = 0; // 有效样本计数器
// 高通滤波状态变量
// float axHP = 0, ayHP = 0, azHP = 0;
// const float highPassAlpha = 0.95f; // 对应于0.63Hz高通滤波
// 四元数相关变量
/*
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数
float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;   // 积分误差
float Kp = 1.6f, Ki = 0.0002f;                     // PI 控制器参数
float halfT = 0.0002f;                             // 采样周期的一半
*/
//电机PID控制
// PID控制器参数

// float Kp_gx_1 = 45.0f, Ki_gx_1 =80.0f, Kd_gx_1 =30.0f, FF_gainx_1=120.0f;//Ki_gx = 80.0f,kp=0.2； Ki_gx = 0.004f,kp=0.5
// float Kp_gy_1 = 45.0f, Ki_gy_1 =80.0f, Kd_gy_1 =34.0f, FF_gainy_1=125.0f;//Ki_gx = 84.0f,kp=0.2；Ki_gy = 0.004f,kp=0.5
// float Kp_gz_1 = 45.0f, Ki_gz_1 =80.0f, Kd_gz_1 = 0.0f, FF_gainz_1=120.0f;//Ki_gx = 80.0f,kp=0.3；Ki_gz = 0.002f,kp=1.0
// float dt_1=9000.0;
//角速度PID参数，和手动模式用不同的一套参数，避免串参
// float Kp_gx_2 = 30.0f, Ki_gx_2  =60.0f, Kd_gx_2  =5.0f, FF_gainx_2 =120.0f;//Ki_gx = 80.0f,kp=0.2； Ki_gx = 0.004f,kp=0.5
// float Kp_gy_2  =30.0f, Ki_gy_2  =60.0f, Kd_gy_2  =5.0f, FF_gainy_2 =125.0f;//Ki_gx = 84.0f,kp=0.2；Ki_gy = 0.004f,kp=0.5
// float Kp_gz_2  =30.0f, Ki_gz_2  =60.0f, Kd_gz_2  = 0.0f, FF_gainz_2 =120.0f;
// float dt_2=6600.0;
// float Kp_gx = 82.0f, Ki_gx =128.0f, Kd_gx =65.0f, FF_gainx=120.0f;//Ki_gx = 80.0f,kp=0.2； Ki_gx = 0.004f,kp=0.5
// float Kp_gy = 75.0f, Ki_gy =128.0f, Kd_gy =57.0f, FF_gainy=125.0f;//Ki_gx = 84.0f,kp=0.2；Ki_gy = 0.004f,kp=0.5
// float Kp_gz = 78.0f, Ki_gz =125.0f, Kd_gz = 0.0f, FF_gainz=120.0f;//Ki_gx = 80.0f,kp=0.3；Ki_gz = 0.002f,kp=1.0
float Kp_gx = 55.0f, Ki_gx =85.0f, Kd_gx =45.0f, FF_gainx=120.0f;//Ki_gx = 80.0f,kp=0.2； Ki_gx = 0.004f,kp=0.5
float Kp_gy = 50.0f, Ki_gy =80.0f, Kd_gy =40.0f, FF_gainy=125.0f;//Ki_gx = 84.0f,kp=0.2；Ki_gy = 0.004f,kp=0.5
float Kp_gz = 80.0f, Ki_gz =90.0f, Kd_gz = 0.0f, FF_gainz=120.0f;//Ki_gx = 80.0f,kp=0.3；Ki_gz = 0.002f,kp=1.0
// float pid_dt=9000.0;
double T=1000.0/8.0,last_T=1000.0/8.0;
float alpha_T=0.8;

float pitch=0.0f, roll=0.0f, yaw=0.0f;

// float gxs[SAMPLE_COUNT]={0},gys[SAMPLE_COUNT]={0},gzs[SAMPLE_COUNT]={0};
// float sumgx=0,sumgy=0,sumgz=0;
// int num=0;
float lastgx=0,lastgy=0,lastgz=0;
float alpha_gx=0.85,alpha_gy=0.85,alpha_gz=0.85;
float differ_gx=0,differ_gy=0,differ_gz=0;
float last_gxoutput=0,last_gyoutput=0,last_gzoutput=0;
float lastax=0,lastay=0,lastaz=0;
float alpha_ax=0.85,alpha_ay=0.85,alpha_az=0.85;
float differ_ax=0,differ_ay=0,differ_az=0;
float last_axoutput=0,last_ayoutput=0,last_azoutput=0;
// float axs[SAMPLE_COUNT2]={0},ays[SAMPLE_COUNT2]={0},azs[SAMPLE_COUNT2]={0};
// float sumax=0,sumay=0,sumaz=0;
// int num2=0;


// PID控制器变量
float gx_error = 0.0f, gy_error = 0.0f,gz_error = 0.0f;
float gx_error_integral = 0.0f, gy_error_integral = 0.0f,gz_error_integral = 0.0f;
float gx_error_derivative = 0.0f, gy_error_derivative = 0.0f,gz_error_derivative = 0.0f;
float previous_gx = 0.0f, previous_gy = 0.0f, previous_gz = 0.0f;
float previous_targetgx = 0.0f, previous_targetgy = 0.0f, previous_targetgz = 0.0f;
float previous_inputx = 992.0f, previous_inputy = 992.0f, previous_inputz = 992.0f;
float filtered_gx_diff = 0.0f, filtered_gy_diff = 0.0f, filtered_gz_diff = 0.0f;
float d_weight=0.85f;

// 目标角速度
float target_gx = 0.0f; // 单位°/s
float target_gy = 0.0f;  // 
float target_gz = 0.0f;  //
// 电机基础转速

int base_pwm=THROTTLE_MIN;
// 创建第二个SPI实例（HSPI）
// SPIClass hspi(HSPI);
// void setupSPI() {
//   // pinMode(AT7456_CS, OUTPUT);
//   // digitalWrite(AT7456_CS, HIGH); // CS 高电平不选中
//   pinMode(ICM_CS_PIN, OUTPUT);
//   digitalWrite(ICM_CS_PIN, HIGH);  // 初始禁用
//   SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
//   SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0)); // 10MHz时钟
//   // hspi.begin(PIN_SCK_H, PIN_MISO_H, PIN_MOSI_H); // 片选引脚通常由用户控制，此处可不传入
//   // // 为HSPI设备设置通信参数（10MHz, 模式0, MSB优先）
//   // hspi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
// }
/*
// 四元数更新函数
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    //将陀螺仪数据先转换为弧度每秒再输入姿态解算函数：
    gx = gx * M_PI / 180.0f;
    gy = gy * M_PI / 180.0f;
    gz = gz * M_PI / 180.0f;
    // 归一化加速度计数据
    norm = sqrt(ax * ax + ay * ay + az * az);
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
  
    // 积分误差
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;
  
    // 修正陀螺仪数据
    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;
   
    // 四元数微分方程
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
*pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; // 俯仰角
*roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // 横滚角
*yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3; // 航向角
}
*/
WiFiClient client;
// int throttle = 50;
// int dir = 1;


void task1(void *pvParameters) {
  while(1){
    // rtc_wdt_feed();           // 重置RTC看门狗
    // esp_task_wdt_reset();     // 重置任务看门狗

    
    //将WIFI传输功能和电机控制割开，是为了更好维护代码，并保证二者不相互影响地工作，仅仅产生工作交流
    // if(server.hasClient() && !client.connected()){//避免有干扰请求覆写client
    //     client = server.available(); 
    // }
    // else {
    //   if(client.connected() && client.available() >= 1){
    //       uint8_t cmd = client.read();
    //       switch (cmd) {
    //         case 0x01: lock = "lock";temp_lock="lock";break;
    //         case 0x02: lock = "unlock"; temp_lock="unlock";break;
    //         case 0x03: gx_error_integral=0;gy_error_integral=0; gz_error_integral=0; break;
    //         default:  Serial.print("未知指令: 0x");Serial.println(cmd, HEX); break; // 记录错误指令
    //       }
    //   }
    // }
    if(client.connected()){
      uint8_t cmd = client.read();
      switch (cmd) {
        case 0x01: lock = "lock";temp_lock="lock";break;
        case 0x02: lock = "unlock"; temp_lock="unlock";break;
        case 0x03: gx_error_integral=0;gy_error_integral=0; gz_error_integral=0; break;
        //default:  Serial.print("未知指令: 0x");Serial.println(cmd, HEX); break; // 记录错误指令
      }
    }
    else if(server.hasClient()){
      client = server.available(); 
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
// unsigned long timer_pwm=millis();
void task2(void *pvParameters) {
  // Serial.print("Task2 running on core: ");
  //   Serial.println(xPortGetCoreID()); // 输出0或1
 
  unsigned long number_time=millis();
  int number_act=0;
  TickType_t xLastWakeTime = xTaskGetTickCount(); 
  const TickType_t xFrequency = pdMS_TO_TICKS(1);
  
  // 初始化RPM滤波器变量
  const float rpmFilterCutoff = 0.5f; // 截止频率(Hz)，根据电机转速调整
  const float rpmFilterBeta = 0.1f;   // 滤波器系数
  float gxFiltered = 0.0f, gyFiltered = 0.0f, gzFiltered = 0.0f;
  float gxPrev = 0.0f, gyPrev = 0.0f, gzPrev = 0.0f;
  // 电机转速估计(假设基础PWM与转速成正比)
  float motorRpmEstimate = 0.0f;
  const float rpmScale = 0.1f; // PWM到RPM的缩放因子
  unsigned long last_micro=micros();
  // 执行陀螺仪校准
  while(1){
 
    // rtc_wdt_feed();           // 重置RTC看门狗
    // esp_task_wdt_reset();     // 重置任务看门狗
    float ax, ay, az,gx,gy,gz;
    
    int16_t rawAX,rawAY,rawAZ,rawGX,rawGY,rawGZ;
    // digitalWrite(AT7456_CS, HIGH); 
    if(xSemaphoreTake(spiMutex_icm,portMAX_DELAY)==pdTRUE){
      digitalWrite(ICM_CS_PIN, LOW);

      SPI.transfer(0x1F | 0x80); // 最高位=1（读操作）
      // 读取加速度计数据
      rawAX = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
      rawAY = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
      rawAZ = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
      
      // 读取陀螺仪数据
      rawGX = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
      rawGY = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
      rawGZ = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
      digitalWrite(ICM_CS_PIN, HIGH);
      xSemaphoreGive(spiMutex_icm);
    }
    // ±4g量程转换 (8192 LSB/g)
    const float accelScale = 0.00119695f; // 9.80665 / 8192
    ax = (rawAX * accelScale) ;
    ay = (rawAY * accelScale) ;
    az = (rawAZ * accelScale) ;
   
    // ax=ax*A[0][0]+ay*A[1][0]+az*A[2][0];
    // ay=ax*A[0][1]+ay*A[1][1]+az*A[2][1];
    // az=ax*A[0][2]+ay*A[1][2]+az*A[2][2];

    // ±500°/s量程转换
    const float gyroScale = 0.015267176f; // 1/65.5
    gx = (rawGX * gyroScale) ;
    gy = (rawGY * gyroScale) ;
    gz = (rawGZ * gyroScale) ;
    // raw_gx=gx;
    // raw_gy=gy;
    // raw_gz=gz;
    // update_raw=1;
    // gx = (rawGX * gyroScale) -gx_offset;
    // gy = (rawGY * gyroScale) -gy_offset;
    // gz = (rawGZ * gyroScale) -gz_offset;
    // Serial.printf("%f %f %f\n",ax,ay,az);
    
    // Serial.printf("%f %f %f \n",gx,gy,gz);
    
    // gx_p=gx;
    // gy_p=gy;
    /*
    if(mode==2){
      IMUupdate(gx, gy, gz, ax, ay, az);
      calculateEulerAngles(&pitch, &roll, &yaw);
      // Serial.printf("%f,%f,%f\n",pitch,roll,yaw);
      // pitch=NAN;
      // roll=NAN;
      if(isnan(pitch) || isnan(roll)){
        motor1.set(0);
        motor2.set(0);
        motor3.set(0);
        motor4.set(0); 
        lock="lock";
        continue;
      }
      getTargetGyro(pitch,roll);
      // Serial.printf("%f %f %f \n",gx,gy,gz);
      // Serial.printf("%f,%f\n",target_gx,target_gy);
    }
    */
    /*
    //加速度的滑动平均
    sumax-=axs[num2];
    sumay-=ays[num2];
    sumaz-=azs[num2];
    axs[num2]=ax;
    ays[num2]=ay;
    azs[num2]=az;
    sumax+=axs[num2];
    sumay+=ays[num2];
    sumaz+=azs[num2];
    ax=sumax/SAMPLE_COUNT2;
    ay=sumay/SAMPLE_COUNT2;
    az=sumaz/SAMPLE_COUNT2;
    num2=++num2%SAMPLE_COUNT2;
    */
    // Serial.printf("%f %f %f ",ax,ay,az);
    if (channels.aux2 ==997 && flag_acce==0 && sqrt(ax*ax+ay*ay)/az<0.05) {
      AverageAcceleration(ax,ay,az,&flag_acce,&ax0,&ay0,&az0);
    }
    if(flag_acce==1){
      float thita=acos(az0/sqrt(ax0*ax0+ay0*ay0+az0*az0));
      float s1=(ax0*ax0+ay0*ay0)==0 ? sqrt(2)/2 : ay0/sqrt(ax0*ax0+ay0*ay0),s2=sin(thita);
      float c1=(ax0*ax0+ay0*ay0)==0 ? sqrt(2)/2 : ax0/sqrt(ax0*ax0+ay0*ay0),c2=cos(thita);//s1=sin(alfa),s2=sin(thita),c1=cos(alfa)
      A[0][0]=c2+s1*s1*(1-c2),A[0][1]=-s1*c1*(1-c2),A[0][2]=s2*c1;
      A[1][0]=-s1*c1*(1-c2),A[1][1]=c2+c1*c1*(1-c2),A[1][2]=s2*s1;
      A[2][0]=-s2*c1,A[2][1]=-s2*s1,A[2][2]=c2;
      ax=ax*A[0][0]+ay*A[1][0]+az*A[2][0];
      ay=ax*A[0][1]+ay*A[1][1]+az*A[2][1];
      az=ax*A[0][2]+ay*A[1][2]+az*A[2][2];
    }
    norm = sqrt(ax * ax + ay * ay + az * az);
    error=fabs(norm-norm0);
    /*
    //加速度两点滤波
    differ_ax=ax-lastax;
    differ_ay=ay-lastay;
    differ_az=az-lastaz;
    alpha_ax=1-0.2/(1.0+abs(differ_ax));
    alpha_ay=1-0.2/(1.0+abs(differ_ay));
    alpha_az=1-0.2/(1.0+abs(differ_az));
    float ax_output=alpha_ax*last_axoutput+(1-alpha_ax)*ax;
    float ay_output=alpha_ay*last_ayoutput+(1-alpha_ay)*ay;
    float az_output=alpha_az*last_azoutput+(1-alpha_az)*az;
    last_axoutput=ax_output;
    last_ayoutput=ay_output;
    last_azoutput=az_output;
    lastax=ax;
    lastay=ay;
    lastaz=az;
    ax=ax_output;
    ay=ay_output;
    az=az_output;
    */
    // Serial.printf("%f %f %f\n",ax,ay,az);
    
    /*
    //角速度的滑动平均
    sumgx-=gxs[num];
    sumgy-=gys[num];
    sumgz-=gzs[num];
    gxs[num]=gx;
    gys[num]=gy;
    gzs[num]=gz;
    sumgx+=gxs[num];
    sumgy+=gys[num];
    sumgz+=gzs[num];
    gx=sumgx/SAMPLE_COUNT;
    gy=sumgy/SAMPLE_COUNT;
    gz=sumgz/SAMPLE_COUNT;
    num=++num%SAMPLE_COUNT;
    // gx-=0.779;
    // gy-=-0.211;
    // gz-=-0.364;
    */
    // Serial.printf("%f %f %f\n",gx,gy,gz);
    if (channels.aux2 ==997 && flag_gyro==0 && abs(gx)<1.0 && abs(gy)<1.0 && abs(gz)<1.0) {
      AverageGyro(gx,gy,gz,&flag_gyro,&gx0,&gy0,&gz0);
    }
    if(flag_gyro==1){
      gx-=gx0;
      gy-=gy0;
      gz-=gz0;
    }
    /*
    //角速度两点滤波
    differ_gx=gx-lastgx;
    differ_gy=gy-lastgy;
    differ_gz=gz-lastgz;
    alpha_gx=1-5/(25+abs(differ_gx));
    alpha_gy=1-5/(25+abs(differ_gy));
    alpha_gz=1-5/(25+abs(differ_gz));
    float gx_output=alpha_gx*last_gxoutput+(1-alpha_gx)*gx;
    float gy_output=alpha_gy*last_gyoutput+(1-alpha_gy)*gy;
    float gz_output=alpha_gz*last_gzoutput+(1-alpha_gz)*gz;
    last_gxoutput=gx_output;
    last_gyoutput=gy_output;
    last_gzoutput=gz_output;
    lastgx=gx;
    lastgy=gy;
    lastgz=gz;
    gx=gx_output;
    gy=gy_output;
    gz=gz_output;
    // Serial.printf("%f %f %f\n",gx,gy,gz);
    */
    
    /*
    if (channels.aux2 ==997){
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
            AverageGyro();
            Serial.printf("%f %f %f\n",gx_offset,gy_offset,gz_offset);
          }
        }
      }
    }
    */

    gyro_record_count++;
    if(gyro_record_count==500){
      if(gyro_record_index<GYRO_CYCLE){
        gx_record[gyro_record_index]=gx;
        gy_record[gyro_record_index]=gy;
        gz_record[gyro_record_index]=gz;
        gyro_record_index++;
      }
      
      gyro_record_count=0;
    }
    /*
    motorRpmEstimate = (throttles[0] + throttles[1] + throttles[2] + throttles[3]) / 4.0f * rpmScale;
    float rpmFilterAlpha = rpmFilterCutoff / (rpmFilterCutoff + motorRpmEstimate * rpmFilterBeta);
    rpmFilterAlpha = constrain(rpmFilterAlpha, 0.01f, 0.99f);
    gxFiltered = rpmFilterAlpha * gx + (1.0f - rpmFilterAlpha) * gxPrev;
    gyFiltered = rpmFilterAlpha * gy + (1.0f - rpmFilterAlpha) * gyPrev;
    gzFiltered = rpmFilterAlpha * gz + (1.0f - rpmFilterAlpha) * gzPrev;
    gxPrev = gxFiltered;
    gyPrev = gyFiltered;
    gzPrev = gzFiltered;
    gx = gxFiltered;
    gy = gyFiltered;
    gz = gzFiltered;
 
    */
    //去掉归零限幅之后，明显姿态漂移减小了，效果是比较好的，但是还存在姿态倾角较大，积分相差大的时候，会震荡，不稳定。
  
    // Serial.printf("%f %f %f \n",gx,gy,gz);
  
    
    if(mode==2){
      IMUupdate(gx, gy, gz, ax, ay, az,norm,error,T);
      calculateEulerAngles(&pitch, &roll, &yaw);
      // Serial.printf("%f %f %f\n",pitch,roll,yaw);
      if(isnan(pitch) || isnan(roll)){
        motor1.set(0);
        motor2.set(0);
        motor3.set(0);
        motor4.set(0); 
        lock="lock";
        continue;
      }
      getTargetGyro(pitch,roll,&target_gx,&target_gy);
     
    }
    
    adjustMotors(gx, gy,gz);
    /*
    if(mode==1){
      adjustMotors(gx, gy,gz);//一定要放在外面这里，否则电机响应会延迟
    }
    if(mode==2){
      adjustMotors_Angle(gx, gy,gz);
    }
    */
    number_act+=1;
    // Serial.printf("%f %f %f\n",gx,gy,gz);
    if(cali_finish==false && flag_acce==1 && flag_gyro==1){
      beep_act=1;
      cali_finish=true;
      beep_seq[0]=200;
      beep_seq[1]=100;
      beep_seq[2]=200;
      beep_seq[3]=100;
      beep_seq[4]=200;
    }
    if (lock == "lock" ) {
      // 设置所有电机为最小PWM值（停转）
      
      motor1.set(0);
      motor2.set(0);
      motor3.set(0);
      motor4.set(0);        
    }
    else{
      
      // throttles[0]=throttles[0]>1500 ? 1500 :throttles[0];
      // throttles[1]=throttles[1]>1500 ? 1500 :throttles[1];
      // throttles[2]=throttles[2]>1500 ? 1500 :throttles[2];
      // throttles[3]=throttles[3]>1500 ? 1500 :throttles[3];
      
      motor1.set(throttles[0]);
      motor2.set(throttles[1]);
      motor3.set(throttles[2]);
      motor4.set(throttles[3]);
     
      // Serial.printf("%d %d %d %d\n",throttles[0],throttles[1],throttles[2],throttles[3]);
      
    }   
    // Serial.printf("%d %d %d %d\n",throttles[0],throttles[1],throttles[2],throttles[3]);
   
    
    T=(alpha_T*last_T+(1-alpha_T)*(micros()-last_micro))/1000000;
    last_T=T;
    last_micro=micros();
    if(millis()-number_time>5000){
      // Serial.println(number_act/5);
      // Serial.printf("%f %f %f\n",pitch,roll,yaw);
      // Serial.printf("%f %f %f %f %f %f\n",ax,ay,az,gx,gy,gz);
      // Serial.printf("%f %f %f %d\n",ax,ay,az,flag_acce);
      // Serial.printf("%d %d %d %d\n",throttles[0],throttles[1],throttles[2],throttles[3]);
      // Serial.println(getCpuFrequencyMhz());
      // Serial.printf("%f,%f,%f\n",gx,gy,gz);
      // imu_data.gyro_x=gx;
      // imu_data.gyro_y=gy;
      // imu_data.gyro_z=gz;
      // Serial.write((const uint8_t*)&imu_data, sizeof(imu_frame_t));
      // Serial.printf("%f %f %f,%f %f %f\n",ax0,ay0,az0,gx0,gy0,gz0);
      number_time=millis();
      number_act=0;
    }
    // Serial.println(1);
   
    // delayMicroseconds(30);
    // vTaskDelayUntil(&xLastWakeTime, xFrequency); // 精确延迟到下一个周期点
    
  }
}
void task3(void *pvParameters) {
  // unsigned long volTimer=millis();
  int number=0;
  int start=0;
  int k=0;
  uint8_t gpsBuffer[96];  // 足够的缓冲区大小,只存储payload和之后的数据
  int gpsbufferIndex = 0;
  uint8_t gpsbyte[4];
  // gpsBuffer[0]=0xB5,gpsBuffer[1]=0x62,gpsBuffer[2]=0x01,gpsBuffer[3]=0x07;
  TickType_t xLastWakeTime = xTaskGetTickCount(); 
  const TickType_t xTickTime = pdMS_TO_TICKS(1);
  // Serial.print("Task3 running on core: ");
  //   Serial.println(xPortGetCoreID()); // 输出0或1
  while(1){
    //Serial.print("4");
    // rtc_wdt_feed();           // 重置RTC看门狗
    // esp_task_wdt_reset();     // 重置任务看门狗
    // Serial.println(3);
    voltage = readPreciseVoltage();
    // Serial.println(voltage);
    // sumvol-=vols[num_vol];
    // vols[num_vol]=voltage;
    // sumvol+=vols[num_vol];
    // voltage=sumvol/SAMPLE_VOL;
    // num_vol=++num_vol%SAMPLE_VOL;
    if(voltage<23.3 && voltage>10.0){
      vol_lock=1;
      // digitalWrite(beep_pin,HIGH);
      beep_act=1;
      beep_seq[0]=100;
      beep_seq[1]=0;
      beep_seq[2]=100;
      beep_seq[3]=0;
      beep_seq[4]=100;
    }
    else{
      vol_lock=0;
      // digitalWrite(beep_pin,LOW);
    }
    // if(voltage<10.0 && voltage>3.0){//低电压锁定，防止电池过放，但不建议低电压直接上锁，最好低电压bb响报警
    //   // lock="lock";
      
    // }
    // else  {
    //   // lock=temp_lock;//由于电压引起的上锁再解锁，也会引起积分重置，所以电压上锁不应该置flag_unlock=1
    // }
    
    if (GpsSerial.available() > 0) {
     
      if(start==0){
      gpsbyte[0]=gpsbyte[1];
      gpsbyte[1]=gpsbyte[2];
      gpsbyte[2]=gpsbyte[3];
      gpsbyte[3] = GpsSerial.read();
      if(gpsbyte[0]==0xB5 && gpsbyte[1]==0x62 && gpsbyte[2]==0x01 && gpsbyte[3]==0x07){
        start=1;
        GpsSerial.read();
        GpsSerial.read();
      }
      }
      else{
        if(k<96){
        gpsBuffer[k]=GpsSerial.read();
        k++;
        }
        else{
          k=0;
          start=0;
          parseNAV_PVT(gpsBuffer);
          if(!home.isSet){
            if(channels.aux2 ==191 && Gps_Data.numSV>=MIN_SATELLITES && Gps_Data.hAcc<=MAX_hAcc){
              home.lat=Gps_Data.lat;
              home.log=Gps_Data.log;
              home.isSet=true;
              last_lat=Gps_Data.lat;
              last_log=Gps_Data.log;
            }
            arrow_valid=0;
          }
          else{//起飞点锁定后，计算方位和距离
            distToHome=calculateDistance(Gps_Data.lat,Gps_Data.log,home.lat,home.log);
            double bearToHome=calculateBearing(Gps_Data.lat,Gps_Data.log,home.lat,home.log);
            //获取当前运动方向
            double currentHeading=calculateBearing(Gps_Data.lat,Gps_Data.log,last_lat,last_log);
            last_lat=Gps_Data.lat;
            last_log=Gps_Data.log;
            arrowAngle=bearToHome-currentHeading;
            if(arrowAngle<0)arrowAngle+=360;
            arrow_valid=1;
          }
          gps_record_count++;
          if(gps_record_count==1){
            if(gps_record_index<GPS_CYCLE){
              gps_record[gps_record_index]=Gps_Data;
              gps_record_index++;
            }
            
            gps_record_count=0;
          }
          // Serial.printf("%d.%d.%d\n",Gps_Data.year,Gps_Data.month,Gps_Data.day);
          // if(client.connected()){
          //   client.printf("time:%.20s\n numSV log lat height gspeed heading %d %.2f %.2f %.2f %.2f %.2f\n",Gps_Data.timeStr,Gps_Data.numSV,Gps_Data.log,Gps_Data.lat,Gps_Data.height_m,Gps_Data.gspeed_kmh,Gps_Data.heading_deg);
          // }
        } 
      }
      
      
    }
    // number++;
    // if(millis()-volTimer>1000){
    //   Serial.printf("num %d\n",number);
    //   number=0;
    //   volTimer=millis();
    // }
    // 添加10ms延时
    vTaskDelayUntil(&xLastWakeTime, xTickTime);
  }
}

// void task4(void *pvParameters){
//   while(1){
//     rtc_wdt_feed();           // 重置RTC看门狗
//     esp_task_wdt_reset();     // 重置任务看门狗
//     breathingEffect(CRGB(0, 100, 255));  // 浅蓝色
    
//     vTaskDelay(pdMS_TO_TICKS(4)); 
//   }
// }
void task5(void *pvParameters){
  TickType_t xLastWakeTime = xTaskGetTickCount(); 
  const TickType_t xTickTime = pdMS_TO_TICKS(50);
  unsigned long osd_timer=millis();
  char voltageStr[7];
  const uint8_t voltageLabel[7] = {myAT7456.charToAt7456('v'),myAT7456.charToAt7456('o'),myAT7456.charToAt7456('l'),myAT7456.charToAt7456('t'),myAT7456.charToAt7456('a'),myAT7456.charToAt7456('g'),myAT7456.charToAt7456('e')};
  // int arraySize = sizeof(voltageStr);
  // UTC时间标签
  const uint8_t timeLabel[4] = {myAT7456.charToAt7456('t'),myAT7456.charToAt7456('i'),myAT7456.charToAt7456('m'),myAT7456.charToAt7456('e')};
  // 经纬度标签
  const uint8_t latLabel[4] = {myAT7456.charToAt7456('L'),myAT7456.charToAt7456('a'),myAT7456.charToAt7456('t'),myAT7456.charToAt7456(':')};
  const uint8_t lonLabel[4] = {myAT7456.charToAt7456('L'),myAT7456.charToAt7456('o'),myAT7456.charToAt7456('n'),myAT7456.charToAt7456(':')};
  const uint8_t satLabel[4] = {myAT7456.charToAt7456('S'),myAT7456.charToAt7456('a'),myAT7456.charToAt7456('t'),myAT7456.charToAt7456(':')};
  const uint8_t heightLabel[7] = {myAT7456.charToAt7456('h'),myAT7456.charToAt7456('e'),myAT7456.charToAt7456('i'),myAT7456.charToAt7456('g'),myAT7456.charToAt7456('h'),myAT7456.charToAt7456('t'),myAT7456.charToAt7456(':')};
  const uint8_t acce_correcting[15]={0x25,0x27,0x27,0x29,0x00,0x27,0x33,0x36,0x36,0x29,0x27,0x38,0x2D,0x32,0x2B};//acce correcting
  const uint8_t gyro_correcting[15]={0x2B,0x3D,0x36,0x33,0x00,0x27,0x33,0x36,0x36,0x29,0x27,0x38,0x2D,0x32,0x2B};//gyro correcting
  const uint8_t arrow_invalid[13]={0x25,0x36,0x36,0x33,0x3B,0x00,0x2D,0x32,0x3A,0x25,0x30,0x2D,0x28};//arrow invalid
  const uint8_t clearing[15]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  const uint8_t distToHome[11]={0x28,0x2D,0x37,0x38,0x1E,0x33,0x12,0x33,0x31,0x29,0x44};//distToHome:
  const uint8_t arrowAngle[11]={0x25,0x36,0x36,0x33,0x3B,0x0B,0x32,0x2B,0x30,0x29,0x44};//arrowAngle:
  char latStr[8];
  char lonStr[8];
  char heightStr[8];
  char satStr[2]; 
  char distStr[8];
  char arrowStr[5];
  last_video_check = millis();  
  uint8_t x0=1;//行
  uint8_t y0=1;
  uint8_t blank[3]={0,0,0};
  // Serial.print("Task5 running on core: ");
  // Serial.println(xPortGetCoreID()); // 输出0或1
  while(1){
  
    // Serial.println(5);
    // if (millis() - last_video_check > 2000) {
    //     uint8_t stat = myAT7456.at7456_read_addr(STAT);
    //     if (!(stat & (PAL_DETECT | NTSC_DETECT))) {
    //         // Serial.println("定期检测到视频信号丢失");
    //         digitalWrite(AT7456_CS, HIGH); // CS 高电平不选中
    //         digitalWrite(ICM_CS_PIN, HIGH);
    //         myAT7456.at7456_init();
    //     }
    //     last_video_check = millis();
    // }
    if(millis()-osd_timer>500){
     
      // 2. 显示"voltage"标签 (位置：第一行第0-6列)
      snprintf(voltageStr, sizeof(voltageStr), "%.2fV", voltage);
      snprintf(lonStr, sizeof(lonStr), "%.2f", Gps_Data.log);
      snprintf(latStr, sizeof(latStr), "%.2f", Gps_Data.lat);
      snprintf(satStr, sizeof(satStr), "%d", Gps_Data.numSV);
      snprintf(heightStr, sizeof(heightStr), "%.2f", Gps_Data.height_m);
      snprintf(distStr, sizeof(distStr), "%.2f", distToHome);
      snprintf(arrowStr, sizeof(arrowStr), "%.1f", arrowAngle);
      if(xSemaphoreTake(spiMutex,portMAX_DELAY)==pdTRUE){
        unsigned long tim_vsync=millis();
        int vsync_get=0;
        while(millis()-tim_vsync<10 && vsync_get==0){
          if(myAT7456.at7456_read_addr(STAT) & VSYNC){
            // while(!(at7456_read_addr(STAT) )){
              
            vsync_get=1;
            for (int i = 0; i < 7; i++) {
                myAT7456.at7456_writeSRAM(x0+0, y0+i, voltageLabel[i]);
            }
            myAT7456.at7456_writeSRAM(x0+0, y0+7, 0x44);
            // 显示在OSD上 (示例)
            for (int i = 0; i < 7; i++) {
                myAT7456.at7456_writeSRAM(x0+0, y0+i+8, myAT7456.charToAt7456(voltageStr[i])); // 第一行显示
            }
            //显示UTC、卫星数、经纬度
            // 显示时间标签
            for (int i = 0; i < 4; i++) {
              myAT7456.at7456_writeSRAM(x0+1, y0+i, timeLabel[i]);
            }
            myAT7456.at7456_writeSRAM(x0+1, y0+4, 0x44);
            // char* currentUTCTime = Gps_Data.timeStr;
            for (int i = 0; i < strlen(Gps_Data.timeStr); i++) {
                myAT7456.at7456_writeSRAM(x0+1, y0+i+5, myAT7456.charToAt7456(Gps_Data.timeStr[i]));
            }
            //经纬度
            for (int i = 0; i < 4; i++) {
                myAT7456.at7456_writeSRAM(x0+2, y0+i, latLabel[i]);
                myAT7456.at7456_writeSRAM(x0+3, y0+i, lonLabel[i]);
            }
            for (int i = 0; i < strlen(latStr); i++) {
                myAT7456.at7456_writeSRAM(x0+2, y0+5+i, myAT7456.charToAt7456(latStr[i]));
            }
            for (int i = 0; i < strlen(lonStr); i++) {
                myAT7456.at7456_writeSRAM(x0+3, y0+5+i, myAT7456.charToAt7456(lonStr[i]));
            }
            // 5. 显示卫星数量 (第五行)
            for (int i = 0; i < 4; i++) {
                myAT7456.at7456_writeSRAM(x0+4, y0+i, satLabel[i]); // 显示"Sat:"标签
            }
            for (int i = 0; i < strlen(satStr); i++) {
                myAT7456.at7456_writeSRAM(x0+4, y0+5+i, myAT7456.charToAt7456(satStr[i])); // 从第5列开始显示
            }
            for (int i = 0; i < 7; i++) {
                myAT7456.at7456_writeSRAM(x0+5, y0+i, heightLabel[i]); // 显示"Sat:"标签
            }
            for (int i = 0; i < strlen(heightStr); i++) {
                myAT7456.at7456_writeSRAM(x0+5, y0+8+i, myAT7456.charToAt7456(heightStr[i])); 
            }
            if(flag_acce==0){
              for(int i=0;i<15;i++){
                myAT7456.at7456_writeSRAM(10,8+i,acce_correcting[i]);
              }
              blank[0]=1;
            }
            else{
              blank[0]=0;
              for(int i=0;i<15;i++){
                myAT7456.at7456_writeSRAM(10,8+i,0x00);
              }
            }
            if(flag_gyro==0){
              if(blank[0]==0){
                for(int i=0;i<15;i++){
                  myAT7456.at7456_writeSRAM(10,8+i,gyro_correcting[i]);
                }
                blank[0]=1;
              }
              else{
                for(int i=0;i<15;i++){
                  myAT7456.at7456_writeSRAM(11,8+i,gyro_correcting[i]);
                }
                blank[1]=1;
              }
            }
            else{
              blank[1]=0;//gyro优先级为2，只需要释放11行，如果占了10行，下一次acce如果需要占用会直接占用10行，所以gyro无需负责10行。
              for(int i=0;i<15;i++){
                myAT7456.at7456_writeSRAM(11,8+i,0x00);
              }
            }
            if(arrow_valid==0){
              if(blank[0]==0){
                for(int i=0;i<13;i++){
                  myAT7456.at7456_writeSRAM(10,9+i,arrow_invalid[i]);
                }
                blank[0]=1;
              }
              else if(blank[1]==0){
                for(int i=0;i<13;i++){
                  myAT7456.at7456_writeSRAM(11,9+i,arrow_invalid[i]);
                }
                blank[1]=1;
              }
              else{
                for(int i=0;i<13;i++){
                  myAT7456.at7456_writeSRAM(12,9+i,arrow_invalid[i]);
                }
                blank[2]=1;
              }
            }
            else{
              blank[2]=0;
              for(int i=0;i<13;i++){
                myAT7456.at7456_writeSRAM(12,9+i,0x00);
              }
              for(int i=0;i<11;i++){
                myAT7456.at7456_writeSRAM(x0+6,y0+i,distToHome[i]);
              }
              for(int i=0;i<strlen(distStr);i++){
                myAT7456.at7456_writeSRAM(x0+6,y0+11+i,myAT7456.charToAt7456(distStr[i]));
              }
              for(int i=0;i<11;i++){
                myAT7456.at7456_writeSRAM(x0+7,y0+i,arrowAngle[i]);
              }
              for(int i=0;i<strlen(arrowStr);i++){
                myAT7456.at7456_writeSRAM(x0+7,y0+11+i,myAT7456.charToAt7456(arrowStr[i]));
              }
            }
            /*
            memset(latStr, ' ', sizeof(latStr));
            memset(lonStr, ' ', sizeof(lonStr));
            
            if (Save_Data.isUsefull) {
              // 数值和单位：23.4567° N
              double lat = convertToDecimalDegrees(Save_Data.latitude, Save_Data.N_S[0]);
              snprintf(latStr, sizeof(latStr), "%.4f° %c", fabs(lat), Save_Data.N_S[0]);
              // 数值和单位：123.4567° E
              double lon = convertToDecimalDegrees(Save_Data.longitude, Save_Data.E_W[0]);
              snprintf(lonStr, sizeof(lonStr), "%.4f° %c", fabs(lon), Save_Data.E_W[0]);
            } else {
              snprintf(latStr, sizeof(latStr), "--INVALID--"); // 显示无效状态
              snprintf(lonStr, sizeof(lonStr), "--INVALID--");
            }
            for (int i = 0; i < strlen(latStr); i++) {
                at7456_writeSRAM(2, 5+i, charToAt7456(latStr[i]));
            }
            for (int i = 0; i < strlen(lonStr); i++) {
                at7456_writeSRAM(3, 5+i, charToAt7456(lonStr[i]));
            }
            */
            
            // 显示卫星数量数值
            /*
            memset(satStr, ' ', sizeof(satStr)); // 初始化为全空格
            satStr[sizeof(satStr)-1] = '\0';
            snprintf(satStr, sizeof(satStr), "%d", Save_Data.satelliteCount);
            for (int i = 0; i < strlen(satStr); i++) {
                at7456_writeSRAM(4, 5+i, charToAt7456(satStr[i])); // 从第5列开始显示
            }
            if (Save_Data.isUsefull) {
              at7456_writeSRAM(4, 9, charToAt7456('+'));  // 有效定位显示+
            } else {
              at7456_writeSRAM(4, 9, charToAt7456('-'));  // 无效定位显示-
            }
            */
            myAT7456.at7456_OSD_on();
          }
        }
        xSemaphoreGive(spiMutex);
      }
      osd_timer=millis();
  
    }
   
    vTaskDelayUntil(&xLastWakeTime, xTickTime);
  }
}
void task6(void *pvParameters) {
    uint16_t lastAux1 = 0;
    uint16_t lastAux2 = 191;
    uint16_t lastAux3 = 191;
    uint16_t lastAux4 = 0;
    uint8_t frameBuffer[32];  // 足够的缓冲区大小
    int bufferIndex = 0;
    // 初始化通道数据
    channels.throttle =THROTTLE_MIN;
    channels.yaw = 992;
    channels.pitch = 992;
    channels.roll = 992;
    channels.aux1 = 191;
    channels.aux2 = 191;
    channels.aux3 = 191;
    channels.aux4 = 191;
    TickType_t xLastWakeTime = xTaskGetTickCount(); 
    const TickType_t xTickTime = pdMS_TO_TICKS(1);
    int flag_receive=0;
    float arm_angle=25.0;//单位°
    // Serial.print("Task6 running on core: ");
    // Serial.println(xPortGetCoreID()); // 输出0或1
    while (1) {
        // rtc_wdt_feed();
        // esp_task_wdt_reset();
        
        // Serial.println(6);
        while (Serial1.available() > 0) {
            uint8_t byte = Serial1.read();
            // Serial.println(byte, HEX); 
            if(bufferIndex == 26 && byte == 0xC8){
              parseFrame(frameBuffer, bufferIndex);
              flag_receive=1;
              break;
              
            }
            if(byte == 0xC8){
              bufferIndex = 0;
            }
          
            // 存储到缓冲区
            frameBuffer[bufferIndex++] = byte;
            
        }
        // if(client.connected()){
        //   client.println(' ');
        //   for(int i=0;i<26;i++){
        //     client.printf("%02X ", frameBuffer[i]);
        //   }
        //   client.println(' ');
        // }
        
        //摇杆值的滑动平均
        if(flag_receive==1){
          flag_receive=0;
          // Serial.printf("%f %f %f\n",target_gx,target_gy,target_gz);
          // 通道数据处理 (保持原有功能)
          base_pwm = map(channels.throttle, 174, 1811, THROTTLE_MIN, THROTTLE_MAX);
          // Serial.println(base_pwm);
          // target_gx =  map(channels.roll, 1811, 174, -gx_max, gx_max);
          // target_gy = map(channels.pitch, 1811, 174, -gy_max, gy_max);
          // target_gz = map(channels.yaw, 1811, 174, -gz_max, gz_max);
          if(mode==1){
          target_gx =getBaserate(channels.pitch,173.0,1811.0,rcRate_x,Rate_x,rcExpo_x, mid)* M_PI / 180.0f;
          target_gy =getBaserate(channels.roll,1811.0,173.0,rcRate_y,Rate_y,rcExpo_y, mid)* M_PI / 180.0f;
          }else if(mode==2){
            target_x=InputMapAngle(channels.pitch,173.0,1811.0)* M_PI / 180.0f;
            target_y=InputMapAngle(channels.roll,1811.0,173.0)* M_PI / 180.0f;
            // Serial.printf("%f,%f\n",target_x,target_y);
          }
          target_gz =getBaserate(channels.yaw,173.0,1811.0,rcRate_z,Rate_z,rcExpo_z, mid)* M_PI / 180.0f;
        }
        // Serial.printf("%d %f %f %f \n",base_pwm,target_gx,target_gy, target_gz);
        // 解锁通道处理
        // if (channels.aux2 > 256 && lastAux2 < 256){
        //   if(flag_calibrate==0){
        //     calibrateGyro();
        //     calibrateAccel();
        //     flag_calibrate=1;
        //   }
        // }
        if (channels.aux2 ==997 && lastAux2 != 997 ) {//按到中位是解锁，向下按是上锁
          previous_gx = 0.0f;previous_gy = 0.0f; previous_gz = 0.0f;
          previous_targetgx = 0.0f; previous_targetgy = 0.0f; previous_targetgz = 0.0f;
          previous_inputx = 992.0f; previous_inputy = 992.0f; previous_inputz = 992.0f;
          filtered_gx_diff = 0.0f, filtered_gy_diff = 0.0f, filtered_gz_diff = 0.0f;
          gx_error_integral=0;gy_error_integral=0; gz_error_integral=0;
          q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 四元数
          exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;   // 积分误差
          x_error_integral=0;
          y_error_integral=0;
          target_x = 0.0f; // 单位rad
          target_y = 0.0f;  // 
          x_error = 0.0f;
          y_error = 0.0f;
          cali_finish=false;
          temp_lock = "unlock";
          /*
          float gyro_con=0.3f;
          float gx_=raw_gx,gy_=raw_gy,gz_=raw_gz;
 
          if(number_gyro==0){
            while(abs(gx_)>=gyro_con || abs(gy_)>=gyro_con || abs(gz_)>=gyro_con || gyro_stable_number<=20){
              gx_fix=abs(gx_)<gyro_con ? 1 : 0;
              gy_fix=abs(gy_)<gyro_con ? 1 : 0;
              gz_fix=abs(gz_)<gyro_con ? 1 : 0;
              
              if(abs(gx_)<gyro_con && abs(gy_)<gyro_con && abs(gz_)<gyro_con){
                gyro_stable_number++;
                Serial.printf("now %f %f %f\n",gx_,gy_,gz_);
                // Serial.printf("%f %f\n",pitch,roll);
              }
              else{
                gyro_stable_number=0;
                while(update_raw==0){

                }
                number_gyro+=1;
                if(gx_fix==0){
                gx_offset=raw_gx;
                }
                if(gy_fix==0){
                gy_offset=raw_gy;
                }
                if(gz_fix==0){
                gz_offset=raw_gz;
                }
                update_raw=0;
                while(update_raw==0){

                }
                gx_=raw_gx-gx_offset;
                gy_=raw_gy-gy_offset;
                gz_=raw_gz-gz_offset;
                update_raw=0;
                // Serial.printf("now %f %f %f\n",gx_offset,gy_offset,gz_offset);
              }
            }
          }
          */  
          /*
          if(mode==2){
            if(abs(pitch) < 1.0f && abs(roll) < 1.0f){
              // temp_lock = "unlock";
              lock ="unlock";
            }
          }else{
            // temp_lock = "unlock";
            lock ="unlock";
          }
          */
          
          
          // previous_gx_2 = 0.0f;previous_gy_2 = 0.0f; previous_gz_2 = 0.0f;
          // previous_targetgx_2 = 0.0f; previous_targetgy_2 = 0.0f; previous_targetgz_2 = 0.0f;
          // gx_error_integral_2=0;gy_error_integral_2=0; gz_error_integral_2=0;
          // }
        }  
        if(temp_lock=="unlock" && base_pwm<0.1*THROTTLE_MAX){
          if(mode==2){
            if(abs(pitch) < arm_angle && abs(roll) < arm_angle){
              // temp_lock = "unlock";
              lock =temp_lock;
            }
          }else{
            // temp_lock = "unlock";
            lock =temp_lock;
          }
        }
        if ((channels.aux2 == 191 && lastAux2 != 191) || flag_acce==0 || flag_gyro==0 ) {//保证连接之后必须先上锁再解锁，每次解锁时必须同时开启积分清零，解锁后可关闭
          temp_lock = "lock";
          lock =temp_lock;
          
          // flag_unlock=1;
        }
        else{
          if(mode==2 ){
            if(abs(pitch)>0.6*ANGLE || abs(roll)>0.6*ANGLE || error>8*norm0){
              temp_lock = "lock";
              lock =temp_lock;
            }
          }
        }
        lastAux2 = channels.aux2;
        //模式切换三档开关,1对应手动模式，2对应自动水平模式
        if (channels.aux3 == 997 && lastAux3 != 997 ) {
          mode=2;
          /*
          Kp_gx=Kp_gx_2;
          Kp_gy=Kp_gy_2;
          Kp_gz=Kp_gz_2;
          Ki_gx=Ki_gx_2;
          Ki_gy=Ki_gy_2;
          Ki_gz=Ki_gz_2;
          Kd_gx=Kd_gx_2;
          Kd_gy=Kd_gy_2;
          Kd_gz=Kd_gz_2;
          FF_gainx=FF_gainx_2;
          FF_gainy=FF_gainy_2;
          FF_gainz=FF_gainz_2;
          */
          // pid_dt=dt_2;
        }
        if (channels.aux3 == 191 && lastAux3 != 191 ) {
          mode=1;
          /*
          Kp_gx=Kp_gx_1;
          Kp_gy=Kp_gy_1;
          Kp_gz=Kp_gz_1;
          Ki_gx=Ki_gx_1;
          Ki_gy=Ki_gy_1;
          Ki_gz=Ki_gz_1;
          Kd_gx=Kd_gx_1;
          Kd_gy=Kd_gy_1;
          Kd_gz=Kd_gz_1;
          FF_gainx=FF_gainx_1;
          FF_gainy=FF_gainy_1;
          FF_gainz=FF_gainz_1;
          */
          // pid_dt=dt_1;
      
        }
        lastAux3 = channels.aux3;
        // Serial.printf("%s , %d\n",lock,mode);
        // 重置积分通道，按下重置
        // if (channels.aux1 > 991 && lastAux1 <= 991) {
        if (channels.aux1 > 991 ) {
            gx_error_integral = 0;
            gy_error_integral = 0;
            gz_error_integral = 0;
            x_error_integral=0;
            y_error_integral=0;
            // gx_error_integral_2=0;gy_error_integral_2=0; gz_error_integral_2=0;
            // Serial.println("Integral resets triggered");
        }
        lastAux1 = channels.aux1;
        // Serial.println(channels.aux4);
        // int beep=0;
        if(vol_lock==0){
          if(channels.aux4>991 ){
            // digitalWrite(beep_pin,HIGH);
            beep_act=1;
            beep_seq[0]=100;
            beep_seq[1]=0;
            beep_seq[2]=100;
            beep_seq[3]=0;
            beep_seq[4]=100;
          }
          // else{
          //   digitalWrite(beep_pin,LOW);
          //   // beep=0;
          // }
        }
        // Serial.println(beep);
        // lastAux4 = channels.aux4;
        // vTaskDelay(pdMS_TO_TICKS(1)); // 100Hz刷新率 (优于50Hz)
        vTaskDelayUntil(&xLastWakeTime, xTickTime);
    } 
}

void task7(void *pvParameters){
  TickType_t xLastWakeTime = xTaskGetTickCount(); 
  const TickType_t xTickTime = pdMS_TO_TICKS(700);
  if(CDState()==LOW){
    writeFile(SD, "/gyro.txt", "gyro_x  gyro_y  gyro_z\n");
    writeFile(SD, "/he.txt", "Hello, this is a test log entry!\n");
    writeFile(SD, "/gps.txt", "Time  Lat纬度  Lon经度  Sat卫星  Height高度(m)\n");
  }
  while(1){
    // Serial.println(7);
    // if(xSemaphoreTake(spiMutex,portMAX_DELAY)==pdTRUE){
    //   hspi.endTransaction();
      if(CDState()==LOW){
        if(gps_record_index>=GPS_CYCLE){
          char latStr[8],lonStr[8],heightStr[8],satStr[2]; 
      
          for(int i_gps=0;i_gps<GPS_CYCLE;i_gps++){
            snprintf(lonStr, sizeof(lonStr), "%.2f",gps_record[i_gps].log);
            snprintf(latStr, sizeof(latStr), "%.2f",gps_record[i_gps].lat);
            snprintf(satStr, sizeof(satStr), "%d", gps_record[i_gps].numSV);
            snprintf(heightStr, sizeof(heightStr), "%.2f", gps_record[i_gps].height_m);
            if(xSemaphoreTake(spiMutex,portMAX_DELAY)==pdTRUE){
              hspi.endTransaction();
              appendFile(SD, "/gps.txt", gps_record[i_gps].timeStr);
              appendFile(SD, "/gps.txt", " ");
              appendFile(SD, "/gps.txt", latStr);
              appendFile(SD, "/gps.txt", " ");
              appendFile(SD, "/gps.txt", lonStr);
              appendFile(SD, "/gps.txt", " ");
              appendFile(SD, "/gps.txt", satStr);
              appendFile(SD, "/gps.txt", " ");
              appendFile(SD, "/gps.txt", heightStr);
              appendFile(SD, "/gps.txt", " \n");
              hspi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
            }
            
          }
          Serial.printf("gps data appended!\n");
          gps_record_index=0;
          memset(&gps_record, 0, GPS_CYCLE);
        }
        
        if(gyro_record_index>=GYRO_CYCLE){
          char gxStr[10],gyStr[10],gzStr[10];
          // String gxString,gyString,gzString;
          for(int i_gyro=0;i_gyro<GYRO_CYCLE;i_gyro++){
            snprintf(gxStr,10,"%.2f°/s",gx_record[i_gyro]);
            snprintf(gyStr,10,"%.2f°/s",gy_record[i_gyro]);
            snprintf(gzStr,10,"%.2f°/s",gz_record[i_gyro]);
            if(xSemaphoreTake(spiMutex,portMAX_DELAY)==pdTRUE){
              hspi.endTransaction();
              appendFile(SD, "/gyro.txt", gxStr);
              appendFile(SD, "/gyro.txt", " ");
              appendFile(SD, "/gyro.txt", gyStr);
              appendFile(SD, "/gyro.txt", " ");
              appendFile(SD, "/gyro.txt", gzStr);
              appendFile(SD, "/gyro.txt", " \n");
              hspi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
            }
          }
          Serial.printf("gyro data appended!\n");
          gyro_record_index=0;
          memset(gx_record, 0, GYRO_CYCLE);
          memset(gy_record, 0, GYRO_CYCLE);
          memset(gz_record, 0, GYRO_CYCLE);
        }
        // 以下为文件操作示例，可根据需要启用
        // listDir(SD, "/", 0);//不使用该语句会导致看门狗重启
        // writeFile(SD, "/he.txt", "Hello, this is a test log entry!\n");
        // appendFile(SD, "/he.txt", "This is an appended line.\n");
        // Serial.printf("test message appended!\n");
        // readFile(SD, "/he.txt");
      }
      // hspi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
      vTaskDelayUntil(&xLastWakeTime, xTickTime);
    //   xSemaphoreGive(spiMutex);
    // }
    
  }
}
void task8(void *pvParameters){
  TickType_t xLastWakeTime = xTaskGetTickCount(); 
  const TickType_t xTickTime = pdMS_TO_TICKS(1);
  int i_sub=0;
  int number=0;
  while(1){
    if(beep_act==1){
      
      if(number==0 && beep_seq[i_sub]>0){
        if(i_sub%2==0){
          digitalWrite(beep_pin,HIGH);
        }
        else{
          digitalWrite(beep_pin,LOW);
        }
      }
      
      if(number==beep_seq[i_sub]){
        number=0;
        i_sub++;
        if(i_sub==5){
          beep_act=0;
          i_sub=0;
        }
      }
      else{
        number++;
      }
    }
    else{
      digitalWrite(beep_pin,LOW);
    }
    vTaskDelayUntil(&xLastWakeTime, xTickTime);
  }
}
void setup() {
    pinMode(beep_pin, OUTPUT);
    digitalWrite(beep_pin,LOW);
   
    Serial.begin(115200);
  

    motor1.begin(MOTOR_1_PIN, DSHOT::DSHOT600);  // 引脚12, 双向DSHOT300
    motor2.begin(MOTOR_2_PIN, DSHOT::DSHOT600);  // 引脚14
    motor3.begin(MOTOR_3_PIN, DSHOT::DSHOT600);  // 引脚27
    motor4.begin(MOTOR_4_PIN, DSHOT::DSHOT600);  // 引脚26
    
    DSHOT::arm();// 执行ESC初始化序列（关键步骤！）
    
    // 初始化串口
    GpsSerial.begin(115200, SERIAL_8N1, 16, 17); // 使用GPIO16(RX), GPIO17(TX)作为Serial2
    // DebugSerial.begin(115200); // ESP32的Serial通常使用较高波特率

    pinMode(VOLTAGE_PIN, INPUT); // 设置为输入模式
    // WiFi.softAP(ssid, password);
    // IPAddress IP=WiFi.softAPIP(); 
    // Serial.println(IP);
    // server.begin();
    // Wire.begin(SDA_PIN, SCL_PIN);
    //创建互斥锁
    spiMutex=xSemaphoreCreateMutex();
    spiMutex_icm=xSemaphoreCreateMutex();
    setupHSPI();
    if(CDState()==LOW){
      mySD.InitSD();
    }
    
    
    setupSPI();
    initICM42688();
    int try_at7456=0;
    while(myAT7456.at7456_init() != 0 && try_at7456<10){
      delay(1);
      try_at7456++;
    }
    // 配置重映射的UART0，接收机的
    Serial1.begin(420000, SERIAL_8N1, RX_PIN, TX_PIN);
    // SD_INIT();
    // FastLED.addLeds<WS2812B, FlowingLightPin, GRB>(leds, NUM_LEDS);
    // 创建处理任务
    // xTaskCreate(task1, "ProcessWIFI", 4096, NULL, 2, NULL);
    // xTaskCreate(task2, "ProcessControl", 4096, NULL, 5, NULL);
    // xTaskCreate(task3, "ProcessSensor", 2048, NULL, 3, NULL);
    // // xTaskCreate(task4, "LightControl", 2048, NULL, 3, NULL);//任务可以创建成功
    // xTaskCreate(task5, "OSDdisplay", 2048, NULL, 3, NULL);
    // xTaskCreate(task6, "ELRS Receiver", 2048, NULL, 4, NULL);
    xTaskCreatePinnedToCore(task2, "ProcessControl", 4096, NULL,1, NULL, 1);  // 核心1
    // 任务3（传感器处理）→ 核心0
    xTaskCreatePinnedToCore(task3, "ProcessSensor", 2048, NULL, 1, NULL, 0);  // 核心0
    // 任务5（OSD显示）→ 核心0
    xTaskCreatePinnedToCore(task5, "OSDdisplay", 2048, NULL, 1, NULL, 0);  // 核心0
    // 任务6（接收机数据处理）→ 核心0
    xTaskCreatePinnedToCore(task6, "ELRS Receiver", 2048, NULL, 1, NULL, 0);  // 核心0
    xTaskCreatePinnedToCore(task7, "SDOperation", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(task8, "Beeping", 2048, NULL, 1, NULL, 0);
    rcExpo_x=getExpo(Rate_x,mid);
    rcExpo_y=getExpo(Rate_y,mid);
    rcExpo_z=getExpo(Rate_z,mid);
    EEPROM.begin(100);
    if(version==DATA_VERSION){
      flag_acce=1;
      flag_gyro=1;
      EEPROM.get(0,ax0);
      EEPROM.get(4,ay0);
      EEPROM.get(8,az0);
      EEPROM.get(12,gx0);
      EEPROM.get(16,gy0);
      EEPROM.get(20,gz0);
      norm0=sqrt(ax0*ax0+ ay0 * ay0 + az0 * az0);
      norm=norm0;
      ACC_ERROR_THRESHOLD=0.02*norm0;
    }
    
    /*
    // 初始化SD卡，使用4位SDIO模式
    // 参数"false"表示不使用1线模式，即启用4线模式
    if (!SD_MMC.begin()) {
        Serial.println("SD卡挂载失败！");
        SDflag[0]=0;
    }
    else{
      Serial.println("SD卡挂载成功！");
      SDflag[0]=1;
      
      // 打印卡信息
      uint8_t cardType = SD_MMC.cardType();
      if (cardType == CARD_NONE) {
          Serial.println("未识别到SD卡");
          SDflag[1]=0;
          return;
      }
      else{
      // Serial.print("SD卡类型: ");
      SDflag[1]=1;
      }
      if(cardType == CARD_MMC) SDtype="MMC";//Serial.println("MMC");
      else if(cardType == CARD_SD) SDtype="SDSC";//Serial.println("SDSC");
      else if(cardType == CARD_SDHC) SDtype="SDHC";//Serial.println("SDHC");
      else SDtype="none";//Serial.println("未知");

      Serial.printf("SD卡总大小: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
      Serial.printf("已用空间: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));

      // 创建并打开数据文件
      dataFile = SD_MMC.open("/gyro_data.txt", FILE_APPEND); // 使用追加模式
      if(!dataFile) {
          Serial.println("创建数据文件失败！");
          return;
      }
      
      cleanUpSD();
    }
    */
}


void loop() {
  
  
  
  vTaskDelay(portMAX_DELAY); 
  
}
void setupHSPI() {
  pinMode(AT7456_CS, OUTPUT);
  digitalWrite(AT7456_CS, HIGH); // CS 高电平不选中
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH); // CS 高电平不选中
  hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI); // 片选引脚通常由用户控制，此处可不传入
  // 为HSPI设备设置通信参数（10MHz, 模式0, MSB优先）
  // hspi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
}
void adjustMotors(float gx, float gy,float gz) {
 
  gx = gx * M_PI / 180.0f;
  gy = gy * M_PI / 180.0f;
  gz = gz * M_PI / 180.0f;
 
  
  // 计算误差
  gx_error = target_gx - gx;
  gy_error = target_gy - gy;
  gz_error = target_gz - gz;
 
  gx_error_integral += gx_error;
  gy_error_integral += gy_error;
  gz_error_integral += gz_error;
  float gx_diff = gx - previous_gx;
  float gy_diff = gy - previous_gy;
  float gz_diff = gz - previous_gz;
 
  gx_error_derivative = d_weight*(target_gx-previous_targetgx)-gx_diff;
  gy_error_derivative = d_weight*(target_gy-previous_targetgy)-gy_diff;
  gz_error_derivative = d_weight*(target_gz-previous_targetgz)-gz_diff;


 
  float FF_termx=FF_gainx*(previous_inputx-channels.roll)* M_PI / 180.0;
  float FF_termy=FF_gainy*(previous_inputy-channels.pitch)* M_PI / 180.0;
  float FF_termz=FF_gainz*(previous_inputz-channels.yaw)* M_PI / 180.0;
  FF_termx=constrain(FF_termx,-40,40);
  FF_termy=constrain(FF_termy,-46,46);
  FF_termz=constrain(FF_termz,-40,40);
  // float gx_output = Kp_gx * gx_error + Ki_gx * gx_error_integral/9000.0+ Kd_gx * gx_error_derivative +FF_termx;
  // float gy_output = Kp_gy * gy_error + Ki_gy * gy_error_integral/9000.0 + Kd_gy * gy_error_derivative +FF_termy;
  // float gz_output = Kp_gz * gz_error + Ki_gz * gz_error_integral/8000.0 +FF_termz;
  
  float gx_output = Kp_gx * gx_error + Ki_gx * gx_error_integral*T+ Kd_gx * gx_error_derivative;
  float gy_output = Kp_gy * gy_error + Ki_gy * gy_error_integral*T+ Kd_gy * gy_error_derivative;
  float gz_output = Kp_gz * gz_error + Ki_gz * gz_error_integral*T;

  // Serial.printf(" %f , %f,%f\n",FF_termx, FF_termy,FF_termz);
  // 更新前一次误差
  previous_gx = gx;
  previous_gy = gy;
  previous_gz = gz;
  previous_targetgx = target_gx;
  previous_targetgy = target_gy;
  previous_targetgz = target_gz;
  previous_inputx=channels.roll;
  previous_inputy=channels.pitch;
  previous_inputz=channels.yaw;
  // 调整电机转速
  //gx_error若小于0，gx要往小调，gx增大一侧的两个电机转速往上调，应该减去gx_error
  //gy_error若小于0，gy要往小调，gy增大一侧的两个电机转速往上调，应该减去gy_error
  //gz_error若小于0，gz正向过大，要往小调，与gz增大同旋向的两个桨叶对应的电机1、4转速往上调，应该减gz_error，因为反扭距会克服gz的过大
  if(base_pwm<1350){
    // throttles[0] = base_pwm + gx_output-gy_output+gz_output; // 电机1
    // throttles[1]  = base_pwm + gx_output+gy_output-gz_output; // 电机2
    // throttles[2]  = base_pwm - gx_output -gy_output-gz_output; // 电机3
    // throttles[3] = base_pwm -gx_output +gy_output +gz_output; // 电机4
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
  int minThrottle=findMin(throttles, 4);
  if(minThrottle<0){
    throttles[0]-=minThrottle;
    throttles[1]-=minThrottle;
    throttles[2]-=minThrottle;
    throttles[3]-=minThrottle;
  }
  // 限制PWM值在有效范围内
  throttles[0] = constrain(throttles[0], THROTTLE_MIN , THROTTLE_MAX );
  throttles[1] = constrain(throttles[1], THROTTLE_MIN , THROTTLE_MAX);
  throttles[2] = constrain(throttles[2], THROTTLE_MIN , THROTTLE_MAX);
  throttles[3] = constrain(throttles[3], THROTTLE_MIN ,THROTTLE_MAX);
}
int findMin(int arr[], int size) {
  if (size <= 0) return 0; // 处理异常情况
  int minVal = arr[0];
  for (int i = 1; i < size; i++) {
    if (arr[i] < minVal) minVal = arr[i];
  }
  return minVal;
}

/*
void calibrateGyro() {
    Serial.println("Starting gyroscope calibration...");
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    // float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    float gxmax=0.0f,gymax=0.0f,gzmax=0.0f;
    float gxmin=0.0f,gymin=0.0f,gzmin=0.0f;
    // 收集指定数量的样本
    for (int i = 0; i < calibrationSamples; i++) {
        digitalWrite(AT7456_CS, HIGH); 
        digitalWrite(ICM_CS_PIN, LOW);
        SPI.transfer(0x1F | 0x80); // 最高位=1（读操作）
       
        for (int j = 0; j < 6; j++) SPI.transfer(0x00);
        
        // 读取陀螺仪原始数据
        int16_t rawGX = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
        int16_t rawGY = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
        int16_t rawGZ = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
        digitalWrite(ICM_CS_PIN, HIGH);
        // if(lastRawgx!=0){
        //   rawGX=(abs(rawGX/lastRawgx-1)>sifter) ? 0 :rawGX;
        // }
        // if(lastRawgy!=0){
        //   rawGY=(abs(rawGY/lastRawgy-1)>sifter) ? 0 :rawGY;
        // }
        // if(lastRawgz!=0){
        //   rawGZ=(abs(rawGZ/lastRawgz-1)>sifter) ? 0 :rawGZ;
        // }
        // 累加数据
     
        gx_sum += rawGX;
        gy_sum += rawGY;
        gz_sum += rawGZ;
        // gxmax=rawGX>gxmax ? rawGX :gxmax;
        // gymax=rawGY>gymax ? rawGY :gymax;
        // gzmax=rawGZ>gzmax ? rawGZ :gzmax;
        // gxmin=rawGX<gxmin ? rawGX :gxmin;
        // gymin=rawGY<gymin ? rawGY :gymin;
        // gzmin=rawGZ<gzmin ? rawGZ :gzmin;
        
        //delay(1); // 每个样本间隔10ms
    }
    
    // 计算平均偏移值 (±500°/s量程转换)
    const float gyroScale = 0.015267176f;
    // gxmax=gxmax*gyroScale;
    // gymax=gymax*gyroScale;
    // gzmax=gzmax*gyroScale;
    // gxmin=gxmin*gyroScale;
    // gymin=gymin*gyroScale;
    // gzmin=gzmin*gyroScale;
    gx_offset = (gx_sum / calibrationSamples) * gyroScale;
    gy_offset = (gy_sum / calibrationSamples) * gyroScale;
    gz_offset = (gz_sum / calibrationSamples) * gyroScale;
    
    // rx_offset=abs(gxmin-gxmax);
    // ry_offset=abs(gymin-gymax);
    // rz_offset=abs(gzmin-gzmax);
    
}
*/
/*
void calibrateAccel() {
    float ax_sum = 0, ay_sum = 0, az_sum = 0;
    for (int i = 0; i < 100; i++) {
        digitalWrite(AT7456_CS, HIGH); 
        digitalWrite(ICM_CS_PIN, LOW);
        SPI.transfer(0x1F | 0x80); // 最高位=1（读操作）
        // 读取加速度计数据
        int16_t rawAX = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
        int16_t rawAY = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
        int16_t rawAZ = SPI.transfer(0x00) << 8 | SPI.transfer(0x00);
        // for (int j = 0; j < 6; j++) SPI.transfer(0x00);
        digitalWrite(ICM_CS_PIN, HIGH);
        // ±4g量程转换 (8192 LSB/g)
        const float accelScale = 0.00119695f; // 9.80665 / 8192
        ax_sum += rawAX * accelScale;
        ay_sum += rawAY * accelScale;
        az_sum += rawAZ * accelScale;
        
    }
    ax_offset = ax_sum / 100;   // 水平静止时理论值应为 0
    ay_offset = ay_sum / 100;   // 水平静止时理论值应为 0
    az_offset = (az_sum / 100) - 9.80665; // 理论值应为 1g (9.80665 m/s²)
    // Serial.printf("%f %f %f\n",ax_offset,ay_offset,az_offset);
}
*/
float getExpo(float Rate,float mid){
  float Expo;
  Expo=(1-Rate)/(1-pow(mid,2));
  return Expo;
}
float getBaserate(uint16_t rawSpeed,float stick_min,float stick_max,float rcRate,float Rate,float rcExpo,float mid){
  int sign;
  float stickInput;
  float response;
  float baseRate;
  // stickInput=map(rawSpeed, 1811, 174, -1, 1);min=173,max=1811
  stickInput=-1+2*((rawSpeed-stick_max)/(stick_min-stick_max));
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
  baseRate=RATE*rcRate_x*response;
  return baseRate;
}
// float max(float a,float b){
//   return (a<b) ? b : a;
// }
//gps有关函数定义
// 将度分格式转换为十进制度格式
double convertToDecimalDegrees(const char* degreeMinute, char direction)
{
  // 示例输入: "2429.53531" 和 'N'
  // 输出: 24.49225517
  
  // 将字符串转换为double
  double dm = atof(degreeMinute);
  
  // 获取度数部分
  int degrees = (int)(dm / 100);
  
  // 获取分钟部分
  double minutes = dm - (degrees * 100);
  
  // 转换为十进制度
  double decimalDegrees = degrees + (minutes / 60.0);
  
  // 处理方向（南纬和西经需要为负值）
  if (direction == 'S' || direction == 'W') {
    decimalDegrees = -decimalDegrees;
  }
  
  return decimalDegrees;
}
void parseNAV_PVT(uint8_t* buffer){
  Gps_Data.year = (buffer[5] << 8) | buffer[4];
  Gps_Data.month = buffer[6];
  Gps_Data.day = buffer[7];
  Gps_Data.hour = buffer[8];
  Gps_Data.minute = buffer[9];
  Gps_Data.second = buffer[10];
  Gps_Data.numSV=buffer[23];
  snprintf(
    Gps_Data.timeStr, sizeof(Gps_Data.timeStr), 
    "%04u-%02u-%02u %02u:%02u:%02u",  // 格式模板
    Gps_Data.year, Gps_Data.month, Gps_Data.day, Gps_Data.hour, Gps_Data.minute, Gps_Data.second  // 填充变量
  );
  uint32_t raw_log = (uint32_t)buffer[27] << 24 |  (uint32_t)buffer[26] << 16 |(uint32_t)buffer[25] << 8  | (uint32_t)buffer[24];
  Gps_Data.log=((buffer[27] & 0x80) ? (raw_log | 0xFF000000) : raw_log)/10000000.0f;
  uint32_t raw_lat = (uint32_t)buffer[31] << 24 |  (uint32_t)buffer[30] << 16 |(uint32_t)buffer[29] << 8  | (uint32_t)buffer[28];
  Gps_Data.lat=((buffer[31] & 0x80) ? (raw_lat | 0xFF000000) : raw_lat)/10000000.0f;
  uint32_t raw_height = (uint32_t)buffer[35] << 24 |  (uint32_t)buffer[34] << 16 |(uint32_t)buffer[33] << 8  | (uint32_t)buffer[32];
  Gps_Data.height_m=((buffer[35] & 0x80) ? (raw_height | 0xFF000000) : raw_height)/ 1000.0f;
  uint32_t raw_gspeed = (uint32_t)buffer[63] << 24 |  (uint32_t)buffer[62] << 16 |(uint32_t)buffer[61] << 8  | (uint32_t)buffer[60];
  Gps_Data.gspeed_kmh=((buffer[63] & 0x80) ? (raw_gspeed | 0xFF000000) : raw_gspeed)* 0.0036f;
  uint32_t raw_heading = (uint32_t)buffer[67] << 24 |  (uint32_t)buffer[66] << 16 |(uint32_t)buffer[65] << 8  | (uint32_t)buffer[64];
  Gps_Data.heading_deg=((buffer[67] & 0x80) ? (raw_heading | 0xFF000000) : raw_heading)/ 100000.0f;
  uint32_t raw_hAcc=(uint32_t)buffer[43] << 24 |  (uint32_t)buffer[42] << 16 |(uint32_t)buffer[41] << 8  | (uint32_t)buffer[40];
  Gps_Data.hAcc=((buffer[43] & 0x80) ? (raw_hAcc | 0xFF000000) : raw_hAcc)/1000.0f;
  uint32_t raw_cAcc=(uint32_t)buffer[47] << 24 |  (uint32_t)buffer[46] << 16 |(uint32_t)buffer[45] << 8  | (uint32_t)buffer[44];
  Gps_Data.cAcc=((buffer[47] & 0x80) ? (raw_cAcc | 0xFF000000) : raw_cAcc)/1000.0f;
  uint32_t raw_speedAcc=(uint32_t)buffer[71] << 24 |  (uint32_t)buffer[70] << 16 |(uint32_t)buffer[69] << 8  | (uint32_t)buffer[68];
  Gps_Data.speedAcc=((buffer[71] & 0x80) ? (raw_speedAcc | 0xFF000000) : raw_speedAcc)* 0.0036f;
  uint32_t raw_headAcc=(uint32_t)buffer[75] << 24 |  (uint32_t)buffer[74] << 16 |(uint32_t)buffer[73] << 8  | (uint32_t)buffer[72];
  Gps_Data.headAcc=((buffer[75] & 0x80) ? (raw_headAcc | 0xFF000000) : raw_headAcc)/ 100000.0f;
}
double calculateDistance(double lat1,double log1,double lat2,double log2){
  double dLat =(lat2 - lat1)* M_PI / 180.0;//从1到2的纬度角差值
  double dLon =(log2 - log1)* M_PI / 180.0;//从1到2的经度角差值
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1* M_PI / 180.0) * cos(lat2* M_PI / 180.0) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return EARTH_RADIUS * c; // 返回单位：米
}
double calculateBearing(double lat1,double log1,double lat2,double log2){//计算从1到2的向量从正北顺时针转过的角度
  double radLat1 = lat1* M_PI / 180.0;
  double radLat2 = lat2* M_PI / 180.0;
  double dLon = (log2 - log1)* M_PI / 180.0;
  double y = sin(dLon) * cos(radLat2);//y的正负由dlon决定，log2>log1，2在1的右侧，y为正
  double x = cos(radLat1) * sin(radLat2) - sin(radLat1) * cos(radLat2) * cos(dLon);
  double bearing = atan2(y, x);
  bearing = bearing* 180.0 / M_PI;
  // 将结果标准化为 0-360 度
  if (bearing < 0) {
      bearing += 360.0;
  }
  return bearing;
}
/*
void errorLog(int num)
{
  DebugSerial.print("ERROR ");
  DebugSerial.println(num);
  while (1)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(300);
  }
}

void printGpsBuffer() {
  if (Save_Data.isParseData) {
    Save_Data.isParseData = false;

    DebugSerial.print("UTC Time: ");
    DebugSerial.println(Save_Data.UTCTime);

    if (Save_Data.isUsefull) {
      Save_Data.isUsefull = false;
      
      // 转换并打印纬度
      double lat = convertToDecimalDegrees(Save_Data.latitude, Save_Data.N_S[0]);
      DebugSerial.print("Latitude(Decimal Degrees): ");
      DebugSerial.println(lat, 8);
      
      // 转换并打印经度
      double lon = convertToDecimalDegrees(Save_Data.longitude, Save_Data.E_W[0]);
      DebugSerial.print("Longitude(Decimal Degrees): ");
      DebugSerial.println(lon, 8);

      // 输出卫星数量
      DebugSerial.print("Satellites in use: ");
      DebugSerial.println(Save_Data.satelliteCount);
      
      // 打印海拔高度（如果已获取）
      if(strlen(Save_Data.altitude) > 0) {
        DebugSerial.print("Altitude: ");
        DebugSerial.print(Save_Data.altitude);
        DebugSerial.print(" ");
        DebugSerial.println(Save_Data.altitudeUnit);
      }
      
      // 同时输出原始数据供参考
      DebugSerial.print("Original Latitude: ");
      DebugSerial.print(Save_Data.latitude);
      DebugSerial.print(" ");
      DebugSerial.println(Save_Data.N_S);
      
      DebugSerial.print("Original Longitude: ");
      DebugSerial.print(Save_Data.longitude);
      DebugSerial.print(" ");
      DebugSerial.println(Save_Data.E_W);
    }
    else {
      DebugSerial.println("GPS DATA is not useful!");
    }
  }
}

void parseGpsBuffer()
{
  char *subString;
  char *subStringNext;
  if (Save_Data.isGetData)
  {
    Save_Data.isGetData = false;
    DebugSerial.println("**************");
    DebugSerial.println(Save_Data.GPS_Buffer);
    // 检查是RMC还是GGA语句
    bool isRMC = (strstr(Save_Data.GPS_Buffer, "$GPRMC") != NULL || 
                 strstr(Save_Data.GPS_Buffer, "$GNRMC") != NULL);
    bool isGGA = (strstr(Save_Data.GPS_Buffer, "$GPGGA") != NULL || 
                strstr(Save_Data.GPS_Buffer, "$GNGGA") != NULL);
    if(isRMC){
      subString = Save_Data.GPS_Buffer;
      for (int i = 0; i <= 6; i++)
      {
        if (i == 0)
        {
          if ((subString = strstr(subString, ",")) == NULL)//substring跳过RMC开头直接找第一个逗号
            errorLog(1); // 解析错误
        }
        else
        {
          subString++;
          if(*subString==','){//此时该字段为空
            subString--;
          }
          if ((subStringNext = strstr(subString, ",")) != NULL)
          {
            char usefullBuffer[2];
            switch (i)
            {
            case 1:{
              memset(Save_Data.UTCTime, '\0', sizeof(Save_Data.UTCTime));
              int len = subStringNext - subString;
              // len = min(len, (int)sizeof(Save_Data.altitude) - 1);
              strncpy(Save_Data.UTCTime, subString, len);
              Save_Data.UTCTime[len] = '\0'; // 冗余终止（安全加固）
              //memcpy(Save_Data.UTCTime, subString, subStringNext - subString);
              break; // 获取UTC时间
            }
            case 2:{
              memset(usefullBuffer, '\0', sizeof(usefullBuffer));
              int len = subStringNext - subString;
              // len = min(len, (int)sizeof(Save_Data.altitude) - 1);
              strncpy(usefullBuffer, subString, len);
              usefullBuffer[len] = '\0'; // 冗余终止（安全加固）
              //memcpy(usefullBuffer, subString, subStringNext - subString);
              break; // 获取有效性标志
            }
            case 3:{
              memset(Save_Data.latitude, '\0', sizeof(Save_Data.latitude));
              int len = subStringNext - subString;
              // len = min(len, (int)sizeof(Save_Data.altitude) - 1);
              strncpy(Save_Data.latitude, subString, len);
              Save_Data.latitude[len] = '\0'; // 冗余终止（安全加固）
              //memcpy(Save_Data.latitude, subString, subStringNext - subString);
              break; // 获取纬度信息
            }
            case 4:{
              memset(Save_Data.N_S, '\0', sizeof(Save_Data.N_S));
              int len = subStringNext - subString;
              len = min(len, (int)sizeof(Save_Data.N_S) - 1);
              strncpy(Save_Data.N_S, subString, len);
              Save_Data.N_S[len] = '\0'; // 冗余终止（安全加固）
              //memcpy(Save_Data.N_S, subString, subStringNext - subString);
              break; // 获取N/S
            }
            case 5:{
              memset(Save_Data.longitude, '\0', sizeof(Save_Data.longitude));
              int len = subStringNext - subString;
              // len = min(len, (int)sizeof(Save_Data.altitude) - 1);
              strncpy(Save_Data.longitude, subString, len);
              Save_Data.longitude[len] = '\0'; // 冗余终止（安全加固）
              //memcpy(Save_Data.longitude, subString, subStringNext - subString);
              break; // 获取经度信息
            }
            case 6:{
              memset(Save_Data.E_W, '\0', sizeof(Save_Data.E_W));
              int len = subStringNext - subString;
              len = min(len, (int)sizeof(Save_Data.E_W) - 1);
              strncpy(Save_Data.E_W, subString, len);
              Save_Data.E_W[len] = '\0'; // 冗余终止（安全加固）
              //memcpy(Save_Data.E_W, subString, subStringNext - subString);
              break; // 获取E/W
            }
            default:
              break;
            }

            subString = subStringNext;
            Save_Data.isParseData = true;
            if (usefullBuffer[0] == 'A')
              Save_Data.isUsefull = true;
            else if (usefullBuffer[0] == 'V')
              Save_Data.isUsefull = false;
          }
          else
          {
            errorLog(2); // 解析错误
          }
        }
      }
    }
    else if(isGGA) {
      // 解析GGA语句获取海拔高度
      subString = Save_Data.GPS_Buffer;
      for (int i = 0; i <= 10; i++) {
        if(i == 0) {
          if ((subString = strstr(subString, ",")) == NULL) errorLog(1);
        } else {
          subString++;//如果高度字段为空，那么在上一个数据字段之后，substring加一跳转到高度字段的后逗号即高度单位的前逗号，substringnext搜索到高度单位的后逗号，高度被赋予了高度单位
          if(*subString==','){//此时该字段为空
            subString--;
          }
          if ((subStringNext = strstr(subString, ",")) != NULL) {//只要能找到下一个逗号的位置就进行if
            switch(i) {
              case 7:{  // 卫星数量
                // 提取两个逗号之间的内容并转换为整数
                char temp[4] = {0};
                int len = min(3, (int)(subStringNext - subString));
                strncpy(temp, subString, len);
                Save_Data.satelliteCount = atoi(temp);
                break;
              }
              case 9:{  // 海拔高度
                //memcpy(Save_Data.altitude, subString, subStringNext - subString);
                memset(Save_Data.altitude, '\0', sizeof(Save_Data.altitude));
                int len = subStringNext - subString;
                // len = min(len, (int)sizeof(Save_Data.altitude) - 1);
                strncpy(Save_Data.altitude, subString, len);
                Save_Data.altitude[len] = '\0'; // 冗余终止（安全加固）
                break;
              }
              case 10:{ // 高度单位(通常是'M')
                //memcpy(Save_Data.altitudeUnit, subString, subStringNext - subString);
                memset(Save_Data.altitudeUnit, '\0', sizeof(Save_Data.altitudeUnit));
                int len = subStringNext - subString;
                len = min(len, (int)sizeof(Save_Data.altitudeUnit) - 1);
                strncpy(Save_Data.altitudeUnit, subString, len);
                Save_Data.altitudeUnit[len] = '\0'; // 冗余终止（安全加固）
                break;
              }
              default:
                break;
            }
            subString = subStringNext;
          }
        }
      }
    }
    Save_Data.isParseData = true;
  }
}

void gpsRead() {
  while (GpsSerial.available()) {
    gpsRxBuffer[ii++] = GpsSerial.read();
    if (ii == gpsRxBufferLength) clrGpsRxBuffer();
  }
  char *GPS_BufferHead;
  char *GPS_BufferTail;
  
  // 检查RMC或GGA语句
  if ((GPS_BufferHead = strstr(gpsRxBuffer, "$GPRMC,")) != NULL || 
      (GPS_BufferHead = strstr(gpsRxBuffer, "$GNRMC,")) != NULL ||
      (GPS_BufferHead = strstr(gpsRxBuffer, "$GPGGA,")) != NULL ||
      (GPS_BufferHead = strstr(gpsRxBuffer, "$GNGGA,")) != NULL) {
    if (((GPS_BufferTail = strstr(GPS_BufferHead, "\r\n")) != NULL) && (GPS_BufferTail > GPS_BufferHead)) {
      memcpy(Save_Data.GPS_Buffer, GPS_BufferHead, GPS_BufferTail - GPS_BufferHead);
      Save_Data.isGetData = true;
      clrGpsRxBuffer();
    }
  }
}

void clrGpsRxBuffer()
{
  memset(gpsRxBuffer, 0, gpsRxBufferLength); // 清空缓冲区
  ii = 0;
}
*/

float readPreciseVoltage() {
    // 读取原始ADC值(12位)
    int rawValue = analogRead(VOLTAGE_PIN);
    // 计算实际电压(考虑分压比)
    // float measuredVoltage = rawValue * (ADC_REF_VOLTAGE / ADC_RESOLUTION) *11.12;
    float measuredVoltage = rawValue * (ADC_REF_VOLTAGE / ADC_RESOLUTION) *11.737;//25v=rawvol*2.13v
    // 根据模块规格进行精度修正
    // 模块最小检测电压: 0.00489V*5=0.02445V
    // 当测量值小于最小分辨率时返回0
    if(measuredVoltage < 0.02445f) {
      return 0.0f;
    }
    
    return measuredVoltage;
  }

  //流水灯动态效果函数
// void rainbowWave() {
//   static uint8_t hue = 0;
//   for(int i=0; i<NUM_LEDS; i++) {
//     leds[i] = CHSV((i * 5 + hue) % 256, 255, 255);
//   }
//   hue++;
//   FastLED.show();
//   delay(20);
// }
// void runningLight(CRGB color, int speed) {
//   static int position = 0;
  
//   // 清除所有LED
//   fill_solid(leds, NUM_LEDS, CRGB::Black);
  
//   // 设置当前位置LED
//   leds[position] = color;
  
//   position = (position + 1) % NUM_LEDS;
//   FastLED.show();
//   delay(speed);
// }
// void breathingEffect(CRGB color) {
//   if(increasing) {
//     brightness += 5;
//     if(brightness >= 255) increasing = false;
//   } else {
//     brightness -= 5;
//     if(brightness <= 0) increasing = true;
//   }
//   fill_solid(leds, NUM_LEDS, color);
//   FastLED.setBrightness(brightness);
//   FastLED.show();
//   //vTaskDelay(10 / portTICK_PERIOD_MS);
// }
// void randomTwinkle() {
//   // 随机选择几个LED
//   for(int i=0; i<NUM_LEDS/10; i++) {
//     int pos = random(NUM_LEDS);
//     leds[pos] = CRGB(random(256), random(256), random(256));
//   }
  
//   // 渐暗效果
//   fadeToBlackBy(leds, NUM_LEDS, 25);
//   FastLED.show();
//   delay(30);
// }

//接收机部分代码
// void handlePairingSequence() {
//   switch (currentState) {
//     case POWER_CYCLE_1_OFF:{
      
//       digitalWrite(POWER_PIN, HIGH);  // 第一次通电
//       currentState = POWER_CYCLE_1_ON;
      
//       Serial.println("Power cycle 1: ON");
//       delay(POWER_CYCLE_DELAY);
      
//       break;
//     }
//     case POWER_CYCLE_1_ON:{
     
//       digitalWrite(POWER_PIN, LOW);  // 第一次断电
//       currentState = POWER_CYCLE_2_OFF;
   
//       Serial.println("Power cycle 1: OFF");
//       delay(POWER_CYCLE_DELAY);
//       break;
//     }
//     case POWER_CYCLE_2_OFF:{
      
//       digitalWrite(POWER_PIN, HIGH);  // 第二次通电
//       currentState = POWER_CYCLE_2_ON;
    
//       Serial.println("Power cycle 2: ON");
//       delay(POWER_CYCLE_DELAY);
//       break;
//     }
//     case POWER_CYCLE_2_ON:{
   
//       digitalWrite(POWER_PIN, LOW);  // 第二次断电
//       currentState = POWER_CYCLE_3_OFF;
    
//       Serial.println("Power cycle 2: OFF");
//       delay(POWER_CYCLE_DELAY);
//       break;
//     }
//     case POWER_CYCLE_3_OFF:{
      
//       digitalWrite(POWER_PIN, HIGH);  // 第三次通电
//       currentState = POWER_CYCLE_3_ON;
     
//       Serial.println("Power cycle 3: ON");
//       delay(POWER_CYCLE_DELAY);
//       break;
//     }
//     case POWER_CYCLE_3_ON:{
   
//       currentState =PAIRING_COMPLETE;
//       Serial.println("Pairing sequence complete. Ready to receive data.");
//       //delay(POWER_CYCLE_DELAY);
//       break;
//     }
   
//     default:{
//       currentState = PAIRING_COMPLETE;
//       break;
//     }
//   }
// }

void parseFrame(uint8_t* frame, int length) {
  // 验证CRC校验（关键步骤）
    // uint8_t crc = crc8(frame, length - 1);  // 计算接收数据的CRC
    // if (crc != frame[length - 1]) {
    //     Serial.println("CRC mismatch");
    //     return;
    // }


    // 解析通道数据（每个通道11位，共8个通道）
    // 解析通道数据（按照Python代码的位操作逻辑）
    channels.roll    = ((frame[4] << 8) | frame[3]) & 0x07FF;  //正确
    channels.pitch   =((frame[4]>>3) | (frame[5]<<5)) & 0x07FF;// ((frame[4] << 8) | frame[5]) & 0x07FF; 
    channels.throttle =((frame[5]>>6) | (frame[6]<<2) | (frame[7]<<10)) & 0x07FF;// ((frame[5] << 8) | frame[6]) & 0x07FF;
    channels.yaw     = ((frame[7]>>1) | (frame[8]<<7)) & 0x07FF;//((frame[8] << 8) | frame[7]) & 0x0FFF;
    channels.aux1    = ((frame[8]>>4) | (frame[9]<<4)) & 0x07FF;//((frame[9] << 8) | frame[10]) & 0x07FF;
    channels.aux2    =((frame[9]>>7) | (frame[10]<<1) | (frame[11]<<9)) & 0x07FF; //((frame[11] << 8) | frame[12]) & 0x07FF;
    channels.aux3    = ((frame[11]>>2) | (frame[12]<<6)) & 0x07FF;//((frame[9] << 8) | frame[10]) & 0x07FF;
    channels.aux4    =((frame[12]>>5) | (frame[13]<<3)) & 0x07FF; //((frame[11] << 8) | frame[12]) & 0x07FF;
    
    // 调试输出
    // if(client.connected()){
    // client.printf("Channels: %d %d %d %d | AUX: %d %d\n",
    //     channels.roll, channels.pitch, 
    //     channels.throttle, channels.yaw,
    //     channels.aux1, channels.aux2);
    // }
}
uint8_t crc8(uint8_t *data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;  // ELRS使用的特定多项式
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
/*
//SD卡函数
// 可以在某个条件满足时执行卸载
void cleanUpSD() {
    if(dataFile) {
        dataFile.close();
        Serial.println("数据文件已关闭");
    }
    SD_MMC.end();
    Serial.println("SD卡已卸载");
}
*/