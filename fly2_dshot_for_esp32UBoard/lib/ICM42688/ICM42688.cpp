#include "ICM42688.h"
void setupSPI() {
  // pinMode(AT7456_CS, OUTPUT);
  // digitalWrite(AT7456_CS, HIGH); // CS 高电平不选中
  pinMode(ICM_CS_PIN, OUTPUT);
  digitalWrite(ICM_CS_PIN, HIGH);  // 初始禁用
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0)); // 10MHz时钟
  // hspi.begin(PIN_SCK_H, PIN_MISO_H, PIN_MOSI_H); // 片选引脚通常由用户控制，此处可不传入
  // // 为HSPI设备设置通信参数（10MHz, 模式0, MSB优先）
  // hspi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
}

void writeRegister(uint8_t reg, uint8_t value) {
  digitalWrite(ICM_CS_PIN, LOW);
  SPI.transfer(reg & 0x7F); // 最高位=0（写操作）
  SPI.transfer(value);
  digitalWrite(ICM_CS_PIN, HIGH);
}

// SPI多字节读取寄存器
void readRegisters(uint8_t reg, uint8_t *buf, uint16_t len) {
  digitalWrite(ICM_CS_PIN, LOW);
  SPI.transfer(reg | 0x80); // 最高位=1（读操作）
  while (len--) *buf++ = SPI.transfer(0x00);
  digitalWrite(ICM_CS_PIN, HIGH);
}
void initICM42688() {
  // 1. 复位设备
  writeRegister(0x11, 0x01); // DEVICE_CONFIG寄存器写入复位命令
  delay(10); // 等待复位完成

  // 2. 验证设备ID (应为0x47)
  uint8_t id;
  readRegisters(0x75, &id, 1); // WHO_AM_I寄存器地址=0x75
  if (id != 0x47) Serial.println("ID校验失败！");
  // writeRegister(0x4E, 0x00);  // 进入待机模式，   配置量程和ODR（示例：加速度±4g，陀螺仪±500dps，32kHz输出）
  // writeRegister(0x50, 0x01);  // 二进制00000001
  // writeRegister(0x4F, 0x01);  // 二进制00000001
  // writeRegister(0x51, 0x03);  // 二进制00000011
  // // 3. 启用传感器（低噪声模式）
  // writeRegister(0x4E, 0x0F); // PWR_MGMT0: 陀螺仪+加速度计均启用LN模式
  // delay(100); // 等待传感器稳定
  // 1. 设置加速度计：±4g@32kHz
  writeRegister(0x50, 0b01000001);  // ACCEL_CONFIG0: FS=010(4g), ODR=0001(32kHz)
  
  // 2. 设置陀螺仪：±500dps@32kHz
  writeRegister(0x4F, 0b01000001);  // GYRO_CONFIG0: FS=010(500dps), ODR=0001(32kHz)
  
  // 3. 启用低噪声模式
  uint8_t pwr_val;
  readRegisters(0x4E,&pwr_val,1); // 读取当前PWR_MGMT0值
  writeRegister(0x4E, (pwr_val & 0xF3) | 0x0C); // ACCEL_MODE=11 (LN)
  readRegisters(0x4E,&pwr_val,1); 
  writeRegister(0x4E, (pwr_val & 0xFC) | 0x03); // GYRO_MODE=11 (LN)
  
  // 4. 启用抗混叠滤波器
  writeRegister(0x76, 0x01);       // REG_BANK_SEL=1 (切换到Bank1)
  writeRegister(0x0B, 0x00);       // 启用陀螺仪AAF
  writeRegister(0x76, 0x00);       // REG_BANK_SEL=0 (返回Bank0)
}