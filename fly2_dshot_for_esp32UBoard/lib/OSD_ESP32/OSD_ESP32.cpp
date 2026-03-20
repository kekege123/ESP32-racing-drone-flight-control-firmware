#include "OSD_ESP32.h"
MyAT7456::MyAT7456(SPIClass& spiInterface)
  : _spi(spiInterface) {
  // 初始化列表确保了 _spi 引用在对象构造时就被正确设置
}
/* 全局变量 */
bool OSD_EN = 1;
bool OSD_WARNING_CLEAR = 1;
bool OSD_MSG_CLEAR = 0;
bool OSD_FLYMODE_SWITCHED = 1;
uint8_t lastFlyMode = 1;
const float Bat_alarm = 3.7; // 低电压阈值
/*
void spi0_init(void) {
    pinMode(AT7456_CS, OUTPUT);
    digitalWrite(AT7456_CS, HIGH); // CS 高电平不选中
    
    SPI.begin(SPI0_PIN_SCK, SPI0_PIN_MISO, SPI0_PIN_MOSI, AT7456_CS); // 无 MISO
    SPI.setFrequency(4000000); // 4 MHz
}
*/
/*
void setupHSPI() {
  pinMode(AT7456_CS, OUTPUT);
  digitalWrite(AT7456_CS, HIGH); // CS 高电平不选中
  hspi.begin(PIN_SCK_H, PIN_MISO_H, PIN_MOSI_H); // 片选引脚通常由用户控制，此处可不传入
  // 为HSPI设备设置通信参数（10MHz, 模式0, MSB优先）
  hspi.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
}
*/
static inline void CS0() {
    digitalWrite(AT7456_CS, LOW);
    delayMicroseconds(1);
}

static inline void CS1() {
    delayMicroseconds(1);
    digitalWrite(AT7456_CS, HIGH);
}

void MyAT7456::at7456_write_addr_data(uint8_t addr, uint8_t dat) {
    CS0();
	// digitalWrite(AT7456_CS, LOW);
    _spi.transfer(addr & 0x7F); // 移除读标志位
    _spi.transfer(dat);
	// digitalWrite(AT7456_CS, HIGH);
    CS1();
}

uint8_t MyAT7456::at7456_init(void) {
    // spi0_init();
    // delay(50); // 上电复位后等待
    uint8_t version = at7456_check_version();
    if(version == 0) Serial.print("初始化失败");//return 1; // 初始化失败
    // 直接配置，不进行版本检查（无 MISO）
    at7456_write_addr_data(VM0, 0x42); // NTSC, 自动同步
    // delay(1);
	/*
    at7456_write_addr_data(VM0, 0x48); // PAL 制式
    */
   	at7456_write_addr_data(VM0, 0b01000001); // 启用OSD，NTSC内同步模式
	// 方案B：如果您知道/确定摄像头是PAL制式
    // at7456_write_addr_data(VM0, 0b01000010); // 启用OSD，PAL内同步模式

	
   	// 等待视频信号，最多尝试10次
    // uint8_t retries = 0;
    // bool videoDetected = false;
    
    // while (retries < 10) {
    //     uint8_t stat = at7456_read_addr(STAT);
        
    //     // 检查视频信号
    //     if (stat & (PAL_DETECT | NTSC_DETECT)) {
    //         videoDetected = true;
    //         break;
    //     }
        
    //     // delay(50); // 等待50ms
    //     retries++;
    // }
    
    // if (!videoDetected) {
    //     Serial.println("AT7456未检测到视频信号!");
    //     return 1; // 初始化失败
    // }
    // // 根据检测到的制式设置VM0
    // uint8_t stat = at7456_read_addr(STAT);
    // if (stat & PAL_DETECT) {
    //     at7456_write_addr_data(VM0, 0x48); // PAL
    // } else {
    //     at7456_write_addr_data(VM0, 0x42); // NTSC
    // }

    


	// 修改背景设置
    // at7456_write_addr_data(VM1, BACKGND_0 | BLINK_TIME120);
    // // ✅ 新增：全局关闭背景驱动
    // for(int i=0; i<16; i++) {
    //     at7456_write_addr_data(RB0 + i, BLACK_LEVEL_0);
    // }
    // at7456_clearSRAM();
    Serial.println("AT7456 Initialized ");
    return 0;
}

void MyAT7456::at7456_clearSRAM(void) {
    at7456_write_addr_data(DMM, (1 << 2)); // 清屏标志
    delayMicroseconds(40);
}

void MyAT7456::at7456_writeSRAM(uint8_t row, uint8_t columns, uint8_t addr) {
    uint16_t address = row * 30 + columns;
	if (at7456_read_addr(STAT) & VSYNC) { // 检查垂直同步信号
    at7456_write_addr_data(DMAH, address >> 8);
    at7456_write_addr_data(DMAL, address & 0xFF);
    at7456_write_addr_data(DMDI, addr);
	}
    
}

void MyAT7456::at7456_OSD_on(void) {
    at7456_write_addr_data(VM0, 0x48); // 开启 OSD
    delayMicroseconds(10);
}

void MyAT7456::at7456_OSD_off(void) {
    at7456_write_addr_data(VM0, 0x40); // 关闭 OSD
    delayMicroseconds(30);
}
uint8_t MyAT7456::at7456_read_addr(uint8_t addr) {
    CS0();
	// digitalWrite(AT7456_CS, LOW);
    _spi.transfer(addr | 0x80); // 设置读标志位
	// delayMicroseconds(10);
    uint8_t data = _spi.transfer(0);
	// digitalWrite(AT7456_CS, HIGH);
    CS1();
    return data;
}

uint8_t MyAT7456::at7456_check_version(void) {
    uint8_t version = at7456_read_addr(0x00); // 读取VM0寄存器
    if(version != 0) {
        Serial.printf("AT7456E Detected. Version: 0x%02X\n", version);
        return version;
    }
    Serial.println("AT7456E Not Detected!");
    return 0;
}
// 检查AT7456状态寄存器
bool MyAT7456::at7456_check_status() {
    uint8_t stat = at7456_read_addr(STAT);
    
    // 调试输出状态寄存器值
    // Serial.printf("AT7456 STAT: 0x%02X\n", stat);
    
    // 检查关键错误标志
    if (stat & (1 << 7)) {   // D7: Busy (设备忙)
        Serial.println("AT7456 Busy");
        return false;
    }
    if (stat & (1 << 2)) {   // D2: LOS (Loss of Signal, 信号丢失)
        Serial.println("AT7456 Signal Lost");
        return false;
    }
    
    // 检查视频模式检测
    if (!(stat & (1 << 0)) && !(stat & (1 << 1))) { // D0: PAL检测, D1: NTSC检测
        Serial.println("AT7456 No Video Signal Detected");
        return false;
    }
    
    // 检查VSYNC状态 (可选)
    if (!(stat & (1 << 4))) { // D4: VSYNC (场同步)
        // 这不是严重错误，但可能影响显示时机
        // Serial.println("AT7456 VSYNC Not Detected");
    }
    
    return true;
}
/* 字符叠加演示函数 */
void MyAT7456::demo_single_char(void) {
    Serial.println("Displaying single character...");
    at7456_clearSRAM();
    at7456_writeSRAM(7, 14, 0x41); // 中心显示点字符
    at7456_OSD_on();
    delay(2000);
}

void MyAT7456::demo_battery_voltage(void) {
    Serial.println("Displaying battery voltage...");
    at7456_clearSRAM();
    
    // 电池标签
    at7456_writeSRAM(0, 6, 0x26); // b
    at7456_writeSRAM(0, 7, 0x25); // a
    at7456_writeSRAM(0, 8, 0x38); // t
    
    // 电压值 (12.34V)
    at7456_writeSRAM(0, 10, 0x01); // 1
    at7456_writeSRAM(0, 11, 0x02); // 2
    at7456_writeSRAM(0, 12, 0x41); // .
    at7456_writeSRAM(0, 13, 0x03); // 3
    at7456_writeSRAM(0, 14, 0x04); // 4
    at7456_writeSRAM(0, 15, 0x20); // V
    
    at7456_OSD_on();
    delay(3000);
}

void MyAT7456::demo_complex_display(void) {
    Serial.println("Displaying complex OSD...");
    at7456_clearSRAM();
    
    // 飞行模式
    at7456_writeSRAM(0, 0, 0x12); // H
    at7456_writeSRAM(0, 1, 0x19); // O
    at7456_writeSRAM(0, 2, 0x1C); // R
    
    // 电池电压
    at7456_writeSRAM(0, 6, 0x26); // b
    at7456_writeSRAM(0, 7, 0x25); // a
    at7456_writeSRAM(0, 8, 0x38); // t
    at7456_writeSRAM(0, 9, 0x44); // :
    at7456_writeSRAM(0, 10, 0x01); // 1
    at7456_writeSRAM(0, 11, 0x02); // 2
    at7456_writeSRAM(0, 12, 0x41); // .
    at7456_writeSRAM(0, 13, 0x03); // 3
    at7456_writeSRAM(0, 14, 0x04); // 4
    at7456_writeSRAM(0, 15, 0x20); // V
    
    // 中心十字线
    at7456_writeSRAM(7, 13, 0x41); // ·
    at7456_writeSRAM(7, 15, 0x41); // ·
    at7456_writeSRAM(6, 14, 0x41); // ·
    at7456_writeSRAM(8, 14, 0x41); // ·
    
    // 底部状态
    at7456_writeSRAM(15, 0, 0x1E); // T
    at7456_writeSRAM(15, 1, 0x3A); // :
    at7456_writeSRAM(15, 2, 0x00); // 3
    at7456_writeSRAM(15, 3, 0x02); // 2
    at7456_writeSRAM(15, 4, 0x0D); // C
    
    at7456_OSD_on();
    delay(5000);
}

void MyAT7456::OSD_init(void) {
    // 简化的初始化显示
    at7456_writeSRAM(0, 0, 0x0B); // A
    at7456_writeSRAM(0, 1, 0x1D); // S
    at7456_writeSRAM(0, 2, 0x1D); // S
    at7456_writeSRAM(0, 3, 0x13); // I
    at7456_writeSRAM(0, 4, 0x1E); // T
    at7456_writeSRAM(0, 5, 0x00); // 
    
    at7456_writeSRAM(0, 7, 0x1E); // T
    at7456_writeSRAM(0, 8, 0x1D); // S
    at7456_writeSRAM(0, 9, 0x01); // 1
    
    at7456_OSD_on();
    Serial.println("OSD Initialized");
}

// 其他函数（OSD_displyInt等）保持不变...
void MyAT7456::OSD_displyInt(uint8_t row,uint8_t columns,uint8_t number_len,long number)
{
	uint8_t k,m,h,i;
	uint8_t Knumber[8]={0};//最大显示10位数据
	k=0;
	h=0;
	if(number==0)//数据=0
	{//每个位上都显示0
		for(i=0;i<number_len;i++)
		{
			m=columns+i;
 			at7456_writeSRAM(row,m,0x0A);//显示0
		}
	}
	else if(number>0)//数据>0
	{
		while(number>0)//分离各位上的数字
		{
			Knumber[k]=number%10;
			number=number/10;
			k=k+1;
		}
		if(k>number_len)	k=number_len;
		h=k;
		for(i=0;i<number_len;i++ )//显示数字
		{
			m=number_len+columns-i-1;
			if(Knumber[i]==0)				
				at7456_writeSRAM(row,m,0x0A);//显示0
			else
				at7456_writeSRAM(row,m,Knumber[i]);//显示数字
		}
	}
	else if(number<0)//数据<0
	{
		long m_Abs_vaule;
		m_Abs_vaule=fabs(number);
		while(m_Abs_vaule>0)
		{
			Knumber[k]=m_Abs_vaule%10;
			m_Abs_vaule=m_Abs_vaule/10;
			k=k+1;
		}  
		//显示负号
		m=columns;
		at7456_writeSRAM(row,m,0x49);//显示”-“

		for(i=0;i<(number_len-1);i++ )
		{
			m=number_len+columns-i-1;
			if(Knumber[i]==0)					 
				at7456_writeSRAM(row,m,0x0A);//显示0
			else
				at7456_writeSRAM(row,m,Knumber[i]);//显示数字
		}
	}
}
/*****显示实数，无0占位**********************/
/***参数：row：        显示行数0~15**********************/
/***参数：columns：    显示起始列数0~29******************/
/***参数：number_len： 整数占位数，负号占1位*************/
/***参数：number：     需要显示的实数********************/
void MyAT7456::OSD_displyInt_1(uint8_t row,uint8_t columns,uint8_t number_len,long number)
{
	uint8_t k,m,i;
	uint8_t Knumber[8]={0};//最大显示10位数据
	k=0;
	if(number==0)//数据=0
	{		
		m=columns+number_len-1;
		at7456_writeSRAM(row,m,0x0A);////最后一位上显示0
		for(i=0;i<(number_len-1);i++)
		{
			m=columns+i;
			at7456_writeSRAM(row,m,0x00);////清除空位
		}
	}
	else if(number>0)//数据>0
	{
		while(number>0)//分离各位上的数字
		{
			Knumber[k]=number%10;
			number=number/10;
			k=k+1;
		}
		if(k>number_len) k=number_len;
		for(i=0;i<k;i++ )//显示数字
		{
			m=number_len+columns-1-i;
			if(Knumber[i]==0)				
				at7456_writeSRAM(row,m,0x0A);//显示0
			else
				at7456_writeSRAM(row,m,Knumber[i]);//显示数字
		}
		if(k<number_len)
		{
			for(i=0;i<number_len-k;i++)
			{
				m=columns+i;
				at7456_writeSRAM(row,m,0x00);//清除空位
			}
		}
		 
	}
	else if(number<0)//数据<0
	{
		long m_Abs_vaule;
		m_Abs_vaule=fabs(number);
		while(m_Abs_vaule>0)
		{
			Knumber[k]=m_Abs_vaule%10;
			m_Abs_vaule=m_Abs_vaule/10;
			k=k+1;
		} 
		if(k>(number_len-1)) k=number_len-1;
		//显示数字
		for(i=0;i<k;i++ )
		{
			m=number_len+columns-i-1;
			if(Knumber[i]==0)					 
				at7456_writeSRAM(row,m,0x0A);//显示0
			else
				at7456_writeSRAM(row,m,Knumber[i]);//显示数字
		}
		if(k<(number_len-1))
		{
			for(i=0;i<(number_len-k-1);i++)
			{
				m=columns+i+1;
				at7456_writeSRAM(row,m,0x00);//清除空位
			}
		}
		//显示负号
		at7456_writeSRAM(row,columns,0x49);//显示”-“
   }
}
/*****显示浮点数,无0占位**********************/
/***参数：row：    显示行数0~15********************/
/***参数：columns：显示起始列数0~290~29************/
/***参数：int_n：  整数占位数，负号占1位***********/
/***参数：flaot_n：小数点后几位********************/
/***参数：number： 显示的数字**********************/
void MyAT7456::OSD_disply_Float(uint8_t row,uint8_t columns,uint8_t int_n,uint8_t flaot_n,float number)
{
	uint8_t i,m;
	uint8_t float_Knumber;//小数最大5位;	
	if(number==0)//数据=0
	{
		m=columns+int_n+flaot_n;
		at7456_writeSRAM(row,m,0x0A);//显示0
	}
	else
	{
		long int_data;
		float m_float_data;
		int_data=number;//分离整数
		//分离小数
		if(number>0)//数据>0
			m_float_data=number-int_data;//分离小数
		else
		{
			float f_number;
			int   I_number;
			I_number=0-number;
			f_number=0-number;		
			m_float_data=f_number-I_number;//分离小数
		}
		//显示小数
		for(i=0;i<flaot_n;i++)
		{
			m_float_data=m_float_data*10;
			float_Knumber=m_float_data;
			m=columns+int_n+i+1;
			if(float_Knumber==0)
				at7456_writeSRAM(row,m,0x0A);
			else
				at7456_writeSRAM(row,m,float_Knumber);
			m_float_data=m_float_data-float_Knumber;
		}
		//处理小数部分显示
		m=columns+int_n;
		at7456_writeSRAM(row,m,0x41);//显示小数点
		//显示整数部分		
		OSD_displyInt_1(row,columns,int_n,int_data);	
		if(number<0 && number >-1)
			at7456_writeSRAM(row,columns,0x49);//显示”-“
	 }

}
//内部函数，显示低电压报警信息
void MyAT7456::dispWarning_batV(void){
	at7456_writeSRAM(10,10,0x16);//L
	at7456_writeSRAM(10,11,0x33);//o
	at7456_writeSRAM(10,12,0x3B);//w

	at7456_writeSRAM(10,14,0x26);//b
	at7456_writeSRAM(10,15,0x25);//a
	at7456_writeSRAM(10,16,0x38);//t
	at7456_writeSRAM(10,17,0x20);//V

}
//内部函数，显示低信号报警信息
void MyAT7456::dispWarning_RSSI(void){
	at7456_writeSRAM(10,10,0x16);//L
	at7456_writeSRAM(10,11,0x33);//o
	at7456_writeSRAM(10,12,0x3B);//w

	at7456_writeSRAM(10,14,0x1C);//R
	at7456_writeSRAM(10,15,0x1D);//S
	at7456_writeSRAM(10,16,0x1D);//S
	at7456_writeSRAM(10,17,0x13);//I
}
//内部函数，清除报警信息
void MyAT7456::dispWarning_clear(void){
	at7456_writeSRAM(10,10,0x00);//
	at7456_writeSRAM(10,11,0x00);//
	at7456_writeSRAM(10,12,0x00);//

	at7456_writeSRAM(10,14,0x00);//
	at7456_writeSRAM(10,15,0x00);//
	at7456_writeSRAM(10,16,0x00);//
	at7456_writeSRAM(10,17,0x00);//
}

void MyAT7456::dispMsg_disarm(uint8_t mode){
	if(mode == 1){
		at7456_writeSRAM(4,11,0x0E);//D
		at7456_writeSRAM(4,12,0x13);//I
		at7456_writeSRAM(4,13,0x1D);//S
		at7456_writeSRAM(4,14,0x0B);//A
		at7456_writeSRAM(4,15,0x1C);//R
		at7456_writeSRAM(4,16,0x17);//M
		at7456_writeSRAM(4,17,0x0F);//E
		at7456_writeSRAM(4,18,0x0E);//D
	}
	else{
		at7456_writeSRAM(4,11,0x00);//D
		at7456_writeSRAM(4,12,0x00);//I
		at7456_writeSRAM(4,13,0x00);//S
		at7456_writeSRAM(4,14,0x00);//A
		at7456_writeSRAM(4,15,0x00);//R
		at7456_writeSRAM(4,16,0x00);//M
		at7456_writeSRAM(4,17,0x00);//E
		at7456_writeSRAM(4,18,0x00);//D
	}
}
//OSD显示更新函数，在50hz循环中调用
void MyAT7456::OSD_update(uint8_t mode, float batV, bool RSSI, uint8_t throt, float pitch, float roll, float coreTemp, bool lock){
	//1)模式显示，1为HOR自稳模式；2为ARCO手动模式
	if(mode != lastFlyMode) OSD_FLYMODE_SWITCHED = 1;
	lastFlyMode = mode; //更新上次模式
	if(OSD_FLYMODE_SWITCHED == 1){
		if(mode == 1) {
			//后续可以改为Angle，暂时先不动
			//at7456_writeSRAM(0,0,0x12);//H
			//at7456_writeSRAM(0,1,0x19);//O
			//at7456_writeSRAM(0,2,0x1C);//R	
			//at7456_writeSRAM(0,3,0x00);//第4位清除显示	
			at7456_writeSRAM(0,0,0x12);//H
			at7456_writeSRAM(0,1,0x19);//O
			at7456_writeSRAM(0,2,0x1C);//R	
			at7456_writeSRAM(0,3,0x00);//第4位清除显示
		}
		else if(mode == 2){
			at7456_writeSRAM(0,0,0x0B);//A
			at7456_writeSRAM(0,1,0x0D);//C
			at7456_writeSRAM(0,2,0x1C);//R
			at7456_writeSRAM(0,3,0x19);//O	
		}
		else{
			at7456_writeSRAM(0,0,0x42);//?
			at7456_writeSRAM(0,1,0x42);//?
			at7456_writeSRAM(0,2,0x42);//?
			at7456_writeSRAM(0,3,0x42);//?	
		}
		OSD_FLYMODE_SWITCHED = 0;
	}
	//2)电池电压显示
	OSD_disply_Float(0,10,2,2,batV);//显示电池电压
	//3)信号强度显示,1强，0弱
	if(RSSI == 1){
	    at7456_writeSRAM(0,26,0x4B);//显示信号阈值 > 
	}
	else if(RSSI == 0){
		at7456_writeSRAM(0,26,0x4A);//显示信号阈值 < 
	}
	//低电压、低信号强度告警显示：会存在当同时告警，但只显示RSSI的情况，但不影响，将RSSI第一个判断，提高其优先级
	if((RSSI == 0)&&(OSD_WARNING_CLEAR)){
		dispWarning_RSSI();
		OSD_WARNING_CLEAR = 0;//未清屏置位
	}
	else if((batV < Bat_alarm)&&(OSD_WARNING_CLEAR)){
		dispWarning_batV();
		OSD_WARNING_CLEAR = 0;//未清屏置位
	}
	else if(!OSD_WARNING_CLEAR){
		dispWarning_clear();
		OSD_WARNING_CLEAR = 1;//已清屏置位
	}
	//4)油门及姿态显示
	OSD_displyInt_1(13,1,4,throt);//显示油门值T
    OSD_disply_Float(14,1,4,1,pitch);//显示俯仰值P
    OSD_disply_Float(15,1,4,1,roll);//显示俯仰值R
	//5)CPU核心温度显示
    OSD_disply_Float(15,13,2,1,coreTemp);//显示核心温度
	//6)锁定状态显示，加标志位判断的目的是避免不需要清屏的时候，重复写入
	if(lock == 1){
		dispMsg_disarm(1);
		OSD_MSG_CLEAR = 0; //屏幕消息未清除
	}
	else{
		if(OSD_MSG_CLEAR == 0){
			dispMsg_disarm(0); //
			OSD_MSG_CLEAR = 1; //屏幕消息已清除
		}
	}

    at7456_OSD_on();
}
// AT7456字符映射表（根据你的代码总结）
uint8_t MyAT7456::charToAt7456(char c) {
    switch(c) {
        case '0': return 0x0A;  // 数字0
        case '1': return 0x01;  // 数字1
        case '2': return 0x02;  // 数字2
        case '3': return 0x03;  // 数字3
        case '4': return 0x04;  // 数字4
        case '5': return 0x05;  // 数字5
        case '6': return 0x06;  // 数字6
        case '7': return 0x07;  // 数字7
        case '8': return 0x08;  // 数字8
        case '9': return 0x09;  // 数字9
        case '.': return 0x41;  // 小数点
		case '-': return 0x49;  // 减号/负号
        case '+': return 0x4B;  // 加号/正号
        case 'V': return 0x20;  // 电压单位V
		case 'A': return 0x0B;
		case 'B': return 0x0C;
		case 'C': return 0x0D;
		case 'D': return 0x0E;
		case 'E': return 0x0F;
		case 'F': return 0x10;
		case 'G': return 0x11;
		case 'H': return 0x12;
		case 'I': return 0x13;
		case 'J': return 0x14;
		case 'K': return 0x15;
		case 'L': return 0x16;
		case 'M': return 0x17;
		case 'N': return 0x18;
		case 'O': return 0x19;
		case 'P': return 0x1A;
		case 'Q': return 0x1B;
		case 'R': return 0x1C;
		case 'S': return 0x1D;
		case 'T': return 0x1E;
		case 'U': return 0x1F;
		case 'W': return 0x21;
		case 'X': return 0x22;
		case 'Y': return 0x23;
		case 'Z': return 0x24;
		case 'a': return 0x25;
		case 'b': return 0x26;
		case 'c': return 0x27;
		case 'd': return 0x28;
		case 'e': return 0x29;
		case 'f': return 0x2A;
		case 'g': return 0x2B;
		case 'h': return 0x2C;
		case 'i': return 0x2D;
		case 'j': return 0x2E;
		case 'k': return 0x2F;
		case 'l': return 0x30;
		case 'm': return 0x31;
		case 'n': return 0x32;
		case 'o': return 0x33;
		case 'p': return 0x34;
		case 'q': return 0x35;
		case 'r': return 0x36;
		case 's': return 0x37;
		case 't': return 0x38;
		case 'u': return 0x39;
		case 'v': return 0x3A;
		case 'w': return 0x3B;
		case 'x': return 0x3C;
		case 'y': return 0x3D;
		case 'z': return 0x3E;
		case ':': return 0x44;
        // 添加更多字符映射...
        default:  return 0x00;  // 空格（未定义字符）
    }
}

void MyAT7456::voltageToAt7456Chars(float voltage, uint8_t* outputArray, int maxSize) {
    // 格式化为字符串 (XX.XXV)
    char voltageStr[7];
    snprintf(voltageStr, sizeof(voltageStr), "%.2fV", voltage);
    
    // 转换为AT7456字符地址
    int len = strlen(voltageStr);
    for (int i = 0; i < len && i < maxSize; i++) {
        outputArray[i] = charToAt7456(voltageStr[i]);
    }
}