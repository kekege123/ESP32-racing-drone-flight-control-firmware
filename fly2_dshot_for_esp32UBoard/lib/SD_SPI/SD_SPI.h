#ifndef _SD_SPI_H
#define _SD_SPI_H
#include <Arduino.h>
#include <SPI.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#define SD_CS   2
#define SD_CD 21
class MySD {
public:
  // 关键的构造函数：接收一个 SPIClass 类型的引用
  MySD(SPIClass& spiInterface);
  void InitSD();

private:
  SPIClass& _spi; // 保存对HSPI对象的引用
 
};
// void InitSD();
int CDState();
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void createDir(fs::FS &fs, const char * path);
void removeDir(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void deleteFile(fs::FS &fs, const char * path);
void testFileIO(fs::FS &fs, const char * path);

#endif
