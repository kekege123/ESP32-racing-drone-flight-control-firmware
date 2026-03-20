#ifndef _ICM42688_H
#define _ICM42688_H
#include <Arduino.h>
#include <SPI.h>
#define ICM_CS_PIN 5
#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_SCK  18
void setupSPI();
void writeRegister(uint8_t reg, uint8_t value);
void readRegisters(uint8_t reg, uint8_t *buf, uint16_t len);
void initICM42688();

#endif