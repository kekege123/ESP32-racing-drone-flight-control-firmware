#include <Arduino.h>
#include "SD_CARD.h"
void SD_INIT(){
// 初始化4位SD总线模式，不设置"true"参数即默认为4位模式 [1,3](@ref)
    if (!SD_MMC.begin()) {
        Serial.println("SD Card Mount Failed");
        return;
    }

    // 获取并打印卡信息
    uint8_t cardType = SD_MMC.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        return;
    }
    Serial.print("SD Card Type: ");
    if (cardType == CARD_MMC) Serial.println("MMC");
    else if (cardType == CARD_SD) Serial.println("SDSC");
    else if (cardType == CARD_SDHC) Serial.println("SDHC");
    else Serial.println("UNKNOWN");

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
}