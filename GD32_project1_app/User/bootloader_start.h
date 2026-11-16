#ifndef BOOTLOADER_START
#define BOOTLOADER_START
#include "gd32f10x.h"
#include <stdio.h>
#define UPGRADE_MAGIC_FLAG  0x5A5AA5A5U
/**
 * @brief 写入 BKP 升级标志并软复位
 */
void app_trigger_ota(void);

#endif