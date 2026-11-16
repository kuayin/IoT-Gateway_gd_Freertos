#ifndef I2C_SW_H
#define I2C_SW_H

#include "gd32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"

#include "i2c_types.h"

/* 驱动初始化，需在调度器启动前或任务中调用一次 */
void sw_i2c_init(void);

/*
 * 寄存器写：START → 地址(W) → reg → len 字节 buf → STOP
 * len == 0 且 buf == NULL 合法：用于只发一个控制字节的从机
 * （如 AHT20 软复位 0xBA）。
 */
i2c_status_t sw_i2c_mem_write(uint8_t addr_7bit, uint8_t reg,
                               const uint8_t *buf, uint16_t len);

/*
 * 寄存器读：START → 地址(W) → reg → 重复起始 → 地址(R)
 *          → 读 len 字节 → STOP
 */
i2c_status_t sw_i2c_mem_read(uint8_t addr_7bit, uint8_t reg,
                              uint8_t *buf, uint16_t len);

/*
 * 直接读（无寄存器阶段）：START → 地址(R) → 读 len 字节 → STOP
 * 用于 AHT20 的读状态 / 读测量结果。
 */
i2c_status_t sw_i2c_raw_read(uint8_t addr_7bit, uint8_t *buf, uint16_t len);

#endif