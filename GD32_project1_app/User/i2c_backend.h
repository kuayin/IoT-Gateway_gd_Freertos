/* i2c_backend.h
 * 全系统唯一的 I2C 后端选择开关。
 *   0 = 硬件 I2C0 (PB6/PB7, AF_OD)
 *   1 = 软件 GPIO 模拟 I2C (PB6/PB7, OUT_OD)
 * 两者共用同一组引脚，全系统必须统一，不可 OLED 用软、sensor 用硬。
 */
#ifndef I2C_BACKEND_H
#define I2C_BACKEND_H

#ifndef USE_SW_I2C
#define USE_SW_I2C   1        /* 默认硬件；切软改这里为 1 */
#endif

#include "i2c_types.h"

#if USE_SW_I2C
#include "i2c_sw.h"
#define i2c_bus_init()                sw_i2c_init()
#define i2c_bus_mem_write(a,r,b,l)    sw_i2c_mem_write((a),(r),(b),(l))
#define i2c_bus_mem_read(a,r,b,l)     sw_i2c_mem_read((a),(r),(b),(l))
#define i2c_bus_raw_read(a,b,l)       sw_i2c_raw_read((a),(b),(l))
#else
#include "i2c_hw.h"
#define i2c_bus_init()                hw_i2c_init()
#define i2c_bus_mem_write(a,r,b,l)    hw_i2c_mem_write((a),(r),(b),(l))
#define i2c_bus_mem_read(a,r,b,l)     hw_i2c_mem_read((a),(r),(b),(l))
#define i2c_bus_raw_read(a,b,l)       hw_i2c_raw_read((a),(b),(l))
#endif

#endif