/**
 * @file    sensor.h
 * @brief   AHT20 温湿度 + BMP280 气压/温度/海拔，FreeRTOS 版（GD32F103）
 *
 * 使用步骤：
 *   1. sensor_os_init()  调度器启动前调用一次（创建模块互斥量）
 *   2. sensor_init()     启动任务中调用（内部 vTaskDelay，约 2s 最坏）
 *   3. sensor_read()     单个采样任务周期调用，结果分发给消费任务
 */
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"

/* ===== 选择 I2C 后端 ===== */
#include "i2c_backend.h"

/* ===== 器件地址 ===== */
#define AHT20_ADDR          0x38U
#define BMP280_ADDR         0x77U    /* SDO=GND:0x76  SDO=VDDIO:0x77 */

/* ===== 状态码 ===== */
typedef enum {
    SENSOR_OK = 0,
    SENSOR_ERR_PARAM,
    SENSOR_ERR_I2C,
    SENSOR_ERR_NOT_READY,
    SENSOR_ERR_TIMEOUT,
    SENSOR_ERR_DATA,
    SENSOR_ERR_OS
} sensor_status_t;

/* ===== 数据结构 ===== */
#define SENSOR_VALID_AHT20   0x01U
#define SENSOR_VALID_BMP280  0x02U

typedef struct {
    float   temperature;   /* ℃   */
    float   humidity;      /* %RH  */
    float   pressure;      /* hPa  */
    float   altitude;      /* m    */
    uint8_t valid;         /* SENSOR_VALID_xxx 位或 */
} sensor_data_t;

/* ===== 对外接口 ===== */
sensor_status_t sensor_os_init(void);                   /* 调度器前调用 */
sensor_status_t sensor_init(void);                      /* 任务中调用   */
sensor_status_t sensor_read(sensor_data_t *data);       /* 单任务采样   */

sensor_status_t aht20_read(float *temp_c, float *humi_rh);
sensor_status_t bmp280_read(float *pressure_hpa, float *temp_c, float *altitude_m);

#endif /* __SENSOR_H__ */