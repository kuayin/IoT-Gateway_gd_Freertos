/**
 * @file    sensor.c
 * @brief   AHT20 + BMP280 驱动，FreeRTOS 版
 */
#include "sensor.h"
#include <math.h>
#include <string.h>

/* ================= I2C 后端 ================= */
#if USE_SW_I2C
  #define I2C_MEM_WRITE   sw_i2c_mem_write
  #define I2C_MEM_READ    sw_i2c_mem_read
  #define I2C_RAW_READ    sw_i2c_raw_read
#else
  #define I2C_MEM_WRITE   hw_i2c_mem_write
  #define I2C_MEM_READ    hw_i2c_mem_read
  #define I2C_RAW_READ    hw_i2c_raw_read
#endif

/* ================= 内部常量 ================= */
/* AHT20 */
#define AHT20_CMD_INIT        0xBEU
#define AHT20_CMD_RESET       0xBAU
#define AHT20_CMD_TRIGGER     0xACU

#define AHT20_ST_BUSY         0x80U
#define AHT20_ST_MODE_MASK    0x68U
#define AHT20_ST_CAL_VAL      0x08U

#define AHT20_INIT_DLY_MS     100U
#define AHT20_MEAS_DLY_MS     80U
#define AHT20_BUSY_TO_MS      100U
#define AHT20_RESET_DLY_MS    200U
#define AHT20_CAL_DLY_MS      500U
#define AHT20_RETRY_MAX       3U     /* 启动阻塞 ≈ 2s，可下调 */

/* BMP280 */
#define BMP280_REG_CHIPID     0xD0U
#define BMP280_REG_CALIB      0x88U
#define BMP280_REG_CTRL       0xF4U
#define BMP280_REG_CFG        0xF5U
#define BMP280_REG_PRESS_MSB  0xF7U
#define BMP280_CHIP_ID        0x58U
#define BMP280_CFG_VAL        (5U << 2)                             /* t_sb=1000ms, filter off */
#define BMP280_CTRL_VAL       ((0x05U << 5) | (0x04U << 2) | 0x03U) /* 16x T, 8x P, normal  */

/* 量程合理性 */
#define SENSOR_T_MIN_C        (-40.0f)
#define SENSOR_T_MAX_C        (85.0f)
#define SENSOR_H_MIN_RH       (0.0f)
#define SENSOR_H_MAX_RH       (100.0f)
#define SENSOR_P_MIN_HPA      (300.0f)
#define SENSOR_P_MAX_HPA      (1100.0f)

/* 海拔/滤波参数 */
#define BMP280_SEA_LEVEL_HPA  1015.7f   /* 按当地气象站标定 */
#define BMP280_FIX_TEMP_C     25.0f
#define BMP280_FILTER_NUM     5U
#define BMP280_FILTER_A       0.1f      /* hPa，变化小于该值才更新 */

/* ================= 工具 ================= */
static inline void delay_ms(uint32_t ms)
{
    TickType_t t = pdMS_TO_TICKS(ms);
    vTaskDelay(t ? t : 1);              /* 确保至少让出 1 tick */
}

static inline sensor_status_t i2c_ok(i2c_status_t s)
{
    return (s == I2C_OK) ? SENSOR_OK : SENSOR_ERR_I2C;
}

/* ================= 互斥量 ================= */
static SemaphoreHandle_t s_mtx = NULL;

sensor_status_t sensor_os_init(void)
{
    if (s_mtx == NULL) {
        s_mtx = xSemaphoreCreateMutex();
        if (s_mtx == NULL) return SENSOR_ERR_OS;
    }
    return SENSOR_OK;
}

static inline int  lock(void)   { return (s_mtx && xSemaphoreTake(s_mtx, portMAX_DELAY) == pdTRUE) ? 0 : -1; }
static inline void unlock(void) { if (s_mtx) xSemaphoreGive(s_mtx); }

/* ================= AHT20 ================= */
static sensor_status_t aht20_status(uint8_t *st)
{
    return i2c_ok(I2C_RAW_READ(AHT20_ADDR, st, 1));
}

static sensor_status_t aht20_init_locked(void)
{
    uint8_t param[2] = {0x08U, 0x00U};
    uint8_t st, retry;

    delay_ms(AHT20_INIT_DLY_MS);
    if (i2c_ok(I2C_MEM_WRITE(AHT20_ADDR, AHT20_CMD_INIT, param, 2)) != SENSOR_OK)
        return SENSOR_ERR_I2C;
    delay_ms(AHT20_CAL_DLY_MS);

    for (retry = 0; retry < AHT20_RETRY_MAX; retry++) {
        if (aht20_status(&st) != SENSOR_OK)                     return SENSOR_ERR_I2C;
        if ((st & AHT20_ST_MODE_MASK) == AHT20_ST_CAL_VAL)      return SENSOR_OK;

        /* 校准未就绪：软复位 + 重新初始化 */
        if (i2c_ok(I2C_MEM_WRITE(AHT20_ADDR, AHT20_CMD_RESET, NULL, 0)) != SENSOR_OK)
            return SENSOR_ERR_I2C;
        delay_ms(AHT20_RESET_DLY_MS);

        if (i2c_ok(I2C_MEM_WRITE(AHT20_ADDR, AHT20_CMD_INIT, param, 2)) != SENSOR_OK)
            return SENSOR_ERR_I2C;
        delay_ms(AHT20_CAL_DLY_MS);
    }
    return SENSOR_ERR_NOT_READY;
}

static sensor_status_t aht20_read_locked(float *t_out, float *h_out)
{
    uint8_t trig[2] = {0x33U, 0x00U};
    uint8_t d[7], st;
    uint32_t rh, rt;
    float t, h;
    TickType_t start, to;

    if (i2c_ok(I2C_MEM_WRITE(AHT20_ADDR, AHT20_CMD_TRIGGER, trig, 2)) != SENSOR_OK)
        return SENSOR_ERR_I2C;

    delay_ms(AHT20_MEAS_DLY_MS);

    /* 轮询忙状态，用 tick 计数，与 configTICK_RATE_HZ 无关 */
    start = xTaskGetTickCount();
    to    = pdMS_TO_TICKS(AHT20_BUSY_TO_MS);
    if (to == 0) to = 1;
    for (;;) {
        if (aht20_status(&st) != SENSOR_OK) return SENSOR_ERR_I2C;
        if (!(st & AHT20_ST_BUSY)) break;
        if ((xTaskGetTickCount() - start) >= to) return SENSOR_ERR_TIMEOUT;
        vTaskDelay(1);
    }

    if (i2c_ok(I2C_RAW_READ(AHT20_ADDR, d, 7)) != SENSOR_OK)
        return SENSOR_ERR_I2C;

    /* 湿度 20bit：d[1..3]；温度 20bit：d[3]低4位 + d[4..5] */
    rh = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | ((uint32_t)d[3] >> 4);
    rt = (((uint32_t)(d[3] & 0x0FU)) << 16) | ((uint32_t)d[4] << 8) | (uint32_t)d[5];

    h = (float)rh * 100.0f / 1048576.0f;
    t = (float)rt * 200.0f / 1048576.0f - 50.0f;

    if (h < SENSOR_H_MIN_RH || h > SENSOR_H_MAX_RH) return SENSOR_ERR_DATA;
    if (t < SENSOR_T_MIN_C || t > SENSOR_T_MAX_C)   return SENSOR_ERR_DATA;

    *t_out = t;
    *h_out = h;
    return SENSOR_OK;
}

/* ================= BMP280 ================= */
/* 前 12 字段必须与 0x88 起 24 字节严格一致 */
typedef struct {
    uint16_t T1;
    int16_t  T2, T3;
    uint16_t P1;
    int16_t  P2, P3, P4, P5, P6, P7, P8, P9;
} bmp280_cal_t;

static bmp280_cal_t s_cal;
static int32_t      s_t_fine;
static int32_t      s_raw_p, s_raw_t;

static float        s_fbuf[BMP280_FILTER_NUM];
static uint8_t      s_fidx, s_fcnt;

static sensor_status_t bmp280_init_locked(void)
{
    uint8_t id, v;

    if (i2c_ok(I2C_MEM_READ(BMP280_ADDR, BMP280_REG_CHIPID, &id, 1)) != SENSOR_OK)
        return SENSOR_ERR_I2C;
    if (id != BMP280_CHIP_ID) return SENSOR_ERR_NOT_READY;

    if (i2c_ok(I2C_MEM_READ(BMP280_ADDR, BMP280_REG_CALIB,
                            (uint8_t *)&s_cal, sizeof(s_cal))) != SENSOR_OK)
        return SENSOR_ERR_I2C;

    /* 先写 config，再写 ctrl 进 normal 模式 */
    v = BMP280_CFG_VAL;
    if (i2c_ok(I2C_MEM_WRITE(BMP280_ADDR, BMP280_REG_CFG, &v, 1)) != SENSOR_OK)
        return SENSOR_ERR_I2C;

    v = BMP280_CTRL_VAL;
    if (i2c_ok(I2C_MEM_WRITE(BMP280_ADDR, BMP280_REG_CTRL, &v, 1)) != SENSOR_OK)
        return SENSOR_ERR_I2C;

    s_fidx = 0; s_fcnt = 0;
    return SENSOR_OK;
}

static sensor_status_t bmp280_fetch(void)
{
    uint8_t d[6];
    if (i2c_ok(I2C_MEM_READ(BMP280_ADDR, BMP280_REG_PRESS_MSB, d, 6)) != SENSOR_OK)
        return SENSOR_ERR_I2C;
    s_raw_p = (int32_t)(((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | ((uint32_t)d[2] >> 4));
    s_raw_t = (int32_t)(((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | ((uint32_t)d[5] >> 4));
    return SENSOR_OK;
}

/* 返回 0.01℃ */
static int32_t bmp280_comp_t(int32_t adc)
{
    int32_t v1, v2;
    v1 = (((adc >> 3) - ((int32_t)s_cal.T1 << 1)) * (int32_t)s_cal.T2) >> 11;
    v2 = ((((adc >> 4) - (int32_t)s_cal.T1) *
           ((adc >> 4) - (int32_t)s_cal.T1)) >> 12) * (int32_t)s_cal.T3 >> 14;
    s_t_fine = v1 + v2;
    return (s_t_fine * 5 + 128) >> 8;
}

/* 返回 Q24.8 Pa（除以 256 得 Pa） */
static uint32_t bmp280_comp_p(int32_t adc)
{
    int64_t v1, v2, p;
    v1  = (int64_t)s_t_fine - 128000;
    v2  = v1 * v1 * (int64_t)s_cal.P6;
    v2 += (v1 * (int64_t)s_cal.P5) << 17;
    v2 += ((int64_t)s_cal.P4) << 35;
    v1  = ((v1 * v1 * (int64_t)s_cal.P3) >> 8) + ((v1 * (int64_t)s_cal.P2) << 12);
    v1  = (((((int64_t)1) << 47) + v1) * (int64_t)s_cal.P1) >> 33;
    if (v1 == 0) return 0;

    p  = 1048576 - adc;
    p  = (((p << 31) - v2) * 3125) / v1;
    v1 = ((int64_t)s_cal.P9 * (p >> 13) * (p >> 13)) >> 25;
    v2 = ((int64_t)s_cal.P8 * p) >> 19;
    p  = ((p + v1 + v2) >> 8) + ((int64_t)s_cal.P7 << 4);
    return (uint32_t)p;
}

/* 限幅平均滤波 */
static float bmp280_filter(float in)
{
    float   sum = 0.0f, prev;
    uint8_t i;

    if (s_fcnt < BMP280_FILTER_NUM) {
        s_fbuf[s_fidx] = in;
        s_fidx = (uint8_t)((s_fidx + 1U) % BMP280_FILTER_NUM);
        s_fcnt++;
        for (i = 0; i < s_fcnt; i++) sum += s_fbuf[i];
        return sum / (float)s_fcnt;
    }

    prev = s_fbuf[(s_fidx + BMP280_FILTER_NUM - 1U) % BMP280_FILTER_NUM];
    if (fabsf(in - prev) < BMP280_FILTER_A) {
        s_fbuf[s_fidx] = in;
        s_fidx = (uint8_t)((s_fidx + 1U) % BMP280_FILTER_NUM);
    }
    for (i = 0; i < BMP280_FILTER_NUM; i++) sum += s_fbuf[i];
    return sum / (float)BMP280_FILTER_NUM;
}

static float bmp280_altitude(float p_hpa)
{
    if (p_hpa <= 0.0f) return 0.0f;
    return ((powf(BMP280_SEA_LEVEL_HPA / p_hpa, 0.190263f) - 1.0f) *
            (BMP280_FIX_TEMP_C + 273.15f)) / 0.0065f;
}

static sensor_status_t bmp280_read_locked(float *p_out, float *t_out, float *alt_out)
{
    float p, t;

    if (bmp280_fetch() != SENSOR_OK) return SENSOR_ERR_I2C;

    t = (float)bmp280_comp_t(s_raw_t) / 100.0f;
    p = (float)bmp280_comp_p(s_raw_p) / 25600.0f;     /* Pa -> hPa */

    if (p < SENSOR_P_MIN_HPA || p > SENSOR_P_MAX_HPA) return SENSOR_ERR_DATA;
    if (t < SENSOR_T_MIN_C || t > SENSOR_T_MAX_C)     return SENSOR_ERR_DATA;

    p = bmp280_filter(p);
    *p_out   = p;
    *t_out   = t;
    *alt_out = bmp280_altitude(p);
    return SENSOR_OK;
}

/* ================= 公共接口 ================= */
sensor_status_t sensor_init(void)
{
    sensor_status_t a, b;
    if (lock()) return SENSOR_ERR_OS;

    a = aht20_init_locked();
    b = bmp280_init_locked();

    unlock();

    if (a == SENSOR_OK && b == SENSOR_OK)           return SENSOR_OK;
    if (a == SENSOR_ERR_I2C && b == SENSOR_ERR_I2C) return SENSOR_ERR_I2C;
    return SENSOR_ERR_NOT_READY;
}

sensor_status_t sensor_read(sensor_data_t *data)
{
    float t, h, p, tb, alt;

    if (data == NULL) return SENSOR_ERR_PARAM;
    if (lock())       return SENSOR_ERR_OS;

    memset(data, 0, sizeof(*data));

    if (aht20_read_locked(&t, &h) == SENSOR_OK) {
        data->temperature = t;
        data->humidity    = h;
        data->valid      |= SENSOR_VALID_AHT20;
    }
    if (bmp280_read_locked(&p, &tb, &alt) == SENSOR_OK) {
        data->pressure = p;
        data->altitude = alt;
        data->valid   |= SENSOR_VALID_BMP280;
        if (!(data->valid & SENSOR_VALID_AHT20))
            data->temperature = tb;         /* AHT20 失效时用 BMP280 温度 */
    }

    unlock();
    return data->valid ? SENSOR_OK : SENSOR_ERR_DATA;
}

sensor_status_t aht20_read(float *temp_c, float *humi_rh)
{
    sensor_status_t r;
    if (!temp_c || !humi_rh) return SENSOR_ERR_PARAM;
    if (lock())               return SENSOR_ERR_OS;
    r = aht20_read_locked(temp_c, humi_rh);
    unlock();
    return r;
}

sensor_status_t bmp280_read(float *pressure_hpa, float *temp_c, float *altitude_m)
{
    sensor_status_t r;
    if (!pressure_hpa || !temp_c || !altitude_m) return SENSOR_ERR_PARAM;
    if (lock())                                   return SENSOR_ERR_OS;
    r = bmp280_read_locked(pressure_hpa, temp_c, altitude_m);
    unlock();
    return r;
}