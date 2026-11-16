#include "OLED_I2C.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "codetab.h"        /* 需包含 F6x8 / F8X16 / F16x16 字模表 */

/* ==================== I2C 后端适配 ==================== */

#if USE_SW_I2C
#define OLED_I2C_WRITE(ctrl, buf, len)   sw_i2c_mem_write(OLED_I2C_ADDR_7BIT, (ctrl), (buf), (len))
#else
#define OLED_I2C_WRITE(ctrl, buf, len)   hw_i2c_mem_write(OLED_I2C_ADDR_7BIT, (ctrl), (buf), (len))
#endif

/* 等待 OLED 内部互斥锁的超时（并非 I2C 传输超时） */
#define OLED_LOCK_TIMEOUT   pdMS_TO_TICKS(200)

/* ==================== 内部状态 ==================== */

static SemaphoreHandle_t s_oled_mutex   = NULL;
static volatile i2c_status_t s_last_err = I2C_OK;
static uint8_t s_page_buf[OLED_WIDTH];      /* 批量数据发送缓冲区 */

static inline void oled_set_error(i2c_status_t st)
{
    if (st != I2C_OK) s_last_err = st;
}

static inline int oled_lock(void)
{
    if (s_oled_mutex == NULL) return 1;                 /* 尚未初始化，跳过保护 */
    return xSemaphoreTake(s_oled_mutex, OLED_LOCK_TIMEOUT) == pdTRUE;
}

static inline void oled_unlock(void)
{
    if (s_oled_mutex != NULL) (void)xSemaphoreGive(s_oled_mutex);
}

/* ==================== 毫秒延时（兼容调度器未启动） ==================== */

static void oled_delay_ms(uint32_t ms)
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        /* 调度器未运行：使用粗略忙等，仅用于上电延时 */
        volatile uint32_t i;
        for (i = 0; i < ms * 8000U; i++) { __NOP(); }
    }
}

/* ==================== 内部无锁辅助函数 ==================== */

static i2c_status_t oled_write_cmd_raw(uint8_t cmd)
{
    i2c_status_t st = OLED_I2C_WRITE(OLED_CTRL_CMD, &cmd, 1U);
    oled_set_error(st);
    return st;
}

static i2c_status_t oled_write_data_raw(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) return I2C_ERR_PARAM;
    i2c_status_t st = OLED_I2C_WRITE(OLED_CTRL_DATA, data, len);
    oled_set_error(st);
    return st;
}

static void oled_set_pos_raw(uint8_t x, uint8_t page)
{
    if (x >= OLED_WIDTH || page >= OLED_PAGES) return;

    /* 三命令合并成一次 I2C 事务（0x00 控制字节后所有字节均为命令） */
    uint8_t cmd[3];
    cmd[0] = (uint8_t)(0xB0U + page);
    cmd[1] = (uint8_t)(0x10U | ((x & 0xF0U) >> 4));
    cmd[2] = (uint8_t)(x & 0x0FU);           /* 原代码 (x&0x0F)|0x01 是 bug */
    oled_set_error(OLED_I2C_WRITE(OLED_CTRL_CMD, cmd, 3U));
}

static void oled_fill_raw(uint8_t fill_data)
{
    uint16_t i;
    for (i = 0; i < OLED_WIDTH; i++) s_page_buf[i] = fill_data;

    for (uint8_t page = 0; page < OLED_PAGES; page++) {
        uint8_t cmd[3];
        cmd[0] = (uint8_t)(0xB0U + page);
        cmd[1] = 0x10U;
        cmd[2] = 0x00U;
        oled_set_error(OLED_I2C_WRITE(OLED_CTRL_CMD, cmd, 3U));
        oled_set_error(OLED_I2C_WRITE(OLED_CTRL_DATA, s_page_buf, OLED_WIDTH));
    }
}

/* ==================== 公共 API ==================== */

i2c_status_t OLED_Init(void)
{
    /* 上电延时：VDD 稳定 + 内部复位（数据手册要求 > 100ms） */
    oled_delay_ms(100);

    /* 创建本模块保护锁（仅创建一次） */
    if (s_oled_mutex == NULL) {
        s_oled_mutex = xSemaphoreCreateMutex();
        configASSERT(s_oled_mutex != NULL);
    }

    /* 一次事务下发全部初始化命令 */
    static const uint8_t init_seq[] = {
        0xAE,               /* display off                    */
        0x20, 0x10,         /* 内存寻址模式：页模式            */
        0xB0,               /* 页起始地址 = 0                 */
        0xC8,               /* COM 反向扫描                   */
        0x00,               /* 低列地址                       */
        0x10,               /* 高列地址                       */
        0x40,               /* 起始行 = 0                     */
        0x81, 0xFF,         /* 对比度最大                     */
        0xA1,               /* 段重映射                       */
        0xA6,               /* 正常显示                       */
        0xA8, 0x3F,         /* MUX = 64                       */
        0xA4,               /* 输出跟随 RAM                   */
        0xD3, 0x00,         /* 显示偏移 = 0                   */
        0xD5, 0xF0,         /* 时钟分频                       */
        0xD9, 0x22,         /* 预充电周期                     */
        0xDA, 0x12,         /* COM 引脚配置                   */
        0xDB, 0x20,         /* VCOMH                          */
        0x8D, 0x14,         /* 电荷泵使能                     */
        0xAF,               /* 显示开启                       */
    };

    i2c_status_t st = OLED_I2C_WRITE(OLED_CTRL_CMD,
                                     init_seq,
                                     (uint16_t)sizeof(init_seq));
    if (st != I2C_OK) {
        oled_set_error(st);
        return st;
    }

    /* 清屏 */
    if (oled_lock()) {
        oled_fill_raw(0x00U);
        oled_unlock();
    }

    return I2C_OK;
}

void OLED_WriteCmd(uint8_t cmd)
{
    if (!oled_lock()) return;
    (void)oled_write_cmd_raw(cmd);
    oled_unlock();
}

i2c_status_t OLED_WriteData(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) return I2C_ERR_PARAM;
    if (!oled_lock()) return I2C_ERR_MUTEX;

    i2c_status_t st = oled_write_data_raw(data, len);
    oled_unlock();
    return st;
}

void OLED_SetPos(uint8_t x, uint8_t page)
{
    if (!oled_lock()) return;
    oled_set_pos_raw(x, page);
    oled_unlock();
}

void OLED_Fill(uint8_t fill_data)
{
    if (!oled_lock()) return;
    oled_fill_raw(fill_data);
    oled_unlock();
}

void OLED_CLS(void)
{
    if (!oled_lock()) return;
    oled_fill_raw(0x00U);
    oled_unlock();
}

void OLED_ON(void)
{
    if (!oled_lock()) return;
    (void)oled_write_cmd_raw(0x8D);   /* 电荷泵设置 */
    (void)oled_write_cmd_raw(0x14);   /* 使能电荷泵 */
    (void)oled_write_cmd_raw(0xAF);   /* 显示开 */
    oled_unlock();
}

void OLED_OFF(void)
{
    if (!oled_lock()) return;
    (void)oled_write_cmd_raw(0x8D);   /* 电荷泵设置 */
    (void)oled_write_cmd_raw(0x10);   /* 关闭电荷泵 */
    (void)oled_write_cmd_raw(0xAE);   /* 显示关 */
    oled_unlock();
}

void OLED_ShowStr(uint8_t x, uint8_t page,
                  const char *str, uint8_t text_size)
{
    if (str == NULL) return;
    if (!oled_lock()) return;

    if (text_size == OLED_FONT_6X8) {
        while (*str != '\0') {
            uint8_t ch = (uint8_t)(*str);
            if (ch < 32U || ch > 126U) { str++; continue; }   /* 非可打印字符 */
            uint8_t idx = ch - 32U;

            if (x + 6U > OLED_WIDTH) {           /* 换行 */
                x = 0U;
                page++;
                if (page >= OLED_PAGES) break;
            }
            oled_set_pos_raw(x, page);
            (void)oled_write_data_raw(F6x8[idx], 6U);
            x += 6U;
            str++;
        }
    } else if (text_size == OLED_FONT_8X16) {
        while (*str != '\0') {
            uint8_t ch = (uint8_t)(*str);
            if (ch < 32U || ch > 126U) { str++; continue; }
            uint8_t idx = ch - 32U;

            if (x + 8U > OLED_WIDTH) {           /* 换行（两页） */
                x = 0U;
                page += 2U;
                if (page + 1U >= OLED_PAGES) break;
            }
            oled_set_pos_raw(x, page);
            (void)oled_write_data_raw(&F8X16[idx * 16U], 8U);

            oled_set_pos_raw(x, (uint8_t)(page + 1U));
            (void)oled_write_data_raw(&F8X16[idx * 16U + 8U], 8U);

            x += 8U;
            str++;
        }
    }

    oled_unlock();
}

void OLED_ShowCN(uint8_t x, uint8_t page, uint8_t index)
{
    if (x + 16U > OLED_WIDTH || page + 1U >= OLED_PAGES) return;
    if (!oled_lock()) return;

    uint16_t base = (uint16_t)index * 32U;
    oled_set_pos_raw(x, page);
    (void)oled_write_data_raw(&F16x16[base], 16U);
    oled_set_pos_raw(x, (uint8_t)(page + 1U));
    (void)oled_write_data_raw(&F16x16[base + 16U], 16U);

    oled_unlock();
}

void OLED_DrawBMP(uint8_t x0, uint8_t page0,
                  uint8_t x1, uint8_t page1,
                  const uint8_t *bmp)
{
    if (bmp == NULL) return;
    if (x1 > OLED_WIDTH)  x1 = OLED_WIDTH;
    if (page1 > OLED_PAGES) page1 = OLED_PAGES;
    if (x0 >= x1 || page0 >= page1) return;
    if (!oled_lock()) return;

    uint16_t width = (uint16_t)(x1 - x0);
    uint32_t offset = 0U;

    for (uint8_t page = page0; page < page1; page++) {
        oled_set_pos_raw(x0, page);
        (void)oled_write_data_raw(&bmp[offset], width);
        offset += width;
    }

    oled_unlock();
}

i2c_status_t OLED_GetLastError(void)
{
    i2c_status_t e = s_last_err;
    s_last_err = I2C_OK;
    return e;
}