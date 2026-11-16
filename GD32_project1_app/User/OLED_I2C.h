#ifndef OLED_I2C_H
#define OLED_I2C_H

#include "gd32f10x.h"

/*
后端选择见 i2c_backend.h
 */
#include "i2c_backend.h"

#ifdef __cplusplus
extern "C" {
#endif

/* SSD1306 7-bit 地址（0x78 >> 1；部分模块为 0x7A>>1 = 0x3D） */
#define OLED_I2C_ADDR_7BIT      0x3C

/* SSD1306 控制字节 */
#define OLED_CTRL_CMD           0x00U   /* Co=0, D/C#=0：后续字节为命令 */
#define OLED_CTRL_DATA          0x40U   /* Co=0, D/C#=1：后续字节为数据 */

/* 面板参数 */
#define OLED_WIDTH              128U
#define OLED_HEIGHT             64U
#define OLED_PAGES              (OLED_HEIGHT / 8U)

/* 文本字号 */
#define OLED_FONT_6X8           1U
#define OLED_FONT_8X16          2U

/*
 * 初始化 SSD1306。
 * 前提：已先调用 sw_i2c_init() 或 hw_i2c_init()。
 * 可在调度器启动前调用，也可在任务中调用。
 */
i2c_status_t OLED_Init(void);

/* 底层命令/数据接口（供自定义绘制使用） */
void          OLED_WriteCmd (uint8_t cmd);
i2c_status_t  OLED_WriteData(const uint8_t *data, uint16_t len);

/* 设置显存写入光标：x ∈ [0,127]，page ∈ [0,7] */
void OLED_SetPos(uint8_t x, uint8_t page);

/* 显示控制 */
void OLED_Fill (uint8_t fill_data);
void OLED_CLS  (void);
void OLED_ON   (void);
void OLED_OFF  (void);

/* 文本绘制（ASCII，6x8 或 8x16） */
void OLED_ShowStr(uint8_t x, uint8_t page,
                  const char *str, uint8_t text_size);

/* 汉字绘制（16x16，index 为 codetab.h 中 F16x16 的索引） */
void OLED_ShowCN (uint8_t x, uint8_t page, uint8_t index);

/*
 * 位图绘制
 *   x0..x1-1  为像素列（x1 ≤ 128）
 *   page0..page1-1 为页号（page1 ≤ 8）
 *   bmp 数据按页-优先顺序排列（与取模软件一致）
 */
void OLED_DrawBMP(uint8_t x0, uint8_t page0,
                  uint8_t x1, uint8_t page1,
                  const uint8_t *bmp);

/* 获取最近一次 I2C 失败状态，并清空记录 */
i2c_status_t OLED_GetLastError(void);

#ifdef __cplusplus
}
#endif

#endif /* OLED_I2C_H */