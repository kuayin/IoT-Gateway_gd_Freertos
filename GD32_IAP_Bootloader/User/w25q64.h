#ifndef W25Q64_H
#define W25Q64_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 总线选择（编译期配置）
 *
 * 1 = 硬件 SPI（spi_hw）
 * 0 = 软件 SPI（spi_sw）
 *
 * 可在编译选项（-DW25Q64_USE_HW_SPI=0）里覆盖，
 * 也可在包含本头文件之前 #define W25Q64_USE_HW_SPI 0 覆盖。
 * ============================================================ */
#ifndef W25Q64_USE_HW_SPI
#define W25Q64_USE_HW_SPI   1
#endif

/* ============================================================
 * 默认 SCK 频率（Hz）
 *
 * 硬件 SPI：5 MHz 起步，可靠后可按 W25Q64 手册上调。
 * 软件 SPI：1 MHz 保守值，实际可达约 4~5 MHz。
 * 用户可在包含头文件前覆盖，或在调用 w25q64_bus_init() 之后
 * 自己调用总线层的 set_speed 改速率。
 * ============================================================ */
#if W25Q64_USE_HW_SPI
  #ifndef W25Q64_DEFAULT_SPEED_HZ
  #define W25Q64_DEFAULT_SPEED_HZ   5000000UL    /* 5 MHz */
  #endif
#else
  #ifndef W25Q64_DEFAULT_SPEED_HZ
  #define W25Q64_DEFAULT_SPEED_HZ   1000000UL    /* 1 MHz */
  #endif
#endif

/* ============================================================
 * 容量与结构常量
 * ============================================================ */
#define W25Q64_PAGE_SIZE        256U        /* 页编程最小单位 */
#define W25Q64_SECTOR_SIZE      4096U       /* 扇区擦除最小单位 */
#define W25Q64_BLOCK_64K_SIZE   65536U
#define W25Q64_CAPACITY         (8UL * 1024UL * 1024UL)

/* ============================================================
 * JEDEC ID 期望值（用于 w25q64_init 校验）
 * ============================================================ */
#define W25Q64_JEDEC_MFR        0xEFU       /* Winbond */
#define W25Q64_JEDEC_TYPE       0x40U       /* SPI Flash */
#define W25Q64_JEDEC_CAP        0x17U       /* 64 Mbit */

/* ============================================================
 * 返回值
 * ============================================================ */
typedef enum {
    W25Q64_OK = 0,
    W25Q64_ERR_PARAM,     /* 参数非法（addr 越界、len=0 等） */
    W25Q64_ERR_TIMEOUT,   /* 等待 BUSY 清零超时 */
    W25Q64_ERR_SPI,       /* 总线层失败（acquire/传输/硬件错误） */
    W25Q64_ERR_VERIFY,    /* JEDEC ID 与期望值不符 */
} w25q64_status_t;

/* ============================================================
 * 初始化
 *
 * 分两步，符合工业标准「总线与器件职责分离」的原则：
 *
 *   1. w25q64_bus_init()   —— 初始化 SPI 总线（软/硬件由 W25Q64_USE_HW_SPI 决定）
 *                             内部调用 spi_sw_init/spi_hw_init，
 *                             并设置 Mode 0 与 W25Q64_DEFAULT_SPEED_HZ
 *   2. w25q64_init()       —— 器件初始化：CS 引脚、释放掉电模式、校验 JEDEC ID
 *
 * 两步都必须在 FreeRTOS 调度器启动之后、任务上下文中调用
 * （内部使用 vTaskDelay 与互斥量）。
 *
 * 典型调用序列：
 *
 *     if (w25q64_bus_init() != W25Q64_OK) { 处理总线错误 }
 *     if (w25q64_init()     != W25Q64_OK) { 处理器件错误 }
 * ============================================================ */

w25q64_status_t w25q64_bus_init(void);
w25q64_status_t w25q64_init(void);

/* ============================================================
 * 器件操作
 * ============================================================ */

/* 读 JEDEC ID（3 字节：EF 40 17） */
w25q64_status_t w25q64_read_jedec_id(uint8_t id[3]);

/* 读状态寄存器 1（bit0 = BUSY）。
 * 内部查询失败时返回 0xFF（bit0=1 会被当作 BUSY），
 * 调用方需要更高语义时用 w25q64_wait_ready。 */
uint8_t w25q64_read_status1(void);

/* 等待 BUSY 清零。每次查询 1 ms 间隔，超时返回 W25Q64_ERR_TIMEOUT */
w25q64_status_t w25q64_wait_ready(uint32_t timeout_ms);

/* 写使能（擦除/编程前必须调用） */
w25q64_status_t w25q64_write_enable(void);

/* 扇区擦除（4 KB 对齐），擦除后全 0xFF */
w25q64_status_t w25q64_sector_erase(uint32_t addr);

/* 页编程（256 字节以内，不跨页）。返回后芯片可能仍在写，需 wait_ready */
w25q64_status_t w25q64_page_program(uint32_t addr, const uint8_t *data, uint16_t len);

/* 读数据（任意长度，可跨页/跨扇区；>64KB 内部分段循环） */
w25q64_status_t w25q64_read(uint32_t addr, uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* W25Q64_H */