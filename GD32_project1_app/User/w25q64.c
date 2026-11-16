#include "w25q64.h"
#include "gd32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

/* ============================================================
 * 总线头文件选择（由 W25Q64_USE_HW_SPI 决定）
 * ============================================================ */
#if W25Q64_USE_HW_SPI
  #include "spi_hw.h"
#else
  #include "spi_sw.h"
#endif

/* ============================================================
 * 总线适配层
 *
 * 把 spi_sw_* 与 spi_hw_* 两套 API 统一成一套 W25Q64_BUS_* 内部接口：
 *   - 器件函数只写 W25Q64_BUS_XFER_BYTE / W25Q64_BUS_XFER，
 *     不关心底层是软件还是硬件；
 *   - 「成功」常量用 W25Q64_BUS_OK，比较时不受枚举类型名差异影响；
 *   - 硬件 SPI 的「最近一次错误」通过 W25Q64_BUS_LAST_STATUS 查询，
 *     软件 SPI 下这个宏返回 SPI_OK（无错误源）。
 * ============================================================ */

#if W25Q64_USE_HW_SPI
  #define W25Q64_BUS_ACQUIRE()      spi_hw_acquire()
  #define W25Q64_BUS_RELEASE()      spi_hw_release()
  #define W25Q64_BUS_XFER_BYTE(x)   spi_hw_xfer_byte(x)
  #define W25Q64_BUS_XFER(tx,rx,n)  spi_hw_xfer((tx), (rx), (n))
  #define W25Q64_BUS_OK             SPI_HW_OK
  #define W25Q64_BUS_INIT()         spi_hw_init()
  #define W25Q64_BUS_SET_MODE(m)    spi_hw_set_mode(m)
  #define W25Q64_BUS_SET_SPEED(hz)  spi_hw_set_speed(hz)
  #define W25Q64_BUS_MODE_0         SPI_HW_MODE_0
  #define W25Q64_BUS_LAST_STATUS()  spi_hw_get_last_status()
  #define W25Q64_BUS_LAST_OK        SPI_HW_OK
#else
  #define W25Q64_BUS_ACQUIRE()      spi_sw_acquire()
  #define W25Q64_BUS_RELEASE()      spi_sw_release()
  #define W25Q64_BUS_XFER_BYTE(x)   spi_sw_xfer_byte(x)
  #define W25Q64_BUS_XFER(tx,rx,n)  spi_sw_xfer((tx), (rx), (n))
  #define W25Q64_BUS_OK             SPI_OK
  #define W25Q64_BUS_INIT()         spi_sw_init()
  #define W25Q64_BUS_SET_MODE(m)    spi_sw_set_mode(m)
  #define W25Q64_BUS_SET_SPEED(hz)  spi_sw_set_speed(hz)
  #define W25Q64_BUS_MODE_0         SPI_SW_MODE_0
  #define W25Q64_BUS_LAST_STATUS()  (SPI_OK)
  #define W25Q64_BUS_LAST_OK        SPI_OK
#endif

/* ============================================================
 * 片选引脚（可被用户工程覆盖）
 * ============================================================ */
#ifndef W25Q64_CS_PORT
#define W25Q64_CS_PORT      GPIOA
#endif
#ifndef W25Q64_CS_PIN
#define W25Q64_CS_PIN       GPIO_PIN_4
#endif
#ifndef W25Q64_CS_RCU
#define W25Q64_CS_RCU       RCU_GPIOA
#endif

#define W25Q64_CS_LOW()     gpio_bit_reset(W25Q64_CS_PORT, W25Q64_CS_PIN)
#define W25Q64_CS_HIGH()    gpio_bit_set(W25Q64_CS_PORT, W25Q64_CS_PIN)

/* ============================================================
 * 命令字（内部使用，不暴露到 .h）
 * ============================================================ */
#define W25Q64_CMD_WRITE_ENABLE     0x06
#define W25Q64_CMD_READ_STATUS1     0x05
#define W25Q64_CMD_READ_DATA        0x03
#define W25Q64_CMD_PAGE_PROGRAM     0x02
#define W25Q64_CMD_SECTOR_ERASE     0x20
#define W25Q64_CMD_JEDEC_ID         0x9F
#define W25Q64_CMD_RELEASE_PD       0xAB

/* ============================================================
 * 事务封装
 *
 * W25Q64 的每条命令都以「CS 低 → 命令/地址/数据 → CS 高」为一个事务。
 * 事务期间必须持有总线锁，否则多任务环境下会被其他从机驱动打断。
 *
 * w25q64_begin()：acquire 总线 + CS 低
 * w25q64_end()  ：CS 高 + 检查硬件错误 + release 总线
 *
 * 硬件 SPI 的错误捕获分两次：
 *   1. release 之前先记下 xfer 期间的错误（s_last_status）；
 *   2. release 之后再查一次，捕获 release 内部等待 TRANS 超时
 *      产生的错误（此时 s_last_status 被更新）；
 *   3. 取两次中「非 OK」的那个作为最终结果。
 *
 * 软件 SPI 没有硬件错误源，直接 release 后返回 OK。
 * ============================================================ */

static w25q64_status_t w25q64_begin(void)
{
    if (W25Q64_BUS_ACQUIRE() != W25Q64_BUS_OK) {
        return W25Q64_ERR_SPI;
    }
    W25Q64_CS_LOW();
    return W25Q64_OK;
}

static w25q64_status_t w25q64_end(void)
{
    W25Q64_CS_HIGH();

#if W25Q64_USE_HW_SPI
    {
        spi_hw_status_t st = spi_hw_get_last_status();
        W25Q64_BUS_RELEASE();
        /* release 内部若 TRANS 超时会再次更新 s_last_status */
        if (st == W25Q64_BUS_LAST_OK) {
            st = spi_hw_get_last_status();
        }
        return (st == W25Q64_BUS_LAST_OK) ? W25Q64_OK : W25Q64_ERR_SPI;
    }
#else
    W25Q64_BUS_RELEASE();
    return W25Q64_OK;
#endif
}

/* ============================================================
 * 内部辅助：发送 24 位地址（MSB 先行）
 * ============================================================ */
static void w25q64_send_addr(uint32_t addr)
{
    W25Q64_BUS_XFER_BYTE((uint8_t)(addr >> 16));
    W25Q64_BUS_XFER_BYTE((uint8_t)(addr >> 8));
    W25Q64_BUS_XFER_BYTE((uint8_t)(addr));
}

/* ============================================================
 * 总线初始化（分层职责：只做总线层）
 * ============================================================ */
w25q64_status_t w25q64_bus_init(void)
{
    W25Q64_BUS_INIT();

    if (W25Q64_BUS_SET_MODE(W25Q64_BUS_MODE_0) != W25Q64_BUS_OK) {
        return W25Q64_ERR_SPI;
    }

    if (W25Q64_BUS_SET_SPEED(W25Q64_DEFAULT_SPEED_HZ) != W25Q64_BUS_OK) {
        return W25Q64_ERR_SPI;
    }

    return W25Q64_OK;
}

/* ============================================================
 * 器件初始化
 *
 * 前置条件：w25q64_bus_init() 已成功调用。
 *
 * 流程：
 *   1. 配置 CS 引脚（推挽输出，初始高）
 *   2. 发送 0xAB 释放掉电模式
 *   3. 等待 tRES1（约 3 μs，这里给 1 ms 余量）
 *   4. 读 JEDEC ID 校验型号
 * ============================================================ */
w25q64_status_t w25q64_init(void)
{
    uint8_t         id[3];
    w25q64_status_t st;

    /* CS 引脚：推挽输出，初始高（未选中） */
    rcu_periph_clock_enable(W25Q64_CS_RCU);
    gpio_init(W25Q64_CS_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, W25Q64_CS_PIN);
    W25Q64_CS_HIGH();

    /* 释放掉电模式 */
    st = w25q64_begin();
    if (st != W25Q64_OK) return st;
    W25Q64_BUS_XFER_BYTE(W25Q64_CMD_RELEASE_PD);
    st = w25q64_end();
    if (st != W25Q64_OK) return st;

    vTaskDelay(pdMS_TO_TICKS(1));   /* 保守等待，远大于 tRES1（3 μs） */

    /* 校验 JEDEC ID */
    st = w25q64_read_jedec_id(id);
    if (st != W25Q64_OK) return st;

    if (id[0] != W25Q64_JEDEC_MFR ||
        id[1] != W25Q64_JEDEC_TYPE ||
        id[2] != W25Q64_JEDEC_CAP) {
        return W25Q64_ERR_VERIFY;
    }

    return W25Q64_OK;
}

/* ============================================================
 * 读 JEDEC ID
 * ============================================================ */
w25q64_status_t w25q64_read_jedec_id(uint8_t id[3])
{
    w25q64_status_t st;

    if (id == NULL) return W25Q64_ERR_PARAM;

    st = w25q64_begin();
    if (st != W25Q64_OK) return st;

    W25Q64_BUS_XFER_BYTE(W25Q64_CMD_JEDEC_ID);
    W25Q64_BUS_XFER(NULL, id, 3);

    return w25q64_end();
}

/* ============================================================
 * 读状态寄存器 1
 *
 * 由于返回类型是 uint8_t（与 spi_xfer_byte 的语义一致），
 * 内部失败时返回 0xFF（bit0=1 会被上层当作 BUSY）。
 * 需要更精细错误语义时用 w25q64_wait_ready。
 * ============================================================ */
uint8_t w25q64_read_status1(void)
{
    uint8_t sr = 0xFF;

    if (w25q64_begin() != W25Q64_OK) return 0xFF;

    W25Q64_BUS_XFER_BYTE(W25Q64_CMD_READ_STATUS1);
    W25Q64_BUS_XFER(NULL, &sr, 1);

    if (w25q64_end() != W25Q64_OK) return 0xFF;
    return sr;
}

/* ============================================================
 * 等待 BUSY 清零
 *
 * 每次查询是一次独立事务（CS 低 → 读 SR1 → CS 高），
 * 每次之间 vTaskDelay(1 tick)，避免高频轮询占用 CPU。
 *
 * 4 KB 扇区擦除典型 45 ms，64 KB 块擦除典型 150 ms。
 * ============================================================ */
w25q64_status_t w25q64_wait_ready(uint32_t timeout_ms)
{
    uint32_t elapsed = 0;
    uint8_t  sr;
    w25q64_status_t st;

    for (;;) {
        st = w25q64_begin();
        if (st != W25Q64_OK) return st;

        W25Q64_BUS_XFER_BYTE(W25Q64_CMD_READ_STATUS1);
        W25Q64_BUS_XFER(NULL, &sr, 1);

        st = w25q64_end();
        if (st != W25Q64_OK) return st;

        if ((sr & 0x01U) == 0U) return W25Q64_OK;   /* BUSY 清零 */

        if (elapsed >= timeout_ms) return W25Q64_ERR_TIMEOUT;

        vTaskDelay(pdMS_TO_TICKS(1));
        elapsed += 1U;
    }
}

/* ============================================================
 * 写使能
 * ============================================================ */
w25q64_status_t w25q64_write_enable(void)
{
    w25q64_status_t st;

    st = w25q64_begin();
    if (st != W25Q64_OK) return st;

    W25Q64_BUS_XFER_BYTE(W25Q64_CMD_WRITE_ENABLE);

    return w25q64_end();
}

/* ============================================================
 * 扇区擦除（4 KB）
 *
 * 流程：写使能 → 发送擦除命令 + 地址 → 拉高 CS → 等 BUSY 清零。
 * 地址必须 4 KB 对齐，否则硬件自动向下对齐（仍能擦除，但语义不对）。
 * ============================================================ */
w25q64_status_t w25q64_sector_erase(uint32_t addr)
{
    w25q64_status_t st;

    if (addr >= W25Q64_CAPACITY) return W25Q64_ERR_PARAM;
    if ((addr % W25Q64_SECTOR_SIZE) != 0U) return W25Q64_ERR_PARAM;

    st = w25q64_write_enable();
    if (st != W25Q64_OK) return st;

    st = w25q64_begin();
    if (st != W25Q64_OK) return st;

    W25Q64_BUS_XFER_BYTE(W25Q64_CMD_SECTOR_ERASE);
    w25q64_send_addr(addr);

    st = w25q64_end();
    if (st != W25Q64_OK) return st;

    /* 扇区擦除典型 45 ms，超时给 500 ms 余量 */
    return w25q64_wait_ready(500U);
}

/* ============================================================
 * 页编程（不跨页，len ≤ 256）
 *
 * 注意：调用方负责「先擦除再写」。Flash 只能把 1 写为 0，
 * 未擦除的区域直接写会导致数据错误。
 * ============================================================ */
w25q64_status_t w25q64_page_program(uint32_t addr, const uint8_t *data, uint16_t len)
{
    uint16_t        page_off;
    w25q64_status_t st;

    if (data == NULL || len == 0U || len > W25Q64_PAGE_SIZE) return W25Q64_ERR_PARAM;
    if (addr >= W25Q64_CAPACITY) return W25Q64_ERR_PARAM;

    /* 检查是否跨页：页内偏移 + 长度不能越过 256 */
    page_off = (uint16_t)(addr % W25Q64_PAGE_SIZE);
    if ((uint32_t)page_off + len > W25Q64_PAGE_SIZE) return W25Q64_ERR_PARAM;

    st = w25q64_write_enable();
    if (st != W25Q64_OK) return st;

    st = w25q64_begin();
    if (st != W25Q64_OK) return st;

    W25Q64_BUS_XFER_BYTE(W25Q64_CMD_PAGE_PROGRAM);
    w25q64_send_addr(addr);
    W25Q64_BUS_XFER(data, NULL, len);

    st = w25q64_end();
    if (st != W25Q64_OK) return st;

    /* 页编程典型 0.7 ms（256 字节），超时给 10 ms 余量 */
    return w25q64_wait_ready(10U);
}

/* ============================================================
 * 读数据（任意长度）
 *
 * 一次 CS 拉低内连续读完（W25Q64 的 0x03 命令地址自动递增）。
 * spi_sw_xfer / spi_hw_xfer 的 len 参数是 uint16_t（上限 65535），
 * 这里按 65535 分段循环，避免大块读取时被静默截断。
 * ============================================================ */
w25q64_status_t w25q64_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint32_t chunk;
    w25q64_status_t st;

    if (buf == NULL || len == 0U) return W25Q64_ERR_PARAM;
    if (addr >= W25Q64_CAPACITY || addr + len > W25Q64_CAPACITY) return W25Q64_ERR_PARAM;

    st = w25q64_begin();
    if (st != W25Q64_OK) return st;

    W25Q64_BUS_XFER_BYTE(W25Q64_CMD_READ_DATA);
    w25q64_send_addr(addr);

    while (len > 0U) {
        chunk = (len > 0xFFFFU) ? 0xFFFFU : len;
        W25Q64_BUS_XFER(NULL, buf, (uint16_t)chunk);
        buf += chunk;
        len -= chunk;
    }

    return w25q64_end();
}