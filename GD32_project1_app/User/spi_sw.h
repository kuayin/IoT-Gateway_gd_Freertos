#ifndef SPI_SW_H
#define SPI_SW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 标准版软件 SPI 主机驱动
 *
 * 设计边界：
 *   - 只管理 SCK / MOSI / MISO 三根信号线；
 *   - 不管理 CS（片选），由上层器件驱动通过 GPIO 控制；
 *   - 不包含任何具体器件的命令/时序，纯总线层。
 *
 * 事务模型（多从机共享 SPI 总线时的标准做法）：
 *
 *     spi_sw_acquire();            // 锁总线
 *     spi_sw_set_mode(...);        // 可选：持锁后切换（此时所有 CS 必须为高）
 *     spi_sw_set_speed(...);       // 可选
 *     DEV_CS_LOW();                // 上层器件驱动控制自己的 CS
 *     spi_sw_xfer_byte(cmd);
 *     spi_sw_xfer(NULL, buf, len);
 *     DEV_CS_HIGH();
 *     spi_sw_release();            // 解锁
 *
 * 约束：CS 为低电平期间必须持有总线锁，保证多阶段命令的原子性。
 * ============================================================ */

/* SPI 时钟模式（CPOL/CPHA 组合） */
typedef enum {
    SPI_SW_MODE_0 = 0,  /* CPOL=0, CPHA=0：SCK 空闲低，leading edge 采样 */
    SPI_SW_MODE_1,      /* CPOL=0, CPHA=1：SCK 空闲低，trailing edge 采样 */
    SPI_SW_MODE_2,      /* CPOL=1, CPHA=0：SCK 空闲高，leading edge 采样 */
    SPI_SW_MODE_3,      /* CPOL=1, CPHA=1：SCK 空闲高，trailing edge 采样 */
} spi_sw_mode_t;

typedef enum {
    SPI_OK = 0,
    SPI_ERR_PARAM,
    SPI_ERR_MUTEX,      /* 获取总线互斥量超时 / 总线正忙时尝试切换配置 */
} spi_status_t;

/*
 * 初始化：
 *   - 配置 SCK/MOSI 推挽输出、MISO 上拉输入；
 *   - 若 SCK/MISO 使用 PB3/PB4 等 JTAG 引脚，通过 SPI_SW_RELEASE_JTAG
 *     宏控制在初始化时释放 JTAG 功能（保留 SWD）；
 *   - 默认 Mode 0、1 MHz；
 *   - 创建总线互斥量（FreeRTOS）。
 * 必须在调度器启动前或任务上下文中调用一次。
 */
void spi_sw_init(void);

/*
 * 设置时钟模式。
 * 调用方有三种场景：
 *   1. 已持锁（acquire 之后）：直接切换，总线独占，安全；
 *   2. 未持锁且总线空闲：内部零等待 take mutex，切换后自动释放；
 *   3. 未持锁但总线正忙（其他任务在传输）：返回 SPI_ERR_MUTEX。
 * 调用方根据返回值决定「稍后重试」或「放弃切换」。
 */
spi_status_t spi_sw_set_mode(spi_sw_mode_t mode);

/*
 * 设置 SCK 频率（Hz）。
 * 内部按 SystemCoreClock 换算 DWT 半周期计数，并以 SPI_SW_MIN_HALF_CYCLES
 * 为下限兜底，避免把「函数固定开销」当成有效延时。
 * 超过软件实现能力的高频会被静默钳制（不返回错误），可用
 * spi_sw_get_max_hz() 查询实际上限。
 * 持锁约束同 spi_sw_set_mode。
 */
spi_status_t spi_sw_set_speed(uint32_t hz);

/*
 * 返回软件 SPI 能达到的最高频率（Hz）。
 * 由 SystemCoreClock / (2 × SPI_SW_MIN_HALF_CYCLES) 推算，换主频自动跟随。
 */
uint32_t spi_sw_get_max_hz(void);

/*
 * 总线互斥。
 * 任何一次完整 SPI 事务（CS 低 → 数据传输 → CS 高）都应包在 acquire/release 之间。
 * 多任务环境下，未持锁调用 xfer 属编程错误（Debug 构建会被 configASSERT 拦截）。
 */
spi_status_t spi_sw_acquire(void);
void         spi_sw_release(void);

/* 单字节全双工交换（必须在 acquire/release 之间调用） */
uint8_t spi_sw_xfer_byte(uint8_t tx);

/*
 * 多字节全双工交换（必须在 acquire/release 之间调用）。
 *   tx == NULL：自动发送 0xFF 填充（纯接收）
 *   rx == NULL：丢弃接收数据（纯发送）
 */
void spi_sw_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len);

/*
 * 便捷一体化接口：acquire → xfer → release。
 * 适合单次、不需要跨多次调用的简单传输。
 * 注意：CS 仍需调用方在调用前拉低、返回后拉高。
 * 若一次 CS 低电平期间需要多段 xfer，请改用 acquire/release 包裹整个区间。
 */
spi_status_t spi_sw_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* SPI_SW_H */