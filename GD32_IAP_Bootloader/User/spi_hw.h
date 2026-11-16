#ifndef SPI_HW_H
#define SPI_HW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 标准版硬件 SPI 主机驱动（与 spi_sw.h 接口语义对齐）
 *
 * 设计边界（与软件版完全一致）：
 *   - 只管理 SCK / MOSI / MISO 三根信号线；
 *   - 不管理 CS，由上层器件驱动通过 GPIO 控制；
 *   - NSS 配置为软件模式（SPI_NSS_SOFT），硬件 NSS 引脚不参与片选；
 *   - 不包含任何具体器件的命令/时序。
 *
 * 事务模型（多从机共享 SPI 总线时的标准做法）：
 *
 *     spi_hw_acquire();                    // 锁总线 + 清理历史错误
 *     spi_hw_set_mode(...);                // 可选：持锁时按需切换
 *     spi_hw_set_speed(...);               // 可选
 *     DEV_CS_LOW();                        // 上层器件驱动控制自己的 CS
 *     spi_hw_xfer_byte(cmd);               // 多阶段命令逐段发送
 *     spi_hw_xfer(NULL, buf, len);
 *     DEV_CS_HIGH();
 *     spi_hw_release();                    // 等 TRANS 清零 + 解锁
 *     if (spi_hw_get_last_status() != SPI_HW_OK) { 重试整个事务 }
 *
 * 关键约束：
 *   1. CS 为低电平期间必须持有总线锁，保证多阶段命令的原子性；
 *   2. 硬件 SPI 有超时和硬件错误源，单字节函数通过返回值只给出数据，
 *      错误通过 spi_hw_get_last_status() 在事务结束后查询；
 *   3. 一旦事务中途出错，xfer 会提前中止，上层应立即拉高 CS 并重试
 *      整个事务，而不是继续发送剩余字节。
 * ============================================================ */

/* SPI 时钟模式（CPOL/CPHA 组合） */
typedef enum {
    SPI_HW_MODE_0 = 0,  /* CPOL=0, CPHA=0：SCK 空闲低，leading edge 采样 */
    SPI_HW_MODE_1,      /* CPOL=0, CPHA=1：SCK 空闲低，trailing edge 采样 */
    SPI_HW_MODE_2,      /* CPOL=1, CPHA=0：SCK 空闲高，leading edge 采样 */
    SPI_HW_MODE_3,      /* CPOL=1, CPHA=1：SCK 空闲高，trailing edge 采样 */
} spi_hw_mode_t;

/*
 * 错误码。
 * 使用 SPI_HW_ 前缀，避免与 spi_sw.h 的 SPI_OK/SPI_ERR_* 同时编译时冲突。
 * 若项目只使用其中一种实现，可通过 typedef 或宏做别名统一。
 */
typedef enum {
    SPI_HW_OK = 0,
    SPI_HW_ERR_PARAM,    /* 参数非法（len=0、mode 越界等） */
    SPI_HW_ERR_MUTEX,    /* 获取总线互斥量超时 / 总线正忙时尝试切换配置 */
    SPI_HW_ERR_TIMEOUT,  /* 等待 TBE/RBNE/TRANS 超时 */
    SPI_HW_ERR_HW,       /* RXORERR / CONFERR 等硬件错误 */
} spi_hw_status_t;

/*
 * 初始化硬件 SPI 主机：
 *   - 选择外设（默认 SPI0，挂在 APB2）；
 *   - 配置 SCK/MOSI 为复用推挽、MISO 为上拉输入；
 *   - 默认 Mode 0、1 MHz、8 位、MSB 先行、全双工、软件 NSS；
 *   - 创建总线互斥量（FreeRTOS）。
 * 必须在调度器启动前或任务上下文中调用一次。
 */
void spi_hw_init(void);

/*
 * 设置时钟模式（CPOL/CPHA）。
 * 调用场景与 spi_sw_set_mode 一致：
 *   1. 已持锁：直接切换；
 *   2. 未持锁但总线空闲：零等待 take mutex 后切换并释放；
 *   3. 未持锁且总线正忙：返回 SPI_HW_ERR_MUTEX。
 * 不会改变当前分频（速率）。
 */
spi_hw_status_t spi_hw_set_mode(spi_hw_mode_t mode);

/*
 * 设置 SCK 频率（Hz）。
 * 内部按 APB 时钟选择「不大于目标值的最接近分频比」，因此实际速率 ≤ 请求值。
 * 这种保守策略保证不会超过从机器件的速率上限。
 * 持锁约束同 spi_hw_set_mode，不会改变当前模式。
 */
spi_hw_status_t spi_hw_set_speed(uint32_t hz);

/*
 * 返回当前 SPI 外设能达到的最高频率（Hz）。
 * 由 APB 时钟 / 2 决定：SPI0 @ 108 MHz → 54 MHz；SPI1/2 → 27 MHz。
 */
uint32_t spi_hw_get_max_hz(void);

/*
 * 总线互斥。
 * acquire 成功后：持有总线锁，同时清理上一次事务遗留的 RXORERR/CONFERR。
 * release 会先等待最后一帧完全移出（TRANS 清零），再释放锁；
 * 若 TRANS 超时（说明外设卡死），会复位外设并将 s_last_status 置为 TIMEOUT。
 */
spi_hw_status_t spi_hw_acquire(void);
void            spi_hw_release(void);

/*
 * 查询上一次事务（上一次 acquire 之后）的最终状态。
 * 语义：
 *   - acquire 时清零；
 *   - xfer_byte / xfer 期间出错时置为相应错误码；
 *   - release 时若 TRANS 超时，也会置为 TIMEOUT；
 *   - release 后仍可查询（直到下一次 acquire）。
 */
spi_hw_status_t spi_hw_get_last_status(void);

/* 单字节全双工交换（必须在 acquire/release 之间调用） */
uint8_t spi_hw_xfer_byte(uint8_t tx);

/*
 * 多字节全双工交换（必须在 acquire/release 之间调用）。
 *   tx == NULL：自动发送 0xFF 填充（纯接收）
 *   rx == NULL：丢弃接收数据（纯发送）
 * 若中途发生错误，会停止发送剩余字节并立即返回（剩余 rx 内容不确定），
 * 上层应检查 spi_hw_get_last_status() 判断事务是否成功。
 */
void spi_hw_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len);

/*
 * 便捷一体化接口：acquire → xfer → release。
 * 返回最终状态；适合单次、不需要跨多次调用的简单传输。
 * 注意：CS 仍需调用方在调用前拉低、返回后拉高。
 */
spi_hw_status_t spi_hw_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* SPI_HW_H */