#include "spi_sw.h"
#include "gd32f10x.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* ======================== 引脚配置 ======================== */

#define SPI_GPIO_PORT       GPIOA
#define SPI_SCK_PIN         GPIO_PIN_5
#define SPI_MISO_PIN        GPIO_PIN_6
#define SPI_MOSI_PIN        GPIO_PIN_7
#define SPI_GPIO_RCU        RCU_GPIOA

/*
 * 若 SCK/MISO 使用 PB3/PB4（GD32F103 上默认是 JTAG 引脚 JTDO/NJTRST），
 * 需在初始化时释放 JTAG、保留 SWD。设为 1 时 spi_sw_init() 会执行：
 *     rcu_periph_clock_enable(RCU_AF);
 *     gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_Msk, ENABLE);
 * 默认 PA5/6/7 不涉及 JTAG，设为 0。
 */
#define SPI_SW_RELEASE_JTAG 0

/* ======================== 时序配置 ======================== */

/* 默认 SCK 频率 */
#define SPI_SW_DEFAULT_HZ   1000000UL   /* 1 MHz */

/*
 * DWT 延时函数的固定开销（CPU 周期）。
 *
 * spi_delay_half() 从入口到第一次 while 判断，Cortex-M3 上至少执行：
 *   LDR(CYCCNT) + MOV + LDR(CYCCNT) + SUB + CMP + BCC
 * 实测约 8~12 周期，取 10 作为保守下限。
 *
 * 若换算出的半周期计数小于此值，实际延时由函数开销兜底，
 * 此时真实半周期 ≈ 10 周期（108 MHz 下约 92 ns，对应约 5.4 MHz）。
 * 显式声明此下限，使「最大可用频率」可被反推（见 spi_sw_get_max_hz），
 * 而不是靠 if (cycles < 1) 掩盖问题。
 */
#define SPI_SW_MIN_HALF_CYCLES  10U

/* 互斥锁等待时间 */
#define SPI_MUTEX_WAIT_TICKS  pdMS_TO_TICKS(20)

/* ======================== 引脚操作 ======================== */

/*
 * SPI 为推挽总线（不同于 I2C 的开漏 + 上拉）：
 *   SCK/MOSI：推挽输出
 *   MISO：上拉输入（从机 DO 推挽输出，主机上拉防止 CS 释放时浮空）
 *
 * MISO 直读 GPIO_ISTAT 寄存器：
 *   gpio_input_bit_get() 走库函数约 10~15 条指令，5 MHz 下会吃掉半个周期。
 *   直接读整个端口的输入状态再按位与，单条 LDR + AND，开销降到 3~4 周期。
 */
#define SCK_HIGH()   gpio_bit_set(SPI_GPIO_PORT, SPI_SCK_PIN)
#define SCK_LOW()    gpio_bit_reset(SPI_GPIO_PORT, SPI_SCK_PIN)
#define MOSI_HIGH()  gpio_bit_set(SPI_GPIO_PORT, SPI_MOSI_PIN)
#define MOSI_LOW()   gpio_bit_reset(SPI_GPIO_PORT, SPI_MOSI_PIN)

#define MISO_READ()  ((GPIO_ISTAT(SPI_GPIO_PORT) & SPI_MISO_PIN) ? SET : RESET)

/* ======================== 内部状态 ======================== */

static SemaphoreHandle_t s_spi_mutex = NULL;
static uint8_t           s_cpol = 0;              /* 0: 空闲低；1: 空闲高 */
static uint8_t           s_cpha = 0;              /* 0 / 1 */
static uint32_t          s_half_delay_cycles = SPI_SW_MIN_HALF_CYCLES;
static uint8_t           s_acquired = 0;

/* ======================== DWT 精确延时 ======================== */

/*
 * 幂等初始化：如果 DWT 已经被其它模块（例如 spi_hw.c）启用，则不再重置
 * CYCCNT，避免干扰正在进行的延时/超时计时。
 */
static void dwt_init(void)
{
    if (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) {
        return;   /* 已经启用，保持现状 */
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void spi_delay_half(void)
{
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < s_half_delay_cycles) { }
}

/* ======================== SCK 电平控制 ======================== */

static inline void sck_idle(void)
{
    if (s_cpol) SCK_HIGH();
    else        SCK_LOW();
}

static inline void sck_active(void)
{
    if (s_cpol) SCK_LOW();
    else        SCK_HIGH();
}

/* ======================== 频率换算 ======================== */

/*
 * 将目标频率换算为半周期的 DWT 周期数。
 * 半周期 = 1/(2*hz) 秒 = SystemCoreClock/(2*hz) 个 CPU 周期。
 * 低于函数固定开销时，实际延时由开销兜底；显式钳制到
 * SPI_SW_MIN_HALF_CYCLES，避免退化为无意义的 1 周期。
 */
static uint32_t half_delay_cycles_from_hz(uint32_t hz)
{
    uint64_t cycles;

    if (hz == 0U) hz = SPI_SW_DEFAULT_HZ;

    cycles = (uint64_t)SystemCoreClock / (2ULL * hz);
    if (cycles < (uint64_t)SPI_SW_MIN_HALF_CYCLES)
        cycles = (uint64_t)SPI_SW_MIN_HALF_CYCLES;

    return (uint32_t)cycles;
}

uint32_t spi_sw_get_max_hz(void)
{
    return SystemCoreClock / (2U * SPI_SW_MIN_HALF_CYCLES);
}

/* ======================== 初始化 ======================== */

void spi_sw_init(void)
{
    rcu_periph_clock_enable(SPI_GPIO_RCU);

#if SPI_SW_RELEASE_JTAG
    /* 释放 PB3/PB4 的 JTAG 功能，保留 SWD（PA13/PA14）用于调试 */
    rcu_periph_clock_enable(RCU_AF);
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
#endif

    /* SCK/MOSI 推挽输出；MISO 上拉输入 */
    gpio_init(SPI_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,
              SPI_SCK_PIN | SPI_MOSI_PIN);
    gpio_init(SPI_GPIO_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, SPI_MISO_PIN);

    dwt_init();

    s_cpol = 0;
    s_cpha = 0;
    s_half_delay_cycles = half_delay_cycles_from_hz(SPI_SW_DEFAULT_HZ);

    /* 空闲状态：SCK 在空闲电平，MOSI 低 */
    sck_idle();
    MOSI_LOW();

    if (s_spi_mutex == NULL) {
        s_spi_mutex = xSemaphoreCreateMutex();
        configASSERT(s_spi_mutex != NULL);
    }
}

/* ======================== 运行时配置 ======================== */

/*
 * 辅助：确保「改配置时总线不被其他任务使用」
 *
 * 三种调用场景的处理：
 *   1. 自己已持锁（s_acquired == 1）：直接返回 1，表示可以切换；
 *   2. 未持锁但总线空闲：零等待 take mutex 成功，返回 2（表示需要调用方
 *      在切换完成后 give 回去）；
 *   3. 未持锁且总线正忙：返回 0，表示不能切换。
 *
 * 返回值约定：
 *   1 = 已持锁，调用方无需归还
 *   2 = 本次 take 的锁，调用方负责 give
 *   0 = 无法获得总线，不能切换
 */
static uint8_t spi_sw_lock_for_config(void)
{
    if (s_acquired) return 1U;

    if (xSemaphoreTake(s_spi_mutex, 0) != pdTRUE) {
        return 0U;   /* 总线正忙，切换会打断正在进行的传输 */
    }
    return 2U;
}

static void spi_sw_unlock_after_config(uint8_t lock_state)
{
    if (lock_state == 2U) {
        xSemaphoreGive(s_spi_mutex);
    }
}

spi_status_t spi_sw_set_mode(spi_sw_mode_t mode)
{
    uint8_t lock_state;

    if ((uint32_t)mode > (uint32_t)SPI_SW_MODE_3) return SPI_ERR_PARAM;

    lock_state = spi_sw_lock_for_config();
    if (lock_state == 0U) return SPI_ERR_MUTEX;

    s_cpol = ((uint8_t)mode & 0x02U) ? 1U : 0U;
    s_cpha = ((uint8_t)mode & 0x01U) ? 1U : 0U;

    /* 切换到新模式的空闲电平；此时总线独占，切换 SCK 不会干扰任何从机 */
    sck_idle();

    spi_sw_unlock_after_config(lock_state);
    return SPI_OK;
}

spi_status_t spi_sw_set_speed(uint32_t hz)
{
    uint8_t lock_state;

    if (hz == 0U) return SPI_ERR_PARAM;

    lock_state = spi_sw_lock_for_config();
    if (lock_state == 0U) return SPI_ERR_MUTEX;

    s_half_delay_cycles = half_delay_cycles_from_hz(hz);

    spi_sw_unlock_after_config(lock_state);
    return SPI_OK;
}

/* ======================== 总线互斥 ======================== */

spi_status_t spi_sw_acquire(void)
{
    if (xSemaphoreTake(s_spi_mutex, SPI_MUTEX_WAIT_TICKS) != pdTRUE)
        return SPI_ERR_MUTEX;

    s_acquired = 1U;
    return SPI_OK;
}

void spi_sw_release(void)
{
    if (!s_acquired) return;

    /* 释放前让 SCK 回到空闲电平，避免影响其他从机 */
    sck_idle();

    s_acquired = 0U;
    xSemaphoreGive(s_spi_mutex);
}

/* ======================== 字节/多字节交换 ======================== */

/*
 * 通用 4 模式实现，每位 2 个半周期（50% 占空比）。
 *
 * CPHA=0：
 *   SCK 空闲电平期间设置数据（MOSI 变化）
 *   → leading edge：从机采样 MOSI，主机采样 MISO
 *   → 有效电平半周期：数据保持稳定
 *   → trailing edge：SCK 回到空闲
 *
 * CPHA=1：
 *   SCK 空闲电平期间设置数据（MOSI 变化）——这一步很重要，
 *     保证数据在 leading edge 之前就已建立稳定；
 *   → leading edge：SCK 翻转，数据自此刻起有效，从机在内部采样准备
 *   → 有效电平半周期：数据保持稳定
 *   → trailing edge：从机采样 MOSI，主机采样 MISO
 *   → 空闲半周期
 *
 * 与上一版的差异：
 *   上一版 CPHA=1 是「先产生 leading edge，再改 MOSI」，从机在边沿
 *   时刻看到的还是上一位旧值，对 CPHA=1 严格的器件可能采样错位。
 *   本版改为「先改 MOSI，再产生边沿」，与 CPHA=0 保持一致的数据建立
 *   时序，只是采样点从 leading edge 移到 trailing edge。
 */
uint8_t spi_sw_xfer_byte(uint8_t tx)
{
    uint8_t rx = 0U;
    uint8_t i;

    configASSERT(s_acquired);

    for (i = 0U; i < 8U; i++) {
        if (s_cpha == 0U) {
            /* ---------- CPHA=0 ---------- */
            if (tx & 0x80U) MOSI_HIGH();
            else            MOSI_LOW();
            tx <<= 1;

            spi_delay_half();           /* 数据建立 */

            sck_active();               /* leading edge */
            rx <<= 1;
            if (MISO_READ() == SET) rx |= 1U;   /* leading edge 采样 */

            spi_delay_half();           /* 有效电平半周期 */
            sck_idle();                 /* trailing edge */
        } else {
            /* ---------- CPHA=1 ---------- */
            /* 先在空闲电平下设置 MOSI，保证 leading edge 时数据已稳定 */
            if (tx & 0x80U) MOSI_HIGH();
            else            MOSI_LOW();
            tx <<= 1;

            sck_active();               /* leading edge：数据自此刻起有效 */
            spi_delay_half();           /* 有效电平半周期 */

            sck_idle();                 /* trailing edge：采样时刻 */
            rx <<= 1;
            if (MISO_READ() == SET) rx |= 1U;

            spi_delay_half();           /* 空闲电平半周期 */
        }
    }

    return rx;
}

void spi_sw_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    uint16_t i;

    configASSERT(s_acquired);

    for (i = 0U; i < len; i++) {
        uint8_t out = (tx != NULL) ? tx[i] : 0xFFU;
        uint8_t in  = spi_sw_xfer_byte(out);
        if (rx != NULL) rx[i] = in;
    }
}

/* ======================== 一体化接口 ======================== */

spi_status_t spi_sw_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    spi_status_t status;

    if (len == 0U || (tx == NULL && rx == NULL))
        return SPI_ERR_PARAM;

    status = spi_sw_acquire();
    if (status != SPI_OK) return status;

    spi_sw_xfer(tx, rx, len);

    spi_sw_release();
    return SPI_OK;
}