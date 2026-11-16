#include "spi_hw.h"
#include "gd32f10x.h"

/* 裸机与 RTOS 兼容开关 */
#ifndef USE_FREERTOS
#define USE_FREERTOS 0
#endif

#if USE_FREERTOS
#include "FreeRTOS.h"
#include "semphr.h"
#else
/* 裸机模式下：伪造信号量类型并屏蔽所有操作 */
typedef void* SemaphoreHandle_t; 

#define xSemaphoreTake(mutex, ticks) pdTRUE
#define xSemaphoreGive(mutex)
#define xSemaphoreCreateMutex() (void*)1
#define pdTRUE 1
#define pdMS_TO_TICKS(x) 0
#define configASSERT(x)
#endif
/* ============================================================
 * 外设与引脚选择
 *
 * 默认使用 SPI0（挂在 APB2，108 MHz 下最高 54 MHz）。
 *
 * 若改用 SPI1 / SPI2（挂在 APB1，54 MHz 下最高 27 MHz），
 * 必须同时修改以下 8 个宏：
 *   SPI_HW_PERIPH / SPI_HW_RCU / SPI_HW_ON_APB2
 *   SPI_HW_GPIO_PORT / SPI_HW_SCK_PIN / SPI_HW_MISO_PIN / SPI_HW_MOSI_PIN
 *   SPI_HW_GPIO_RCU
 *
 * 为什么需要 SPI_HW_ON_APB2 这个额外的布尔宏？
 *   因为 GD32 头文件里 SPI0 是寄存器地址宏，展开后含 (uint32_t) 强转和
 *   + 运算，C 预处理器无法在 #if 里对它求值。
 *   所以不能写 #if (SPI_HW_PERIPH == SPI0)，只能用一个独立的纯数字宏
 *   来指示该外设挂在哪个总线上。
 *
 * 切换外设对照表：
 *                SPI0(默认)    SPI1(PB13/14/15)   SPI2(PB3/4/5,需remap)
 *   PERIPH       SPI0          SPI1               SPI2
 *   RCU          RCU_SPI0      RCU_SPI1           RCU_SPI2
 *   ON_APB2      1             0                  0
 *   GPIO_PORT    GPIOA         GPIOB              GPIOB
 *   SCK_PIN      PIN_5         PIN_13             PIN_3
 *   MISO_PIN     PIN_6         PIN_14             PIN_4
 *   MOSI_PIN     PIN_7         PIN_15             PIN_5
 *   GPIO_RCU     RCU_GPIOA     RCU_GPIOB          RCU_GPIOB
 * ============================================================ */

#define SPI_HW_PERIPH       SPI0
#define SPI_HW_RCU          RCU_SPI0
#define SPI_HW_ON_APB2      1        /* SPI0 挂 APB2；SPI1/SPI2 改为 0 */

#define SPI_HW_GPIO_PORT    GPIOA
#define SPI_HW_SCK_PIN      GPIO_PIN_5
#define SPI_HW_MISO_PIN     GPIO_PIN_6
#define SPI_HW_MOSI_PIN     GPIO_PIN_7
#define SPI_HW_GPIO_RCU     RCU_GPIOA

/*
 * 若使用 SPI0 的重映射引脚（PB3/PB4/PB5），需同时释放 JTAG 并开启 SPI0 remap。
 * 默认使用 PA5/6/7，不涉及 JTAG，设为 0。
 */
#define SPI_HW_SPI0_REMAP   0

/* ============================================================
 * 速率与超时配置
 * ============================================================ */

/* 默认 SCK 频率 */
#define SPI_HW_DEFAULT_HZ   1000000UL

/* 等待 TBE/RBNE/TRANS 的超时：5 ms @ 108 MHz = 540,000 DWT 周期 */
#define SPI_HW_TIMEOUT_CYCLES  (540000UL)

/* 互斥锁等待时间 */
#define SPI_HW_MUTEX_WAIT_TICKS pdMS_TO_TICKS(20)

/* 分频比表：索引即 SPI_CTL0 的 PSC 字段值（3 位编码），元素为实际分频比 */
static const uint16_t s_psc_map[8] = { 2, 4, 8, 16, 32, 64, 128, 256 };

/* ============================================================
 * 内部状态
 *
 * s_mode / s_prescale 是「期望配置」的唯一来源。
 * 任何配置变化都通过 spi_hw_apply_config() 一次性写入寄存器，
 * 避免「改模式把速率带跑」「改速率把模式带跑」。
 * ============================================================ */

static SemaphoreHandle_t s_spi_mutex    = NULL;
static uint8_t           s_acquired     = 0;
static uint32_t          s_apb_clock    = 0;                 /* APB 总线频率 */
static spi_hw_mode_t     s_mode         = SPI_HW_MODE_0;     /* 当前模式 */
static uint16_t          s_prescale     = 2;                 /* 当前分频比 */
static spi_hw_status_t   s_last_status  = SPI_HW_OK;         /* 上一次事务最终状态 */

/* ============================================================
 * DWT 超时基准
 *
 * 幂等初始化：如果 DWT 已经被其它模块启用，则不再重置 CYCCNT，
 * 避免干扰正在进行的延时/超时计时。
 * ============================================================ */

static void dwt_init(void)
{
    if (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) {
        return;
    }
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ============================================================
 * 错误标志清除
 *
 * RXORERR：接收过载。清除序列：读 SPI_DATA → 读 SPI_STAT。
 * CONFERR：配置错误（主机模式下 NSS 被拉低等）。
 *           清除序列：读 SPI_STAT → 写 SPI_CTL0。
 * ============================================================ */

static void spi_hw_clear_errors(void)
{
    if (spi_i2s_flag_get(SPI_HW_PERIPH, SPI_FLAG_RXORERR)) {
        (void)SPI_DATA(SPI_HW_PERIPH);
        (void)SPI_STAT(SPI_HW_PERIPH);
    }
    if (spi_i2s_flag_get(SPI_HW_PERIPH, SPI_FLAG_CONFERR)) {
        (void)SPI_STAT(SPI_HW_PERIPH);
        SPI_CTL0(SPI_HW_PERIPH) = SPI_CTL0(SPI_HW_PERIPH);
    }
}

/* ============================================================
 * APB 时钟获取
 * ============================================================ */

static uint32_t spi_hw_get_apb_clock(void)
{
#if SPI_HW_ON_APB2
    return rcu_clock_freq_get(CK_APB2);
#else
    return rcu_clock_freq_get(CK_APB1);
#endif
}

/* ============================================================
 * 分频器选择
 *
 * 目标：选择「不大于 hz 的最大实际速率」对应的分频值。
 * 即：在 {2,4,...,256} 中找最小的 psc，使 apb/psc ≤ hz。
 * 保守策略：宁可慢一点，绝不超速。
 * ============================================================ */

static uint32_t spi_hw_prescale_from_hz(uint32_t hz)
{
    uint32_t apb = s_apb_clock ? s_apb_clock : spi_hw_get_apb_clock();
    uint32_t psc;

    if (hz == 0U) hz = SPI_HW_DEFAULT_HZ;

    for (psc = 2U; psc <= 256U; psc <<= 1) {
        if (apb / psc <= hz) {
            return psc;
        }
    }
    return 256U;
}

/* ============================================================
 * 分频比 → SPI_PSC_* 编码值
 *
 * spi_init() 的 prescale 参数要求 SPI_PSC_* 宏值（已按 CTL0 bit[5:3]
 * 编码），不是分频比本身。
 *
 * 例如：若把分频比 128 直接传给 spi_init()，写入 CTL0 的值是 0x80，
 * bit[5:3] 全为 0 → 实际分频 /2，而不是预期的 /128。
 *
 * s_psc_map[] 存的是分频比，索引恰好就是 PSC 的编码值（idx << 3 即
 * SPI_PSC_* 宏）。
 * ============================================================ */

static uint16_t spi_hw_psc_encode(uint32_t psc)
{
    uint8_t idx;

    for (idx = 0U; idx < 8U; idx++) {
        if (s_psc_map[idx] == psc) {
            return (uint16_t)(idx << 3);
        }
    }
    return SPI_PSC_256;
}

/* ============================================================
 * 模式映射
 * ============================================================ */

static uint32_t spi_hw_cpol_cpha(spi_hw_mode_t mode)
{
    switch (mode) {
        case SPI_HW_MODE_0: return SPI_CK_PL_LOW_PH_1EDGE;
        case SPI_HW_MODE_1: return SPI_CK_PL_LOW_PH_2EDGE;
        case SPI_HW_MODE_2: return SPI_CK_PL_HIGH_PH_1EDGE;
        case SPI_HW_MODE_3: return SPI_CK_PL_HIGH_PH_2EDGE;
        default:            return SPI_CK_PL_LOW_PH_1EDGE;
    }
}

/* ============================================================
 * 统一配置应用
 *
 * 按 s_mode / s_prescale 重新配置 SPI 外设。
 * GD32 的 spi_init() 会整体重写 CTL0，因此改任一参数都走这一条路径。
 * 注意：p.prescale 必须是 SPI_PSC_* 编码值，不是分频比。
 * ============================================================ */

static void spi_hw_apply_config(void)
{
    spi_parameter_struct p;

    spi_disable(SPI_HW_PERIPH);

    spi_struct_para_init(&p);
    p.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    p.device_mode          = SPI_MASTER;
    p.frame_size           = SPI_FRAMESIZE_8BIT;
    p.clock_polarity_phase = spi_hw_cpol_cpha(s_mode);
    p.nss                  = SPI_NSS_SOFT;
    p.prescale             = spi_hw_psc_encode(s_prescale);
    p.endian               = SPI_ENDIAN_MSB;

    spi_init(SPI_HW_PERIPH, &p);
    spi_enable(SPI_HW_PERIPH);

    spi_hw_clear_errors();
}

/* ============================================================
 * 初始化
 * ============================================================ */

void spi_hw_init(void)
{
    rcu_periph_clock_enable(SPI_HW_GPIO_RCU);
    rcu_periph_clock_enable(SPI_HW_RCU);
    rcu_periph_clock_enable(RCU_AF);

#if SPI_HW_SPI0_REMAP
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
    gpio_pin_remap_config(GPIO_SPI0_REMAP, ENABLE);
#endif

    /*
     * SCK/MOSI：复用推挽输出（由 SPI 外设驱动）
     * MISO：上拉输入
     */
    gpio_init(SPI_HW_GPIO_PORT, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ,
              SPI_HW_SCK_PIN | SPI_HW_MOSI_PIN);
    gpio_init(SPI_HW_GPIO_PORT, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ,
              SPI_HW_MISO_PIN);

    spi_i2s_deinit(SPI_HW_PERIPH);

    s_apb_clock = spi_hw_get_apb_clock();

    s_mode     = SPI_HW_MODE_0;
    s_prescale = (uint16_t)spi_hw_prescale_from_hz(SPI_HW_DEFAULT_HZ);
    spi_hw_apply_config();

    dwt_init();

    if (s_spi_mutex == NULL) {
        s_spi_mutex = xSemaphoreCreateMutex();
        configASSERT(s_spi_mutex != NULL);
    }
}

/* ============================================================
 * 运行时配置
 * ============================================================ */

/*
 * 辅助：确保「改配置时总线不被其他任务使用」
 *
 * 返回值：
 *   1 = 已持锁，调用方无需归还
 *   2 = 本次 take 的锁，调用方负责 give
 *   0 = 无法获得总线，不能切换
 */
static uint8_t spi_hw_lock_for_config(void)
{
    if (s_acquired) return 1U;

    if (xSemaphoreTake(s_spi_mutex, 0) != pdTRUE) {
        return 0U;
    }
    return 2U;
}

static void spi_hw_unlock_after_config(uint8_t lock_state)
{
    if (lock_state == 2U) {
        xSemaphoreGive(s_spi_mutex);
    }
}

spi_hw_status_t spi_hw_set_mode(spi_hw_mode_t mode)
{
    uint8_t lock_state;

    if ((uint32_t)mode > (uint32_t)SPI_HW_MODE_3) return SPI_HW_ERR_PARAM;

    lock_state = spi_hw_lock_for_config();
    if (lock_state == 0U) return SPI_HW_ERR_MUTEX;

    /* 只改模式，保留分频；apply_config 会 disable → init → enable，
     * 必须持锁进行，否则会打断正在进行的传输。 */
    s_mode = mode;
    spi_hw_apply_config();

    spi_hw_unlock_after_config(lock_state);
    return SPI_HW_OK;
}

spi_hw_status_t spi_hw_set_speed(uint32_t hz)
{
    uint8_t lock_state;

    if (hz == 0U) return SPI_HW_ERR_PARAM;

    lock_state = spi_hw_lock_for_config();
    if (lock_state == 0U) return SPI_HW_ERR_MUTEX;

    s_prescale = (uint16_t)spi_hw_prescale_from_hz(hz);
    spi_hw_apply_config();

    spi_hw_unlock_after_config(lock_state);
    return SPI_HW_OK;
}

uint32_t spi_hw_get_max_hz(void)
{
    uint32_t apb = s_apb_clock ? s_apb_clock : spi_hw_get_apb_clock();
    return apb / 2U;
}

/* ============================================================
 * 总线互斥
 * ============================================================ */

spi_hw_status_t spi_hw_acquire(void)
{
    if (xSemaphoreTake(s_spi_mutex, SPI_HW_MUTEX_WAIT_TICKS) != pdTRUE)
        return SPI_HW_ERR_MUTEX;

    s_acquired    = 1U;
    s_last_status = SPI_HW_OK;
    spi_hw_clear_errors();
    return SPI_HW_OK;
}

void spi_hw_release(void)
{
    uint8_t timeout = 0U;

    if (!s_acquired) return;

    /* 等待最后一帧完全移出（TRANS 清零） */
    {
        uint32_t start = DWT->CYCCNT;
        while (spi_i2s_flag_get(SPI_HW_PERIPH, SPI_FLAG_TRANS)) {
            if ((DWT->CYCCNT - start) > SPI_HW_TIMEOUT_CYCLES) {
                timeout = 1U;
                break;
            }
        }
    }

    /*
     * TRANS 超时说明外设卡死（SCK 被外部拉死、MISO 短路、从机掉电等），
     * 复位 SPI 外设并按当前 s_mode/s_prescale 重建，避免下次 acquire
     * 继续带病工作。同时置位错误状态，让上层知道本次事务最终失败。
     */
    if (timeout) {
        spi_disable(SPI_HW_PERIPH);
        spi_hw_apply_config();
        s_last_status = SPI_HW_ERR_TIMEOUT;
    }

    s_acquired = 0U;
    xSemaphoreGive(s_spi_mutex);
}

spi_hw_status_t spi_hw_get_last_status(void)
{
    return s_last_status;
}

/* ============================================================
 * 单字节全双工交换
 *
 * 硬件 SPI 全双工模式下写一字节必然收一字节，因此流程是：
 *   1. 等 TBE → 写入待发数据
 *   2. 等 RBNE → 读出接收数据
 * 每一步都带 DWT 超时并检查硬件错误标志。
 * ============================================================ */

uint8_t spi_hw_xfer_byte(uint8_t tx)
{
    uint32_t start;

    configASSERT(s_acquired);

    /* ---------- 等待发送缓冲区空 ---------- */
    start = DWT->CYCCNT;
    while (!spi_i2s_flag_get(SPI_HW_PERIPH, SPI_FLAG_TBE)) {
        if (spi_i2s_flag_get(SPI_HW_PERIPH, SPI_FLAG_RXORERR) ||
            spi_i2s_flag_get(SPI_HW_PERIPH, SPI_FLAG_CONFERR)) {
            spi_hw_clear_errors();
            s_last_status = SPI_HW_ERR_HW;
            return 0xFFU;
        }
        if ((DWT->CYCCNT - start) > SPI_HW_TIMEOUT_CYCLES) {
            s_last_status = SPI_HW_ERR_TIMEOUT;
            return 0xFFU;
        }
    }

    spi_i2s_data_transmit(SPI_HW_PERIPH, tx);

    /* ---------- 等待接收缓冲区非空 ---------- */
    start = DWT->CYCCNT;
    while (!spi_i2s_flag_get(SPI_HW_PERIPH, SPI_FLAG_RBNE)) {
        if (spi_i2s_flag_get(SPI_HW_PERIPH, SPI_FLAG_RXORERR) ||
            spi_i2s_flag_get(SPI_HW_PERIPH, SPI_FLAG_CONFERR)) {
            spi_hw_clear_errors();
            s_last_status = SPI_HW_ERR_HW;
            return 0xFFU;
        }
        if ((DWT->CYCCNT - start) > SPI_HW_TIMEOUT_CYCLES) {
            s_last_status = SPI_HW_ERR_TIMEOUT;
            return 0xFFU;
        }
    }

    return (uint8_t)spi_i2s_data_receive(SPI_HW_PERIPH);
}

/* ============================================================
 * 多字节全双工交换
 *
 * 每轮先检查 s_last_status，避免「上一次已出错还继续发下一个字节」。
 * ============================================================ */

void spi_hw_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    uint16_t i;

    configASSERT(s_acquired);

    for (i = 0U; i < len; i++) {
        /* 先检查历史状态：已出错则不发新字节 */
        if (s_last_status != SPI_HW_OK) {
            return;
        }

        uint8_t out = (tx != NULL) ? tx[i] : 0xFFU;
        uint8_t in  = spi_hw_xfer_byte(out);

        /* 单字节内出错：立即返回，不覆盖本次已读数据以外的东西 */
        if (s_last_status != SPI_HW_OK) {
            return;
        }

        if (rx != NULL) rx[i] = in;
    }
}

/* ============================================================
 * 一体化接口
 * ============================================================ */

spi_hw_status_t spi_hw_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    spi_hw_status_t status;

    if (len == 0U || (tx == NULL && rx == NULL))
        return SPI_HW_ERR_PARAM;

    status = spi_hw_acquire();
    if (status != SPI_HW_OK) return status;

    spi_hw_xfer(tx, rx, len);

    status = s_last_status;
    spi_hw_release();

    /* release 内可能因 TRANS 超时再次修改 s_last_status，重新查询一次 */
    if (status == SPI_HW_OK) {
        status = s_last_status;
    }
    return status;
}