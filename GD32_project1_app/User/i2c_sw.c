#include "i2c_sw.h"

/* ======================== 可配置项 ======================== */

#define I2C_GPIO_PORT       GPIOB
#define I2C_SCL_PIN         GPIO_PIN_6
#define I2C_SDA_PIN         GPIO_PIN_7
#define I2C_GPIO_RCU        RCU_GPIOB

#define I2C_HALF_DELAY_US   6U

#define I2C_TIMEOUT_CYCLES  (5400000UL)     /* 50ms @ 108MHz，容忍 AHT20 测量期间的 clock stretch */
#define I2C_MUTEX_WAIT_TICKS  pdMS_TO_TICKS(20)
#define I2C_RX_MAX_LEN      255U

/* ======================== GPIO 位操作 ======================== */

#define SCL_HIGH()   gpio_bit_set(I2C_GPIO_PORT, I2C_SCL_PIN)
#define SCL_LOW()    gpio_bit_reset(I2C_GPIO_PORT, I2C_SCL_PIN)
#define SDA_HIGH()   gpio_bit_set(I2C_GPIO_PORT, I2C_SDA_PIN)
#define SDA_LOW()    gpio_bit_reset(I2C_GPIO_PORT, I2C_SDA_PIN)

#define SCL_READ()   gpio_input_bit_get(I2C_GPIO_PORT, I2C_SCL_PIN)
#define SDA_READ()   gpio_input_bit_get(I2C_GPIO_PORT, I2C_SDA_PIN)

static SemaphoreHandle_t s_i2c_mutex = NULL;

/* ======================== DWT 精确延时 ======================== */

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void i2c_delay_us(uint32_t us)
{
    uint32_t start  = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < cycles) { }
}

/* ======================== 超时与时钟拉伸 ======================== */

static i2c_status_t scl_wait_high(void)
{
    uint32_t start = DWT->CYCCNT;
    while (SCL_READ() == RESET) {
        if ((DWT->CYCCNT - start) > I2C_TIMEOUT_CYCLES) {
            return I2C_ERR_TIMEOUT;
        }
    }
    return I2C_OK;
}

static i2c_status_t i2c_wait_idle(void)
{
    uint32_t start = DWT->CYCCNT;
    while (SCL_READ() == RESET || SDA_READ() == RESET) {
        if ((DWT->CYCCNT - start) > I2C_TIMEOUT_CYCLES) {
            return I2C_ERR_BUS_BUSY;
        }
    }
    return I2C_OK;
}

/* ======================== 基本时序单元 ======================== */

static i2c_status_t i2c_start(void)
{
    i2c_status_t status;

    SDA_HIGH();
    SCL_HIGH();
    status = scl_wait_high();
    if (status != I2C_OK) return status;
    i2c_delay_us(I2C_HALF_DELAY_US);

    SDA_LOW();
    i2c_delay_us(I2C_HALF_DELAY_US);

    SCL_LOW();
    i2c_delay_us(I2C_HALF_DELAY_US);
    return I2C_OK;
}

static i2c_status_t i2c_stop(void)
{
    i2c_status_t status;

    SDA_LOW();
    i2c_delay_us(I2C_HALF_DELAY_US);

    SCL_HIGH();
    status = scl_wait_high();
    if (status != I2C_OK) {
        /* 关键修复：SCL 拉不高（从机 clock stretch / 死锁），
         * 至少要把 SDA 拉高，避免留下"SDA 低"的脏状态，
         * 否则下一次 recover 会误判为"总线被占用"而永远修不好。 */
        SDA_HIGH();
        i2c_delay_us(I2C_HALF_DELAY_US);
        return status;
    }
    i2c_delay_us(I2C_HALF_DELAY_US);

    SDA_HIGH();
    i2c_delay_us(I2C_HALF_DELAY_US);
    return I2C_OK;
}

static i2c_status_t i2c_write_byte(uint8_t byte)
{
    i2c_status_t status;
    uint8_t i, ack;

    for (i = 0; i < 8; i++) {
        if (byte & 0x80) SDA_HIGH();
        else             SDA_LOW();
        byte <<= 1;

        i2c_delay_us(I2C_HALF_DELAY_US);
        SCL_HIGH();
        status = scl_wait_high();
        if (status != I2C_OK) return status;
        i2c_delay_us(I2C_HALF_DELAY_US);
        SCL_LOW();
    }

    SDA_HIGH();
    i2c_delay_us(I2C_HALF_DELAY_US);

    SCL_HIGH();
    status = scl_wait_high();
    if (status != I2C_OK) return status;
    i2c_delay_us(I2C_HALF_DELAY_US);

    ack = (SDA_READ() == RESET) ? 1U : 0U;

    SCL_LOW();
    i2c_delay_us(I2C_HALF_DELAY_US);

    return ack ? I2C_OK : I2C_ERR_NACK;
}

static i2c_status_t i2c_read_byte(uint8_t *byte, uint8_t send_ack)
{
    i2c_status_t status;
    uint8_t i, val = 0;

    SDA_HIGH();

    for (i = 0; i < 8; i++) {
        val <<= 1;

        i2c_delay_us(I2C_HALF_DELAY_US);
        SCL_HIGH();
        status = scl_wait_high();
        if (status != I2C_OK) return status;
        i2c_delay_us(I2C_HALF_DELAY_US);

        if (SDA_READ() == SET) val |= 1U;

        SCL_LOW();
    }

    if (send_ack) SDA_LOW();
    else          SDA_HIGH();

    i2c_delay_us(I2C_HALF_DELAY_US);
    SCL_HIGH();
    status = scl_wait_high();
    if (status != I2C_OK) return status;
    i2c_delay_us(I2C_HALF_DELAY_US);
    SCL_LOW();

    SDA_HIGH();
    i2c_delay_us(I2C_HALF_DELAY_US);

    *byte = val;
    return I2C_OK;
}

/* ======================== 总线恢复 ======================== */

static i2c_status_t i2c_bus_recover_if_needed(void)
{
    /* 关键：如果 SCL 低，先等 20ms —— 可能是 AHT20 正常 clock stretching，
     * 不要立即发 9 个时钟骚扰它。AHT20 测量 80ms，90ms 足够它自己释放。 */
    if (SCL_READ() == RESET) {
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            /* 调度器运行：用 vTaskDelay 让出 CPU，其他任务照常跑 */
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            /* 调度器未运行（启动期）：忙等 20ms */
            for (volatile uint32_t k = 0; k < 2000000U; k++) __NOP();
        }
        if (SCL_READ() == SET) return I2C_OK;   /* AHT20 自己释放了 */
    }

    /* 等到 20ms 还低 → 真的死锁，发 9 个时钟救 */
    if (SDA_READ() == RESET || SCL_READ() == RESET) {
        for (uint8_t i = 0; i < 9; i++) {
            SCL_LOW();
            i2c_delay_us(I2C_HALF_DELAY_US);
            SCL_HIGH();
            i2c_delay_us(I2C_HALF_DELAY_US);
            if (SDA_READ() == SET && SCL_READ() == SET) break;
        }

        SDA_LOW();
        i2c_delay_us(I2C_HALF_DELAY_US);
        SCL_HIGH();
        i2c_delay_us(I2C_HALF_DELAY_US);
        SDA_HIGH();
        i2c_delay_us(I2C_HALF_DELAY_US);

        if (SDA_READ() == RESET || SCL_READ() == RESET) {
            return I2C_ERR_RECOVER_FAIL;
        }
    }
    return I2C_OK;
}

/* ======================== 初始化 ======================== */

void sw_i2c_init(void)
{
    rcu_periph_clock_enable(I2C_GPIO_RCU);

    /* 阶段 1：推挽输出高，强制释放总线。
     * 用来覆盖 Bootloader 跳转时遗留的低电平，以及 MCU 复位瞬间可能的半截信号。 */
    gpio_init(I2C_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ,
              I2C_SCL_PIN | I2C_SDA_PIN);
    gpio_bit_set(I2C_GPIO_PORT, I2C_SCL_PIN | I2C_SDA_PIN);
    for (volatile int d = 0; d < 10000; d++) { __NOP(); }   /* 保持高 ~100us+ */

    /* 阶段 2：切回开漏，正常 I2C 模式 */
    gpio_init(I2C_GPIO_PORT, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ,
              I2C_SCL_PIN | I2C_SDA_PIN);

    SCL_HIGH();
    SDA_HIGH();

    /* 阶段 3：初始化后立刻尝试一次总线恢复，把可能卡住的从机踢出来 */
    (void)i2c_bus_recover_if_needed();

    dwt_init();

    if (s_i2c_mutex == NULL) {
        s_i2c_mutex = xSemaphoreCreateMutex();
        configASSERT(s_i2c_mutex != NULL);
    }
}

/* ======================== 寄存器写（多字节） ======================== */

/*
 * 修改点：允许 len == 0。
 * AHT20 软复位 0xBA 只发一个控制字节，没有后续数据。
 */
i2c_status_t sw_i2c_mem_write(uint8_t addr_7bit, uint8_t reg,
                               const uint8_t *buf, uint16_t len)
{
    i2c_status_t status;
    uint16_t i;

    /* len == 0 时 buf 不访问，允许传 NULL */
    if (len > 0 && buf == NULL) return I2C_ERR_PARAM;

    if (xSemaphoreTake(s_i2c_mutex, I2C_MUTEX_WAIT_TICKS) != pdTRUE)
        return I2C_ERR_MUTEX;

    status = i2c_bus_recover_if_needed();
    if (status != I2C_OK) goto exit;

    status = i2c_wait_idle();
    if (status != I2C_OK) goto exit;

    status = i2c_start();
    if (status != I2C_OK) goto stop;

    status = i2c_write_byte((uint8_t)(addr_7bit << 1));
    if (status != I2C_OK) goto stop;

    status = i2c_write_byte(reg);
    if (status != I2C_OK) goto stop;

    for (i = 0; i < len; i++) {                 /* len == 0 时循环不执行 */
        status = i2c_write_byte(buf[i]);
        if (status != I2C_OK) goto stop;
    }

stop:
    i2c_stop();

exit:
    xSemaphoreGive(s_i2c_mutex);
    return status;
}

/* ======================== 寄存器读（多字节） ======================== */

i2c_status_t sw_i2c_mem_read(uint8_t addr_7bit, uint8_t reg,
                              uint8_t *buf, uint16_t len)
{
    i2c_status_t status;
    uint16_t remaining;

    if (buf == NULL || len == 0 || len > I2C_RX_MAX_LEN)
        return I2C_ERR_PARAM;

    if (xSemaphoreTake(s_i2c_mutex, I2C_MUTEX_WAIT_TICKS) != pdTRUE)
        return I2C_ERR_MUTEX;

    status = i2c_bus_recover_if_needed();
    if (status != I2C_OK) goto exit;

    status = i2c_wait_idle();
    if (status != I2C_OK) goto exit;

    status = i2c_start();
    if (status != I2C_OK) goto stop;

    status = i2c_write_byte((uint8_t)(addr_7bit << 1));
    if (status != I2C_OK) goto stop;

    status = i2c_write_byte(reg);
    if (status != I2C_OK) goto stop;

    status = i2c_start();
    if (status != I2C_OK) goto stop;

    status = i2c_write_byte((uint8_t)((addr_7bit << 1) | 0x01U));
    if (status != I2C_OK) goto stop;

    remaining = len;
    while (remaining > 0) {
        uint8_t send_ack = (remaining > 1U) ? 1U : 0U;
        status = i2c_read_byte(buf, send_ack);
        if (status != I2C_OK) goto stop;
        buf++;
        remaining--;
    }

stop:
    i2c_stop();

exit:
    xSemaphoreGive(s_i2c_mutex);
    return status;
}

/* ======================== 直接读（无寄存器阶段） ======================== */

/*
 * 新增：START → 地址(R) → 读 len 字节 → STOP。
 * AHT20 读状态 / 读结果走这条路径。
 */
i2c_status_t sw_i2c_raw_read(uint8_t addr_7bit, uint8_t *buf, uint16_t len)
{
    i2c_status_t status;
    uint16_t remaining;

    if (buf == NULL || len == 0 || len > I2C_RX_MAX_LEN)
        return I2C_ERR_PARAM;

    if (xSemaphoreTake(s_i2c_mutex, I2C_MUTEX_WAIT_TICKS) != pdTRUE)
        return I2C_ERR_MUTEX;

    status = i2c_bus_recover_if_needed();
    if (status != I2C_OK) goto exit;

    status = i2c_wait_idle();
    if (status != I2C_OK) goto exit;

    status = i2c_start();
    if (status != I2C_OK) goto stop;

    status = i2c_write_byte((uint8_t)((addr_7bit << 1) | 0x01U));
    if (status != I2C_OK) goto stop;

    remaining = len;
    while (remaining > 0) {
        uint8_t send_ack = (remaining > 1U) ? 1U : 0U;
        status = i2c_read_byte(buf, send_ack);
        if (status != I2C_OK) goto stop;
        buf++;
        remaining--;
    }

stop:
    i2c_stop();

exit:
    xSemaphoreGive(s_i2c_mutex);
    return status;
}