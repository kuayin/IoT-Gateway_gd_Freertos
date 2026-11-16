#include "i2c_hw.h"

#define I2C_PERIPH          I2C0
#define I2C_GPIO_PORT       GPIOB
#define I2C_SCL_PIN         GPIO_PIN_6
#define I2C_SDA_PIN         GPIO_PIN_7
#define I2C_GPIO_RCU        RCU_GPIOB
#define I2C_PERIPH_RCU      RCU_I2C0

/* 超时：5ms @ 108MHz = 540,000 个 DWT 周期 */
#define I2C_TIMEOUT_CYCLES  (540000UL)

/* 纯循环计数兜底：DWT 失效时的保险，约 2~3ms @ 108MHz */
#define I2C_TIMEOUT_LOOPS   200000UL


/* 互斥锁等待时间 */
#define I2C_MUTEX_WAIT_TICKS  pdMS_TO_TICKS(20)

/* 读长度上限（轮询方式保守限制） */
#define I2C_RX_MAX_LEN      255U

static SemaphoreHandle_t s_i2c_mutex = NULL;

/* ======================== DWT ======================== */

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ======================== 等待与错误处理 ======================== */

static i2c_status_t i2c_wait_idle(void)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t loops = 0;
    while (i2c_flag_get(I2C_PERIPH, I2C_FLAG_I2CBSY)) {
        if ((++loops > I2C_TIMEOUT_LOOPS) ||
            ((DWT->CYCCNT - start) > I2C_TIMEOUT_CYCLES)) {
            return I2C_ERR_BUS_BUSY;
        }
    }
    return I2C_OK;
}

static i2c_status_t i2c_wait_stop_done(void)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t loops = 0;

    while ((I2C_CTL0(I2C_PERIPH) & I2C_CTL0_STOP) != 0U) {
        if ((++loops > I2C_TIMEOUT_LOOPS) ||
            ((DWT->CYCCNT - start) > I2C_TIMEOUT_CYCLES)) {
            return I2C_ERR_TIMEOUT;
        }
    }

    i2c_flag_clear(I2C_PERIPH, I2C_FLAG_STPDET);
    return I2C_OK;
}



static i2c_status_t i2c_wait_flag(uint32_t flag)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t loops = 0;

    while (!i2c_flag_get(I2C_PERIPH, flag)) {
        if (i2c_flag_get(I2C_PERIPH, I2C_FLAG_AERR)) {
            i2c_flag_clear(I2C_PERIPH, I2C_FLAG_AERR);
            i2c_stop_on_bus(I2C_PERIPH);
            (void)i2c_wait_stop_done();     /* ← 关键：等 STOP 完成再返回 */
            return I2C_ERR_NACK;
        }
        /* 双保险：DWT 或 纯循环，任一触发即超时 */
        if ((++loops > I2C_TIMEOUT_LOOPS) ||
            ((DWT->CYCCNT - start) > I2C_TIMEOUT_CYCLES)) {
            i2c_stop_on_bus(I2C_PERIPH);
            (void)i2c_wait_stop_done();     /* ← 超时也要等 STOP 完成 */
            return I2C_ERR_TIMEOUT;
        }
    }
    return I2C_OK;
}


static inline void i2c_clear_addsend(void)
{
    (void)I2C_STAT0(I2C_PERIPH);
    (void)I2C_STAT1(I2C_PERIPH);
}

/* ======================== 总线恢复 ======================== */

static i2c_status_t i2c_bus_recover_if_needed(void)
{
    if (gpio_input_bit_get(I2C_GPIO_PORT, I2C_SDA_PIN) == RESET) {

        gpio_init(I2C_GPIO_PORT, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, I2C_SCL_PIN);

        for (uint8_t i = 0; i < 9; i++) {
            gpio_bit_reset(I2C_GPIO_PORT, I2C_SCL_PIN);
            for (volatile int d = 0; d < 50; d++) { __NOP(); }
            gpio_bit_set(I2C_GPIO_PORT, I2C_SCL_PIN);
            for (volatile int d = 0; d < 50; d++) { __NOP(); }

            if (gpio_input_bit_get(I2C_GPIO_PORT, I2C_SDA_PIN) == SET) {
                break;
            }
        }

        i2c_software_reset_config(I2C_PERIPH, I2C_SRESET_RESET);
        i2c_software_reset_config(I2C_PERIPH, I2C_SRESET_SET);

        gpio_init(I2C_GPIO_PORT, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ,
                  I2C_SCL_PIN | I2C_SDA_PIN);

				i2c_clock_config(I2C_PERIPH, 400000, I2C_DTCY_2);
				i2c_mode_addr_config(I2C_PERIPH, I2C_I2CMODE_ENABLE,
																		I2C_ADDFORMAT_7BITS, 0x30);
				
        i2c_enable(I2C_PERIPH);
        i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);
        i2c_ackpos_config(I2C_PERIPH, I2C_ACKPOS_CURRENT);

        if (gpio_input_bit_get(I2C_GPIO_PORT, I2C_SDA_PIN) == RESET) {
            return I2C_ERR_RECOVER_FAIL;
        }
    }
    return I2C_OK;
}

/* ======================== 初始化 ======================== */

void hw_i2c_init(void)
{
    rcu_periph_clock_enable(I2C_GPIO_RCU);
    rcu_periph_clock_enable(I2C_PERIPH_RCU);

    gpio_init(I2C_GPIO_PORT, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ,
              I2C_SCL_PIN | I2C_SDA_PIN);

    i2c_clock_config(I2C_PERIPH, 400000, I2C_DTCY_2);
    i2c_mode_addr_config(I2C_PERIPH, I2C_I2CMODE_ENABLE,
                         I2C_ADDFORMAT_7BITS, 0x30);
    i2c_enable(I2C_PERIPH);
    i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);
    i2c_ackpos_config(I2C_PERIPH, I2C_ACKPOS_CURRENT);

    dwt_init();

    if (s_i2c_mutex == NULL) {
        s_i2c_mutex = xSemaphoreCreateMutex();
        configASSERT(s_i2c_mutex != NULL);
    }
}

/* ======================== 寄存器写（多字节） ======================== */

/*
 * 修改点：允许 len == 0。
 * 这样 "reg 本身即是唯一控制字节" 的从机（如 AHT20 软复位 0xBA）才能使用。
 * 序列：START → 地址(W) → reg → (可选 len 字节) → STOP。
 */
i2c_status_t hw_i2c_mem_write(uint8_t addr_7bit, uint8_t reg,
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

    i2c_start_on_bus(I2C_PERIPH);
    status = i2c_wait_flag(I2C_FLAG_SBSEND);
    if (status != I2C_OK) goto exit;

    i2c_master_addressing(I2C_PERIPH, (uint16_t)(addr_7bit << 1), I2C_TRANSMITTER);
    status = i2c_wait_flag(I2C_FLAG_ADDSEND);
    if (status != I2C_OK) goto exit;
    i2c_clear_addsend();

    i2c_data_transmit(I2C_PERIPH, reg);
    status = i2c_wait_flag(I2C_FLAG_BTC);
    if (status != I2C_OK) goto exit;

    for (i = 0; i < len; i++) {                 /* len == 0 时循环不执行 */
        i2c_data_transmit(I2C_PERIPH, buf[i]);
        status = i2c_wait_flag(I2C_FLAG_BTC);
        if (status != I2C_OK) goto exit;
    }

    i2c_stop_on_bus(I2C_PERIPH);
    status = i2c_wait_stop_done();

exit:
		if (status != I2C_OK) {
        (void)i2c_bus_recover_if_needed();    /* 失败后确保总线干净 */
    }
		
    xSemaphoreGive(s_i2c_mutex);
    return status;
}

/* ======================== 寄存器读（多字节） ======================== */

i2c_status_t hw_i2c_mem_read(uint8_t addr_7bit, uint8_t reg,
                              uint8_t *buf, uint16_t len)
{
    i2c_status_t status;
    uint16_t remaining;
    uint8_t poap_was_set = 0;

    if (buf == NULL || len == 0 || len > I2C_RX_MAX_LEN)
        return I2C_ERR_PARAM;

    if (xSemaphoreTake(s_i2c_mutex, I2C_MUTEX_WAIT_TICKS) != pdTRUE)
        return I2C_ERR_MUTEX;

    status = i2c_bus_recover_if_needed();
    if (status != I2C_OK) goto exit;

    status = i2c_wait_idle();
    if (status != I2C_OK) goto exit;

    /* 阶段 1：写寄存器地址 */
    i2c_start_on_bus(I2C_PERIPH);
    status = i2c_wait_flag(I2C_FLAG_SBSEND);
    if (status != I2C_OK) goto exit;

    i2c_master_addressing(I2C_PERIPH, (uint16_t)(addr_7bit << 1), I2C_TRANSMITTER);
    status = i2c_wait_flag(I2C_FLAG_ADDSEND);
    if (status != I2C_OK) goto exit;
    i2c_clear_addsend();

    i2c_data_transmit(I2C_PERIPH, reg);
    status = i2c_wait_flag(I2C_FLAG_BTC);
    if (status != I2C_OK) goto exit;

    /* 阶段 2：重复起始 + 读地址 */
    i2c_start_on_bus(I2C_PERIPH);
    status = i2c_wait_flag(I2C_FLAG_SBSEND);
    if (status != I2C_OK) goto exit;

    if (len == 1) {
        i2c_ack_config(I2C_PERIPH, I2C_ACK_DISABLE);
    } else if (len == 2) {
        i2c_ackpos_config(I2C_PERIPH, I2C_ACKPOS_NEXT);
        poap_was_set = 1;
        i2c_ack_config(I2C_PERIPH, I2C_ACK_DISABLE);
    } else {
        i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);
    }

    i2c_master_addressing(I2C_PERIPH, (uint16_t)(addr_7bit << 1), I2C_RECEIVER);
    status = i2c_wait_flag(I2C_FLAG_ADDSEND);
    if (status != I2C_OK) goto exit;
    i2c_clear_addsend();

    if (len == 1) {
        i2c_stop_on_bus(I2C_PERIPH);
    } else if (len == 2) {
        status = i2c_wait_flag(I2C_FLAG_BTC);
        if (status != I2C_OK) goto exit;
        i2c_stop_on_bus(I2C_PERIPH);
    }

    /* 阶段 3：逐字节接收 */
    remaining = len;

    while (remaining > 0) {

        if (len >= 3 && remaining == 3) {
            status = i2c_wait_flag(I2C_FLAG_BTC);
            if (status != I2C_OK) goto exit;
            i2c_ack_config(I2C_PERIPH, I2C_ACK_DISABLE);
        }

        if (len >= 3 && remaining == 2) {
            status = i2c_wait_flag(I2C_FLAG_BTC);
            if (status != I2C_OK) goto exit;
            i2c_stop_on_bus(I2C_PERIPH);
        }

        status = i2c_wait_flag(I2C_FLAG_RBNE);
        if (status != I2C_OK) goto exit;

        *buf++ = (uint8_t)i2c_data_receive(I2C_PERIPH);
        remaining--;
    }

    status = i2c_wait_stop_done();

exit:
    if (status != I2C_OK) {
        (void)i2c_bus_recover_if_needed();    /* 失败后确保总线干净 */
    }
		
		i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);
    if (poap_was_set) {
        i2c_ackpos_config(I2C_PERIPH, I2C_ACKPOS_CURRENT);
    }
    xSemaphoreGive(s_i2c_mutex);
    return status;
}

/* ======================== 直接读（无寄存器阶段） ======================== */

/*
 * 新增：START → 地址(R) → 读 len 字节 → STOP。
 * AHT20 触发测量后，状态字节和 7 字节结果都必须走这条路径。
 * ACK/NACK/POAP/STOP 时机与 mem_read 的阶段 2~3 一致。
 */
i2c_status_t hw_i2c_raw_read(uint8_t addr_7bit, uint8_t *buf, uint16_t len)
{
    i2c_status_t status;
    uint16_t remaining;
    uint8_t poap_was_set = 0;

    if (buf == NULL || len == 0 || len > I2C_RX_MAX_LEN)
        return I2C_ERR_PARAM;

    if (xSemaphoreTake(s_i2c_mutex, I2C_MUTEX_WAIT_TICKS) != pdTRUE)
        return I2C_ERR_MUTEX;

    status = i2c_bus_recover_if_needed();
    if (status != I2C_OK) goto exit;

    status = i2c_wait_idle();
    if (status != I2C_OK) goto exit;

    i2c_start_on_bus(I2C_PERIPH);
    status = i2c_wait_flag(I2C_FLAG_SBSEND);
    if (status != I2C_OK) goto exit;

    if (len == 1) {
        i2c_ack_config(I2C_PERIPH, I2C_ACK_DISABLE);
    } else if (len == 2) {
        i2c_ackpos_config(I2C_PERIPH, I2C_ACKPOS_NEXT);
        poap_was_set = 1;
        i2c_ack_config(I2C_PERIPH, I2C_ACK_DISABLE);
    } else {
        i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);
    }

    i2c_master_addressing(I2C_PERIPH, (uint16_t)(addr_7bit << 1), I2C_RECEIVER);
    status = i2c_wait_flag(I2C_FLAG_ADDSEND);
    if (status != I2C_OK) goto exit;
    i2c_clear_addsend();

    if (len == 1) {
        i2c_stop_on_bus(I2C_PERIPH);
    } else if (len == 2) {
        status = i2c_wait_flag(I2C_FLAG_BTC);
        if (status != I2C_OK) goto exit;
        i2c_stop_on_bus(I2C_PERIPH);
    }

    remaining = len;
    while (remaining > 0) {

        if (len >= 3 && remaining == 3) {
            status = i2c_wait_flag(I2C_FLAG_BTC);
            if (status != I2C_OK) goto exit;
            i2c_ack_config(I2C_PERIPH, I2C_ACK_DISABLE);
        }

        if (len >= 3 && remaining == 2) {
            status = i2c_wait_flag(I2C_FLAG_BTC);
            if (status != I2C_OK) goto exit;
            i2c_stop_on_bus(I2C_PERIPH);
        }

        status = i2c_wait_flag(I2C_FLAG_RBNE);
        if (status != I2C_OK) goto exit;

        *buf++ = (uint8_t)i2c_data_receive(I2C_PERIPH);
        remaining--;
    }

    status = i2c_wait_stop_done();

exit:
    if (status != I2C_OK) {
        (void)i2c_bus_recover_if_needed();    /* 失败后确保总线干净 */
    }
		
		i2c_ack_config(I2C_PERIPH, I2C_ACK_ENABLE);
    if (poap_was_set) {
        i2c_ackpos_config(I2C_PERIPH, I2C_ACKPOS_CURRENT);
    }
    xSemaphoreGive(s_i2c_mutex);
    return status;
}