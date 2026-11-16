
#include "gd32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "gd32f10x_usart.h"
#include "gd32f10x_dma.h"
#include "gd32f10x_fwdgt.h"     /* GD32 独立看门狗（对应 STM32 的 IWDG） */
#include "OLED_I2C.h"
#include <stdio.h>
#include "sensor.h"                 /* AHT20 + BMP280 驱动 */
#include <string.h>
#include "bootloader_start.h"

/* ============================================================================
 *                                配置宏
 * ============================================================================ */

/* 环形缓冲区大小：必须是 2 的幂，便于用按位与代替取模 */
#define RX_BUF_SIZE             1024U
#if ((RX_BUF_SIZE & (RX_BUF_SIZE - 1U)) != 0U)
#error "RX_BUF_SIZE must be a power of two"
#endif

/* 积压安全阈值：超过该值认为存在溢出风险，强制丢弃并重新同步 */
#define RX_OVERFLOW_THRESHOLD   (RX_BUF_SIZE / 2U)

/* 串口发送单字节的超时时间（115200 bps 下 1 字节约 87 us，5 ms 有 50 倍余量） */
#define UART_TX_BYTE_TIMEOUT_MS 5U

/* 状态机“帧间超时”：静默超过该时长则复位到 WAIT_HEADER1 */
#define PARSER_IDLE_RESET_MS    50U

/* OLED 初始化最大重试次数（每次间隔 500ms） */
#define OLED_INIT_RETRY_MAX     4U

/* 任务栈大小（单位：word；Cortex-M3 上 1 word = 4 字节） */
#define TASK_STACK_HEARTBEAT    512U
#define TASK_STACK_UARTPROC     384U
#define TASK_STACK_MONITOR      128U

/* 任务优先级：数值越大优先级越高（FreeRTOS 语义） */
#define PRIO_MONITOR            1U
#define PRIO_HEARTBEAT          2U
#define PRIO_UARTPROC           3U

/* FWDGT 超时约 2 秒：
 *   时钟源 IRC40K 约 40 kHz，预分频 64 → 625 Hz；
 *   重载值 1250 → 625 * 2 = 1250 → 约 2 秒超时。 */
#define IWDG_RELOAD_VAL         3000U


/* ============================================================================
 *                                全局数据
 * ============================================================================ */

/* DMA 循环接收缓冲区：仅 USART0 硬件写、解析任务读（无锁 SPSC） */
static uint8_t  s_dma_rx_buf[RX_BUF_SIZE];

/* 读指针：由解析任务维护，表示“下一个要解析的字节” */
static volatile uint16_t s_read_index  = 0;

/* 写指针：由 USART0 空闲中断维护，表示“DMA 下一个将写入的位置” */
static volatile uint16_t s_write_index = 0;

/* 跨任务共享的 LED 状态（0=灭 1=亮），volatile 保证编译期不被优化掉 */
static volatile uint8_t  s_led_state   = 0;

/* ---- 诊断计数器：仅用于观测，非控制逻辑 ---- */
static volatile uint32_t s_diag_overflow_cnt     = 0;  /* 环形缓冲区强制丢弃次数 */
static volatile uint32_t s_diag_checksum_err_cnt = 0;  /* 校验失败次数          */
static volatile uint32_t s_diag_frame_ok_cnt     = 0;  /* 正确帧计数            */

/* ---- 健康监控“心跳”变量：由关键任务递增，监控任务据此判断是否喂狗 ---- */
static volatile uint32_t s_hb_tick        = 0;   /* vTaskHeartbeat 递增 */
static volatile uint32_t s_uartproc_tick  = 0;   /* vTaskUartProcess 递增 */

/* 二值信号量：由 USART0 空闲中断发出，通知解析任务“有新数据到了” */
static SemaphoreHandle_t s_uart_rx_sem = NULL;


//发送AHT20和BMP280数据，从串口
static volatile uint8_t uart0_aht20_bmp280 = 0;

/* ============================================================================
 *                          轻量工具函数
 * ============================================================================ */

/**
 * @brief 无符号 32 位整数转十进制字符串（避免引入 sprintf 的栈开销）
 * @param buf  输出缓冲区，至少 11 字节
 * @param val  待转换的数值
 */
static void u32_to_dec(char *buf, uint32_t val)
{
    char tmp[11];
    uint32_t i = 0, j;
    if (val == 0U) { buf[0] = '0'; buf[1] = '\0'; return; }
    while (val != 0U && i < 10U) {
        tmp[i++] = (char)('0' + (val % 10U));
        val /= 10U;
    }
    for (j = 0U; j < i; j++) {
        buf[j] = tmp[i - 1U - j];
    }
    buf[i] = '\0';
}

/* 浮点数格式化为 1 位小数，写入 p，返回新的末尾指针 */
static char *append_dec1(char *p, float v)
{
    int32_t x10 = (int32_t)(v * 10.0f + (v >= 0.0f ? 0.5f : -0.5f));
    if (x10 < 0) { *p++ = '-'; x10 = -x10; }
    uint32_t ip = (uint32_t)(x10 / 10);
    uint8_t  fp = (uint8_t)(x10 % 10);
    char     tmp[8];
    uint8_t  n = 0;
    if (ip == 0) {
        *p++ = '0';
    } else {
        while (ip) { tmp[n++] = (char)('0' + ip % 10); ip /= 10; }
        while (n)  *p++ = tmp[--n];
    }
    *p++ = '.';
    *p++ = (char)('0' + fp);
    *p   = '\0';
    return p;
}

/* 右补空格到 width（擦掉上一次长字符残留） */
static void pad_to(char *line, uint8_t width)
{
    uint8_t len = 0;
    while (line[len] != '\0') len++;
    while (len < width) line[len++] = ' ';
    line[len] = '\0';
}





/* ============================================================================
 *                          串口发送（带超时）
 * ============================================================================ */

/**
 * @brief 发送单个字节，带超时
 * @note  使用 xTaskGetTickCount 计算超时；即使在毫秒级调度延迟下也足够精确。
 * @return pdPASS 成功；pdFAIL 超时（硬件故障，应记录错误）
 */
static BaseType_t usart0_send_byte_timeout(uint8_t byte, TickType_t timeout_ticks)
{
    TickType_t start = xTaskGetTickCount();
    while (usart_flag_get(USART0, USART_FLAG_TBE) == RESET) {
        if ((xTaskGetTickCount() - start) > timeout_ticks) {
            return pdFAIL;
        }
    }
    usart_data_transmit(USART0, byte);
    return pdPASS;
}

/**
 * @brief 发送以 '\0' 结尾的字符串，每个字节独立超时
 * @return pdPASS 全部成功；pdFAIL 任一字节超时（已发送部分不回滚）
 */
static BaseType_t usart0_send_string_timeout(const char *str, TickType_t per_byte_timeout)
{
    while (*str != '\0') {
        if (usart0_send_byte_timeout((uint8_t)(*str), per_byte_timeout) != pdPASS) {
            return pdFAIL;
        }
        str++;
    }
    return pdPASS;
}


/* ============================================================================
 *                     独立看门狗 FWDGT（GD32 的 IWDG）
 * ============================================================================ */

/**
 * @brief 初始化独立看门狗（超时约 2 秒）
 * @note  fwdgt_config() 内部会自动使能写保护、写配置、再关闭写保护，
 *        无需像 STM32 那样手工调用 xxx_write_access_enable/disable。
 *        一旦启动不可关闭，只有系统复位才能停止。
 */
static void iwdg_init_config(void)
{
    /* 启动内部 40 kHz RC 振荡器（FWDGT 的时钟源） */
    rcu_osci_on(RCU_IRC40K);
    (void)rcu_osci_stab_wait(RCU_IRC40K);

    /* 配置：预分频 = 64  → 40 kHz / 64 = 625 Hz
     *       重载值 = 1250 → 约 2 秒超时 */
    fwdgt_config(IWDG_RELOAD_VAL, FWDGT_PSC_DIV64);

    /* 启动 FWDGT：此后开始计数 */
    fwdgt_enable();
}

/**
 * @brief 喂狗：重载 FWDGT 计数器
 */
static void iwdg_feed(void)
{
    fwdgt_counter_reload();
}


/* ============================================================================
 *                      USART + DMA 硬件初始化与启动
 * ============================================================================ */

/**
 * @brief 硬件层初始化：GPIO、USART 参数、DMA 通道。
 * @note  本函数不使能中断/接收，真正的“开始接收”在 usart_rx_start() 中完成。
 *        这样保证同步原语已创建后再放行中断，避免 ISR 里访问 NULL 信号量。
 */
static void usart_dma_hw_init(void)
{
    /* ---- 1. 时钟 ---- */
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_DMA0);

    /* ---- 2. GPIO：TX 复用推挽，RX 浮空输入 ---- */
    gpio_init(GPIOA, GPIO_MODE_AF_PP,        GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING,  GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    /* ---- 3. USART 参数 ---- */
    usart_baudrate_set    (USART0, 115200U);
    usart_word_length_set (USART0, USART_WL_8BIT);
    usart_stop_bit_set    (USART0, USART_STB_1BIT);
    usart_parity_config   (USART0, USART_PM_NONE);
    usart_receive_config  (USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config (USART0, USART_TRANSMIT_ENABLE);

    /* ---- 4. DMA 通道：USART0_RX 固定对应 DMA0 通道 4 ---- */
    dma_parameter_struct dma_cfg;
    dma_deinit(DMA0, DMA_CH4);
    dma_cfg.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_cfg.memory_addr  = (uint32_t)s_dma_rx_buf;
    dma_cfg.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_cfg.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_cfg.number       = RX_BUF_SIZE;
    dma_cfg.periph_addr  = (uint32_t)&USART_DATA(USART0);
    dma_cfg.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_cfg.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_cfg.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH4, &dma_cfg);

    /* 循环模式：DMA 写满缓冲区后自动绕回，与我们的环形逻辑配合 */
    dma_circulation_enable(DMA0, DMA_CH4);

    /* 注意：这里不 enable DMA 通道，留给 usart_rx_start() 统一启动 */
}

/**
 * @brief 放行接收：使能 DMA 接收请求、空闲中断、NVIC 与 USART。
 * @note  必须在 s_uart_rx_sem 创建成功之后调用。
 */
static void usart_rx_start(void)
{
    /* 使能串口 DMA 接收请求 */
    usart_dma_receive_config(USART0, USART_RECEIVE_DMA_ENABLE);

    /* 使能空闲中断（一帧结束但未满缓冲区时触发） */
    usart_interrupt_enable(USART0, USART_INT_IDLE);

    /* NVIC 抢占优先级 = 5：
     * FreeRTOS Cortex-M 端口要求调用 FromISR API 的 ISR 抢占优先级
     * 数值 >= configMAX_SYSCALL_INTERRUPT_PRIORITY 对应的抢占级别。
     * 若你修改了该配置项，请同步调整这里的数字。 */
    nvic_irq_enable(USART0_IRQn, 5, 0);

    /* 启动 DMA 通道 */
    dma_channel_enable(DMA0, DMA_CH4);

    /* 最后使能 USART */
    usart_enable(USART0);
}


/* ============================================================================
 *                          USART0 空闲中断
 * ============================================================================ */

void USART0_IRQHandler(void)
{
    /* 只处理空闲中断 */
    if (usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE) != RESET) {
        /* GD32 清空闲标志的“坑点”：必须先读 STAT 再读 DATA，
         * 顺序反了或漏读 DATA，标志位清不掉，中断会持续触发。 */
        volatile uint32_t stat = USART_STAT(USART0);
        volatile uint32_t data = USART_DATA(USART0);
        (void)stat; (void)data;

        /* 计算 DMA 已写入的位置。
         * dma_transfer_number_get 返回“剩余未传输数量”，
         * 因此已传输 = RX_BUF_SIZE - remain。
         * 当 remain==0 时（缓冲区刚写满绕回）已传输 = RX_BUF_SIZE，
         * 直接使用会导致 write_index 越界，必须 & (RX_BUF_SIZE-1) 归一化。 */
        uint16_t remain = (uint16_t)dma_transfer_number_get(DMA0, DMA_CH4);
        s_write_index = (uint16_t)((RX_BUF_SIZE - remain) & (RX_BUF_SIZE - 1U));

        /* 通知解析任务 */
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(s_uart_rx_sem, &woken);
        portYIELD_FROM_ISR(woken);
    }
}


/* ============================================================================
 *                       环形缓冲区积压检查
 * ============================================================================ */

/**
 * @brief 检查环形缓冲区积压是否超过安全阈值
 * @return pdTRUE 表示已发生/即将发生溢出，调用方必须复位状态机
 * @note  积压 = (write - read) mod RX_BUF_SIZE。
 *        采用无符号环形差值运算，避免 uint16 溢出问题。
 */
static BaseType_t rx_buffer_check_overflow(void)
{
    uint16_t pending;
		if(s_write_index >= s_read_index){
				pending = (uint16_t)((s_write_index - s_read_index) & (RX_BUF_SIZE - 1U));
		}
		else{
				pending = RX_BUF_SIZE - s_read_index +s_write_index;
		}
		
    if (pending > RX_OVERFLOW_THRESHOLD) {
        /* 积压过深：DMA 很可能已绕回覆盖了尚未解析的旧数据，
         * 继续按字节解析会从帧中间开始，导致状态机错乱。
         * 正确做法是直接丢弃全部积压，把读指针追到写指针，
         * 并让调用方复位状态机，等待下一个帧头。 */
        s_read_index  = s_write_index;
        s_diag_overflow_cnt++;
        return pdTRUE;
    }
    return pdFALSE;
}




/* ============================================================================
 *                          任务 1：心跳 / OLED 显示
 * ============================================================================ */

/**
 * @brief 心跳任务：初始化 OLED，周期刷新 LED 状态与诊断信息。
 * @note  
 */

static void vTaskHeartbeat(void *pvParameters)
{
    (void)pvParameters;

    /* ---- 1. OLED 初始化 ---- */
    uint8_t oled_ready = 0;
    for (uint32_t i = 0; i < OLED_INIT_RETRY_MAX && !oled_ready; i++) {
        if (OLED_Init() == I2C_OK) {
            oled_ready = 1;
            iwdg_feed();
        } else {
            iwdg_feed();
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    /* ---- 2. 传感器初始化 ---- */
    iwdg_feed();
    sensor_status_t sen_st = sensor_init();
    iwdg_feed();
    {
        char dbg[24];
        snprintf(dbg, sizeof(dbg), "[SEN] init=%d\r\n", (int)sen_st);
        (void)usart0_send_string_timeout(dbg, pdMS_TO_TICKS(5));
    }

    sensor_data_t sen;
    memset(&sen, 0, sizeof(sen));

    TickType_t last_sen_tick  = 0;
    TickType_t last_diag_tick = 0;

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        /* ---- 3. 每秒读一次传感器 + 串口打印 ---- */
        if ((now - last_sen_tick) >= pdMS_TO_TICKS(1000)) {
            last_sen_tick = now;

            sensor_data_t tmp;
            memset(&tmp, 0, sizeof(tmp));
            sensor_status_t r = sensor_read(&tmp);

            /* 串口诊断打印 */
            if(uart0_aht20_bmp280 == 1) {
                uart0_aht20_bmp280 = 0;
								char dbg[96];
                int t10 = (int)(tmp.temperature * 10.0f);
                int h10 = (int)(tmp.humidity    * 10.0f);
                int p10 = (int)(tmp.pressure    * 10.0f);
                int t_abs = t10 < 0 ? -t10 : t10;
                snprintf(dbg, sizeof(dbg),
                         "[SEN] r=%d V=0x%02X T=%s%d.%dC H=%d.%d%% P=%d.%dhPa\r\n",
                         (int)r, (unsigned)tmp.valid,
                         t10 < 0 ? "-" : "", t_abs / 10, t_abs % 10,
                         h10 / 10, h10 % 10,
                         p10 / 10, p10 % 10);
                (void)usart0_send_string_timeout(dbg, pdMS_TO_TICKS(5));
								
            }

            /* 更新显示（读失败时保留上次有效值，OLED 不闪 --） */
            if (r == SENSOR_OK || r == SENSOR_ERR_DATA) {
                if (tmp.valid & SENSOR_VALID_AHT20) {
                    sen.temperature = tmp.temperature;
                    sen.humidity    = tmp.humidity;
                }
                if (tmp.valid & SENSOR_VALID_BMP280) {
                    sen.pressure = tmp.pressure;
                    sen.altitude = tmp.altitude;
                    if (!(tmp.valid & SENSOR_VALID_AHT20)) {
                        sen.temperature = tmp.temperature;
                    }
                }
                sen.valid = tmp.valid;
            }
        }

        /* ---- 4. OLED 每 200ms 刷一次传感器数据 ---- */
        if (oled_ready) {
            char line[24];

            /* Page 0-1: 温度 8x16 */
            if (sen.valid & SENSOR_VALID_AHT20) {
                strcpy(line, "T:");
                append_dec1(line + 2, sen.temperature);
                strcat(line, "C");
            } else {
                strcpy(line, "T:--.-C");
            }
            pad_to(line, 16);
            OLED_ShowStr(0, 0, line, OLED_FONT_8X16);

            /* Page 2-3: 湿度 8x16 */
            if (sen.valid & SENSOR_VALID_AHT20) {
                strcpy(line, "H:");
                append_dec1(line + 2, sen.humidity);
                strcat(line, "%");
            } else {
                strcpy(line, "H:--.-%");
            }
            pad_to(line, 16);
            OLED_ShowStr(0, 2, line, OLED_FONT_8X16);

            /* Page 4: 气压 6x8 */
            if (sen.valid & SENSOR_VALID_BMP280) {
                strcpy(line, "P:");
                append_dec1(line + 2, sen.pressure);
                strcat(line, "hPa");
            } else {
                strcpy(line, "P:----.-hPa");
            }
            pad_to(line, 21);
            OLED_ShowStr(0, 4, line, OLED_FONT_6X8);

            /* Page 5: 海拔 6x8 */
            if (sen.valid & SENSOR_VALID_BMP280) {
                strcpy(line, "Alt:");
                append_dec1(line + 4, sen.altitude);
                strcat(line, "m");
            } else {
                strcpy(line, "Alt:----.-m");
            }
            pad_to(line, 21);
            OLED_ShowStr(0, 5, line, OLED_FONT_6X8);
        }

        /* ---- 5. 每秒刷新诊断页（Page 6/7） ---- */
        if ((now - last_diag_tick) >= pdMS_TO_TICKS(1000)) {
            last_diag_tick = now;
            if (oled_ready) {
                char line[24];
                char num[11];

                /* Page 6: LED 状态 + 传感器 init 结果 */
                strcpy(line, "LED:");
                strcat(line, s_led_state ? "ON " : "OFF");
                strcat(line, " SEN:");
                strcat(line,
                       sen_st == SENSOR_OK            ? "OK"   :
                       sen_st == SENSOR_ERR_I2C       ? "I2C"  :
                       sen_st == SENSOR_ERR_NOT_READY ? "NRDY" : "ERR");
                pad_to(line, 21);
                OLED_ShowStr(0, 6, line, OLED_FONT_6X8);

                /* Page 7: 串口诊断计数 */
                strcpy(line, "OK:");
                u32_to_dec(num, s_diag_frame_ok_cnt);
                strcat(line, num);
                strcat(line, " ERR:");
                u32_to_dec(num, s_diag_checksum_err_cnt);
                strcat(line, num);
                pad_to(line, 21);
                OLED_ShowStr(0, 7, line, OLED_FONT_6X8);
            }
        }

        s_hb_tick++;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ============================================================================
 *                     任务 2：状态机协议解析
 * ============================================================================ */

/* 协议格式：0xAA 0x55 <CMD> <CHECK>，CHECK = (0xAA + 0x55 + CMD) & 0xFF */
typedef enum {
    STATE_WAIT_HEADER1 = 0,   /* 等待 0xAA */
    STATE_WAIT_HEADER2,       /* 等待 0x55 */
    STATE_WAIT_CMD,           /* 等待命令字 */
    STATE_WAIT_CHECK          /* 等待校验和 */
} ParseState_t;

/**
 * @brief 串口协议解析任务
 * @note  解析过程中不阻塞、不发送长时间字符串；发送失败记录计数后继续。
 */
static void vTaskUartProcess(void *pvParameters)
{
    (void)pvParameters;

    ParseState_t state        = STATE_WAIT_HEADER1;
    TickType_t   last_rx_tick = xTaskGetTickCount();   /* 最近一次收到字节的时间 */
    uint8_t      rx_cmd       = 0;
    uint8_t      rx_check     = 0;

    for (;;) {
        /* 每次循环递增健康心跳：说明本任务活着且在正常轮转 */
        s_uartproc_tick++;

        /* 带超时地等待信号量：
         *  - pdPASS  ：有新数据 → 立即解析
         *  - pdFALSE ：总线静默超过 50ms → 复位状态机，防止卡在中间状态 */
        if (xSemaphoreTake(s_uart_rx_sem, pdMS_TO_TICKS(PARSER_IDLE_RESET_MS)) == pdPASS) {

            /* ---- 第一步：积压溢出检查（必须在消费前做） ---- */
            if (rx_buffer_check_overflow()) {
                /* 已强制丢弃积压 → 状态机必须复位，重新寻找帧头 */
                state = STATE_WAIT_HEADER1;
                last_rx_tick = xTaskGetTickCount();
                continue;
            }

            /* ---- 第二步：记录本次处理的上界（快照），避免循环中写指针变化 ---- */
            uint16_t snapshot_end = s_write_index;

            /* ---- 第三步：逐字节消费（零拷贝，直接解析 DMA 缓冲区） ---- */
            while (s_read_index != snapshot_end) {
                uint8_t byte = s_dma_rx_buf[s_read_index];
                s_read_index = (uint16_t)((s_read_index + 1U) & (RX_BUF_SIZE - 1U));
                last_rx_tick = xTaskGetTickCount();

                switch (state) {
                case STATE_WAIT_HEADER1:
                    if (byte == 0xAA) state = STATE_WAIT_HEADER2;
                    break;

                case STATE_WAIT_HEADER2:
                    /* 收到 0x55 进入下一状态；否则回原点重新找 */
                    state = (byte == 0x55) ? STATE_WAIT_CMD : STATE_WAIT_HEADER1;
                    break;

                case STATE_WAIT_CMD:
                    rx_cmd   = byte;
                    rx_check = (uint8_t)(0xAAu + 0x55u + rx_cmd);  /* 累加校验 */
                    state    = STATE_WAIT_CHECK;
                    break;

                case STATE_WAIT_CHECK:
                    if (byte == rx_check) {
                        /* 校验通过：执行命令 */
                        s_diag_frame_ok_cnt++;

                        if (rx_cmd == 0x01) {
                            gpio_bit_reset(GPIOC, GPIO_PIN_13);          /* 开灯 */
                            s_led_state = 1;
                            (void)usart0_send_string_timeout(
                                "CMD OK: LED ON\r\n",
                                pdMS_TO_TICKS(UART_TX_BYTE_TIMEOUT_MS));
                        } else if (rx_cmd == 0x02) {
                            gpio_bit_set(GPIOC, GPIO_PIN_13);            /* 关灯 */
                            s_led_state = 0;
                            (void)usart0_send_string_timeout(
                                "CMD OK: LED OFF\r\n",
                                pdMS_TO_TICKS(UART_TX_BYTE_TIMEOUT_MS));
                        } else if(rx_cmd == 0x04) {
														(void)usart0_send_string_timeout(
                                "\r\n[APP] OTA command received! Rebooting to Bootloader...\r\n",
                                pdMS_TO_TICKS(UART_TX_BYTE_TIMEOUT_MS));
														app_trigger_ota();
													
												} else if(rx_cmd == 0x05){
														uart0_aht20_bmp280 = 1;
												
												} else {
                            /* 未知命令：记录并静默丢弃 */
                            (void)usart0_send_string_timeout(
                                "CMD ERR: UNKNOWN\r\n",
                                pdMS_TO_TICKS(UART_TX_BYTE_TIMEOUT_MS));
                        }
                    } else {
                        /* 校验失败：累计计数（供现场诊断观察链路质量） */
                        s_diag_checksum_err_cnt++;
                        (void)usart0_send_string_timeout(
                            "CMD ERR: CHECKSUM\r\n",
                            pdMS_TO_TICKS(UART_TX_BYTE_TIMEOUT_MS));
                    }

                    /* 无论成功失败，回到帧头等待状态准备下一帧 */
                    state = STATE_WAIT_HEADER1;
                    break;
                }
            }

        } else {
            /* ---- 超时：总线静默超过 PARSER_IDLE_RESET_MS ---- */
            TickType_t now = xTaskGetTickCount();
            if ((now - last_rx_tick) > pdMS_TO_TICKS(PARSER_IDLE_RESET_MS)) {
                /* 复位状态机：截断帧 / 孤立噪声不会再“卡”在中间状态 */
                state = STATE_WAIT_HEADER1;
                last_rx_tick = now;
            }
        }
    }
}


/* ============================================================================
 *                   任务 3：任务健康监控与看门狗喂狗
 * ============================================================================ */

/**
 * @brief 监控任务：仅当心跳任务和解析任务都在“活着”时才喂狗。
 * @note  这是工业设计的关键点——喂狗不能无脑进行，否则任务卡死时看门狗失效。
 *        本任务优先级设为最低应用优先级，避免抢占实时任务。
 */
static void vTaskMonitor(void *pvParameters)
{
    (void)pvParameters;

    uint32_t last_hb   = s_hb_tick;
    uint32_t last_proc = s_uartproc_tick;
    uint32_t hb_stall  = 0;
    uint32_t pr_stall  = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(200));

        uint32_t hb   = s_hb_tick;
        uint32_t proc = s_uartproc_tick;

        if (hb != last_hb)        { last_hb = hb;   hb_stall = 0; }
        else if (hb_stall < 100U) { hb_stall++; }

        if (proc != last_proc)    { last_proc = proc; pr_stall = 0; }
        else if (pr_stall < 100U) { pr_stall++; }

        /* 5 秒容忍窗口（25 × 200ms）：
         * sensor_init 阻塞 2.7s 期间心跳任务不推进也不会误复位。
         * 真正卡死超过 5s 才停喂 → IWDG 4.8s 后复位。 */
        if (hb_stall < 25U && pr_stall < 25U) {
            iwdg_feed();
        }
    }
}


/* ============================================================================
 *                                  主函数
 * ============================================================================ */

int main(void)
{
		/* 必须放在 main 的第一行 */
    SCB->VTOR = 0x08004000U; 
	
	
    /* ---- 0. 看门狗尽早启动：任何初始化失败 → 不喂狗 → 复位重试 ---- */
    iwdg_init_config();

    /* ---- 1. NVIC 优先级分组（4 位抢占 + 0 位子优先级，与 FreeRTOS 端口一致） ---- */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    /* ---- 2. LED GPIO ---- */
    rcu_periph_clock_enable(RCU_GPIOC);
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_bit_set(GPIOC, GPIO_PIN_13);   /* 默认灭（高电平灭，视电路而定） */

    /* ---- 3. USART + DMA 硬件配置（不使能中断） ---- */
    usart_dma_hw_init();

    /* ---- 4. 创建同步原语（必须先于使能中断） ---- */
    s_uart_rx_sem = xSemaphoreCreateBinary();
    configASSERT(s_uart_rx_sem != NULL);

    if (s_uart_rx_sem == NULL) {
        /* 创建失败：不喂狗，等待 FWDGT 复位系统重新尝试 */
        for (;;) { /* 空循环等待复位 */ }
    }

		//i2c
		i2c_bus_init();
		(void)sensor_os_init();
		
    /* ---- 5. 创建任务（全部检查返回值） ---- */
    BaseType_t rc;
    rc = xTaskCreate(vTaskMonitor,    "Monitor",  TASK_STACK_MONITOR,
                     NULL, PRIO_MONITOR,   NULL);
    configASSERT(rc == pdPASS);

    rc = xTaskCreate(vTaskHeartbeat,  "Heartbeat", TASK_STACK_HEARTBEAT,
                     NULL, PRIO_HEARTBEAT, NULL);
    configASSERT(rc == pdPASS);

    rc = xTaskCreate(vTaskUartProcess, "UartProc", TASK_STACK_UARTPROC,
                     NULL, PRIO_UARTPROC,  NULL);
    configASSERT(rc == pdPASS);

		
    /* ---- 6. 放行串口中断与接收（在同步原语已就绪之后） ---- */
    usart_rx_start();

    /* ---- 7. 启动调度器（不返回） ---- */
    vTaskStartScheduler();

    /* 正常情况下不会到这里；万一调度器启动失败，让 FWDGT 复位重试 */
    for (;;) { /* 等待复位 */ }
}