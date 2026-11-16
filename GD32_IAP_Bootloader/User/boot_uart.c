#include "boot_uart.h"
#include "gd32f10x.h"
#include <stdio.h>

#define RX_BUF_SIZE 2048 
static uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

static void dwt_init(void)
{
    if (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) return;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void boot_uart_init(uint32_t baudrate)
{
    dwt_init();
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_USART0);

    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
    gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

    usart_deinit(USART0);
    usart_baudrate_set(USART0, baudrate);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

    nvic_irq_enable(USART0_IRQn, 1, 0);
    usart_interrupt_enable(USART0, USART_INT_RBNE);
    usart_enable(USART0);
}

void USART0_IRQHandler(void)
{
    if (RESET != usart_flag_get(USART0, USART_FLAG_ORERR)) {
        volatile uint32_t tmp;
        tmp = USART_STAT(USART0); 
        tmp = USART_DATA(USART0); 
        (void)tmp;
    }

    if (RESET != usart_flag_get(USART0, USART_FLAG_RBNE)) {
        uint8_t data = (uint8_t)USART_DATA(USART0);
        uint16_t next = (rx_head + 1) % RX_BUF_SIZE;
        if (next != rx_tail) {
            rx_buf[rx_head] = data;
            rx_head = next;
        }
    }
}

uint8_t boot_uart_read_byte(uint8_t *ch, uint32_t timeout_ms)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t timeout_cycles = timeout_ms * (SystemCoreClock / 1000U);
    
    while (1) {
        if (rx_head != rx_tail) {
            *ch = rx_buf[rx_tail];
            rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
            return 1;
        }
        if ((DWT->CYCCNT - start) >= timeout_cycles) {
            return 0;
        }
    }
}

void boot_uart_putc(uint8_t ch)
{
    usart_data_transmit(USART0, ch);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE)) { }
}

void boot_uart_write(const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        boot_uart_putc(buf[i]);
    }
}

int fputc(int ch, FILE *f) { boot_uart_putc((uint8_t)ch); return ch; }
int _write(int file, char *ptr, int len) { for (int i=0; i<len; i++) boot_uart_putc(ptr[i]); return len; }