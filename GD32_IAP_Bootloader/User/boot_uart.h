#ifndef BOOT_UART_H
#define BOOT_UART_H

#include <stdint.h>

void boot_uart_init(uint32_t baudrate);
void boot_uart_write(const uint8_t *buf, uint16_t len);
void boot_uart_putc(uint8_t ch);
uint8_t boot_uart_read_byte(uint8_t *ch, uint32_t timeout_ms);

#endif