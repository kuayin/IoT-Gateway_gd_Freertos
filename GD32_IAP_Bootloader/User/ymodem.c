#include "ymodem.h"
#include "boot_uart.h"
#include "w25q64.h"
#include <stdio.h>

#define SOH  0x01
#define STX  0x02
#define EOT  0x04
#define ACK  0x06
#define NAK  0x15
#define CAN  0x18
#define C    0x43

static uint8_t pkt_buf[1024 + 5];
uint32_t fw_total_size = 0;

static int write_flash_chunk(uint32_t addr, const uint8_t *data, uint32_t len)
{
    while (len > 0) {
        uint32_t page_off = addr % W25Q64_PAGE_SIZE;
        uint32_t write_len = W25Q64_PAGE_SIZE - page_off;
        if (write_len > len) write_len = len;
        
        if (w25q64_page_program(addr, data, write_len) != W25Q64_OK) return -1;
        if (w25q64_wait_ready(100) != W25Q64_OK) return -1;
        
        addr += write_len;
        data += write_len;
        len -= write_len;
    }
    return 0;
}

static uint16_t receive_packet(uint8_t *buf)
{
    uint8_t ch;
    while (1) {
        if (!boot_uart_read_byte(&ch, 3000)) return 0xFFFE; 
        
        uint16_t data_len = 0;
        if (ch == SOH)      data_len = 128;
        else if (ch == STX) data_len = 1024;
        else if (ch == EOT) return 0;
        else if (ch == CAN) return 0xFFFF;
        else continue; 

        uint16_t read_count = 2 + data_len + 2;
        for (uint16_t i = 0; i < read_count; i++) {
            if (!boot_uart_read_byte(&buf[i], 2000)) {
                return 0xFFFD;
            }
        }
        return data_len;
    }
}

ymodem_status_t ymodem_receive(void)
{
    uint16_t len;
    uint8_t packet_num = 1;
    uint32_t write_addr = W25Q64_UPDATE_START_ADDR;
    uint32_t current_erased_sector = 0xFFFFFFFFU;
    uint8_t retry = 0;
    uint8_t error_count = 0;

    printf("[Ymodem] Wait...\r\n");

    while (retry < 10) {
        boot_uart_putc(C);
        len = receive_packet(pkt_buf);
        
        if (len == 128 || len == 1024) {
            if (pkt_buf[0] == 0 && pkt_buf[1] == 0xFF) { 
                boot_uart_putc(ACK);
                boot_uart_putc(C);
                retry = 0;
                break;
            }
        } else if (len == 0xFFFF) {
            return YMODEM_ERR_CANCEL;
        }
        retry++;
    }
    
    if (retry >= 10) return YMODEM_ERR_TIMEOUT;

    fw_total_size = 0;

    while (1) {
        len = receive_packet(pkt_buf);
        
        if (len == 0) { // 收到 EOT
            boot_uart_putc(ACK);
            boot_uart_putc(C); 
            
            len = receive_packet(pkt_buf);
            if (len == 128 || len == 1024) {
                boot_uart_putc(ACK);
            }
            break;
        }
        
        if (len == 0xFFFF) return YMODEM_ERR_CANCEL;
        
        if (len == 0xFFFE || len == 0xFFFD) { 
            error_count++;
            if (error_count > 10) return YMODEM_ERR_TIMEOUT;
            boot_uart_putc(NAK);
            continue;
        }
        
        if (pkt_buf[0] != packet_num || pkt_buf[1] != (uint8_t)(~packet_num)) {
            error_count++;
            if (error_count > 10) return YMODEM_ERR_TIMEOUT;
            boot_uart_putc(NAK);
            continue;
        }

        error_count = 0;

        /* 先回 ACK 稳定上位机 */
        boot_uart_putc(ACK); 
        packet_num++;
        fw_total_size += len;
        
        uint32_t sector_addr = write_addr & 0xFFFFF000U;
        if (sector_addr != current_erased_sector) {
            w25q64_sector_erase(sector_addr);
            w25q64_wait_ready(500);
            current_erased_sector = sector_addr;
        }
        
        if (write_flash_chunk(write_addr, &pkt_buf[2], len) != 0) {
            boot_uart_putc(CAN); boot_uart_putc(CAN);
            return YMODEM_ERR_FLASH;
        }
        
        write_addr += len;
        
        /* ========================================================= */
        /* 实验三：加上这句进度打印，观察它是否会由于阻塞导致串口溢出 */
        printf("\r[OK] Pkt %d written...   ", packet_num - 1); 
        /* ========================================================= */
    }
    
    printf("\r\n[Ymodem] Transfer Complete!\r\n");
    return YMODEM_OK;
}